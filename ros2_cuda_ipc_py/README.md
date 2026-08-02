# ros2_cuda_ipc_py

Python subscriber support for `ros2_cuda_ipc`. The package keeps ROS 2
subscription and QoS handling in `rclpy`, while the pybind11 extension calls
the existing C++ subscriber mapper. DLPack is the framework-neutral tensor
interoperability path used by CuPy, PyTorch, and other compatible consumers.

## Requirements

Build with a sourced ROS 2 environment, CUDA Driver API headers/libraries,
`ros2_cuda_ipc_core`, the `pybind11-dev` package, and an installed DLPack CMake
package exposing `dlpack::dlpack`. CuPy and PyTorch are optional separate
runtime dependencies and must match the installed CUDA toolkit/driver
compatibility requirements. If DLPack is not installed, explicitly enable the
network fetch fallback with
`-DROS2_CUDA_IPC_PY_FETCH_DLPACK=ON`.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to ros2_cuda_ipc_py
source install/setup.bash
```

### CuPy runtime environment

CuPy is an optional runtime dependency. Create the virtual environment with
system site packages enabled so that ROS 2 Python packages and system packages
such as `python3-empy` remain visible from the virtual environment:

```bash
sudo apt install python3-venv python3-empy

# Replace `lyrical` with the ROS 2 distribution installed on the system.
source /opt/ros/lyrical/setup.bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
source install/setup.bash

python -m pip install --upgrade pip
# Select the wheel matching the installed CUDA major version.
python -m pip install cupy-cuda12x  # CUDA 12.x
# python -m pip install cupy-cuda13x  # CUDA 13.x
```

Verify the environment before running the subscriber:

```bash
python -c 'import em, rclpy, cupy; print(cupy.__version__)'
python -m cupy.show_config
ros2 run ros2_cuda_ipc_py gpu_image_cupy_subscriber.py
```

Do not install both `cupy` and a `cupy-cudaXX` package. PyTorch setup is not
covered by this procedure.

## Usage

```python
import cupy as cp
from ros2_cuda_ipc_py import ImageMapper

mapper = ImageMapper()  # keep one mapper for the subscriber's lifetime

def callback(msg):
    image = mapper.map(msg)
    array = cp.from_dlpack(image)
    # array is a zero-copy view of the imported allocation.
    consume(array)
    cp.cuda.get_current_stream().synchronize()
```

`cupy.from_dlpack(image)` consumes the standard DLPack producer protocol. It
preserves the message's `(rows, cols, channels)` shape, byte strides, and dtype,
and the returned array keeps the native image buffer reference alive until it is released.

`ImageView` is a typed DLPack projection. It exposes projection metadata such
as `byte_size`, `device_id`, `shape`, `strides`, `dtype`, `encoding`, and
`frame_id`, but it never exposes an unbound raw device pointer. The regular
raw-pointer path is `BufferMapper.map(message, stream)`, which returns a
`ReadHandle`.

Framework-neutral DLPack consumers use the standard producer protocol:

```python
import torch
from ros2_cuda_ipc_py import ImageMapper

mapper = ImageMapper()

def callback(message):
    image = mapper.map(message)
    tensor = torch.from_dlpack(image)

    consumer_stream = torch.cuda.current_stream(device=image.device_id)
    with torch.cuda.stream(consumer_stream):
        result = model(tensor)

    consumer_stream.synchronize()
    # Keep tensor (and any result that aliases it) alive until asynchronous
    # work is complete. The DLPack tensor owns the retained native read state.
```

CuPy and PyTorch both consume the same DLPack export. The path is zero-copy and
preserves shape, dtype, device, and non-contiguous strides. The ownership chain
is:

```text
framework tensor/array
    -> DLPack managed tensor or CuPy owner
        -> retained native read state
            -> imported CUDA resource
            -> publication buffer reference
                -> shared-memory slot refcount
```

The first successful `__dlpack__(stream)` transfers the imported resource and
publication buffer reference to an export-specific read state. The mapped object is then
consumed: a second `__dlpack__()` and any publication-lifetime-dependent or
raw-pointer operation are rejected, while metadata remains readable. An
unconsumed capsule also releases its owner when the capsule is destroyed, and
a mapped object permits one export only.

The producer ready event and consumer work completion are separate. For
DLPack, `__dlpack__(stream)` creates the completion event before publishing the
capsule, then enqueues the producer-ready wait on the requested stream. The
capsule deleter records completion on that same stream and puts the event,
imported-resource reference, and buffer reference into an internal deferred
queue. A process-internal worker waits for queue entries sequentially with
`cuEventSynchronize` and then releases the handle-specific references; import
cache entry lifetime is managed independently by the import cache.

`stream=-1` is rejected because completion cannot be managed without a
consumer stream. `None`, CUDA legacy default stream, per-thread default stream,
and a valid CUDA stream pointer are supported. The stream bound at export must
remain valid until the capsule deleter has recorded the completion event.

DLPack support emits the legacy `dltensor` capsule by default for compatibility
with current CuPy and PyTorch consumers. When `max_version >= (1, 0)` is
requested, it emits the versioned DLPack v1.0 capsule. The native extension
does not import or link against either framework. The supported representation
is CUDA `GpuImage` rank 3 with the dtypes defined by `ros2_cuda_ipc_msgs`; CPU
fallback, publishing, PointCloud2, and the DLPack C exchange API are not
included.

Do not close or release the last array/view owner until all asynchronous CUDA
work using the array has completed. A mapper should be reused across callbacks
so its internal buffer metadata mapping and IPC import caches remain effective. C++
`BufferMapper.map()` returns an optional read; the detailed reason for a failed
map is logged internally, while callers only receive the empty result.

The descriptor boundary also accepts a mapping with `GpuImage` fields. This is
useful for tests and keeps the extension independent of generated `rclpy`
message internals. The Python adapter converts generated ROS messages to that
descriptor and copies only small metadata/handle fields, never the GPU payload.

The current Python API supports `GpuImage`/`BufferCore` subscription mapping
and DLPack consumers such as CuPy and PyTorch. CuPy and PyTorch remain optional
runtime dependencies; users of only the native view or another DLPack consumer
do not need CuPy installed. The development integration tests exercise
PyTorch 2.6 when CUDA is available and skip framework-specific tests when a
dependency or CUDA device is unavailable.

A runnable subscriber is installed as:

```bash
ros2 run ros2_cuda_ipc_py gpu_image_cupy_subscriber.py --ros-args -p use_sim_time:=false
```

The DLPack/Torch example is installed as:

```bash
ros2 run ros2_cuda_ipc_py gpu_image_torch_subscriber.py --ros-args -p use_sim_time:=false
```
