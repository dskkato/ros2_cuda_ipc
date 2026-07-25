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
and the returned array keeps the native image lease alive until it is released.

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
    # work is complete. The DLPack tensor owns a retained native image view.
```

CuPy and PyTorch both consume the same DLPack export. The path is zero-copy and
preserves shape, dtype, device, and non-contiguous strides. The ownership chain
is:

```text
framework tensor/array
    -> DLPack managed tensor or CuPy owner
        -> retained native ImageView
            -> imported CUDA resource
            -> LeaseHandle
                -> shared-memory slot lease
```

Closing or destroying the original `ImageView` does not release this retained
owner. The final framework object releases it through the CuPy owner or the
DLPack managed-tensor deleter. An unconsumed DLPack capsule also releases its
owner when the capsule is destroyed, and a capsule is single-use.

The producer ready event and consumer work completion are separate. The
consumer stream passed by CuPy or DLPack is made to wait for the producer-ready
event before the framework object is returned. This does not prove that later
consumer kernels have completed. Keep the framework object alive until all
asynchronous CUDA work using its memory has completed; this PR does not release
leases automatically from stream completion.

DLPack support emits the legacy `dltensor` capsule by default for compatibility
with current CuPy and PyTorch consumers. When `max_version >= (1, 0)` is
requested, it emits the versioned DLPack v1.0 capsule. The native extension
does not import or link against either framework. The supported representation
is CUDA `GpuImage` rank 3 with the dtypes defined by `ros2_cuda_ipc_msgs`; CPU
fallback, publishing, PointCloud2, and the DLPack C exchange API are not
included.

Do not close or release the last array/view owner until all asynchronous CUDA
work using the array has completed. A mapper should be reused across callbacks
so its C++ lease and IPC import caches remain effective.

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
