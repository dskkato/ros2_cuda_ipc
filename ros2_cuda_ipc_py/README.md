# ros2_cuda_ipc_py

Python subscriber support for `ros2_cuda_ipc`. The package keeps ROS 2
subscription and QoS handling in `rclpy`, while the pybind11 extension calls
the existing C++ subscriber mapper.

## Requirements

Build with a sourced ROS 2 environment, CUDA Driver API headers/libraries,
`ros2_cuda_ipc_core`, and the `pybind11-dev` package. Runtime CuPy is a
separate dependency and must match the installed CUDA major version.

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
    stream = cp.cuda.get_current_stream()
    array = image.as_cupy(stream)
    # array is a zero-copy view of the imported allocation.
    consume(array)
    stream.synchronize()  # before dropping the final array/view owner
```

`as_cupy()` enqueues the publisher ready-event wait on the specified CuPy
stream, or on the current stream when omitted. It preserves the message's
`(rows, cols, channels)` shape, byte strides, and dtype. The returned ndarray
owns an `UnownedMemory` object whose owner is an independent native image view;
this keeps the native view, imported allocation, and shared-memory lease alive
until the ndarray is released, even if the original image view is closed.

Do not close or release the last array/view owner until all asynchronous CUDA
work using the array has completed. A mapper should be reused across callbacks
so its C++ lease and IPC import caches remain effective.

The descriptor boundary also accepts a mapping with `GpuImage` fields. This is
useful for tests and keeps the extension independent of generated `rclpy`
message internals. The Python adapter converts generated ROS messages to that
descriptor and copies only small metadata/handle fields, never the GPU payload.

The current Python API supports `GpuImage`/`BufferCore` subscription mapping and
CuPy only. Python publishing, PointCloud2, PyTorch, DLPack, CPU fallback, and a
CUDA-array-interface-only adapter are not included.

A runnable subscriber is installed as:

```bash
ros2 run ros2_cuda_ipc_py gpu_image_subscriber.py --ros-args -p use_sim_time:=false
```
