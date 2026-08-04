# ros2_cuda_ipc

`ros2_cuda_ipc` lets ROS 2 processes share GPU-resident data without copying the
full payload through host memory.

The project provides:

- CUDA VMM-FD-based inter-process GPU memory sharing;
- CUDA event-based stream synchronization;
- explicit multi-consumer buffer lifetime management;
- typed C++ APIs for images and point clouds;
- Python integration with DLPack-compatible frameworks such as PyTorch and
  CuPy;
- a multi-process demo in which one GPU image publisher feeds preview,
  encoder-like, and inference-like consumers.

> [!NOTE]
> **Related ROS 2 work**
>
> ROS 2 Lyrical introduced the `rosidl::Buffer` abstraction and a CUDA buffer
> backend. It addresses a closely related problem by integrating pluggable
> storage backends with generated ROS message types and the ROS 2 middleware.
>
> If you are using ROS 2 Lyrical or later, the upstream buffer backend is worth
> evaluating, especially when you want a ROS-native storage abstraction that is
> integrated throughout the message and middleware stack.
>
> `ros2_cuda_ipc` was developed independently and uses a standalone library
> approach with explicit GPU buffer descriptors. It may still be useful when:
>
> - supporting ROS 2 distributions before Lyrical;
> - working with existing or custom message definitions;
> - requiring explicit control over CUDA stream synchronization and
>   multi-consumer buffer lifetimes;
> - consuming GPU data directly from Python through DLPack-compatible
>   frameworks such as PyTorch and CuPy;
> - experimenting with CUDA IPC behavior independently of the middleware
>   implementation.
>
> The projects have different integration points rather than being direct
> replacements for one another. Users on Lyrical or later may want to evaluate
> both approaches based on their deployment and API requirements.
>
> See the ROS 2 Lyrical release notes for more information about
> `rosidl::Buffer` and the CUDA buffer backend:
> https://docs.ros.org/en/lyrical/Releases/Release-Lyrical-Luth.html#publish-messages-without-copying-data-using-rosidl-buffer

## What You Can Try

The main demo is `multi_process_image_fanout`.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to multi_process_image_fanout
source install/setup.bash
ros2 launch multi_process_image_fanout multi_process_image_fanout.launch.py
```

The demo publishes one GPU RGBA image and fans it out to three independent ROS
2 processes:

- `preview_node` republishes a CPU `sensor_msgs::msg::Image` for standard image tools.
- `encoder_like_node` runs GPU-side downscale/checksum work.
- `inference_like_node` runs GPU-side preprocessing/stat extraction.

Only the preview path copies the full image back to CPU memory.

## Requirements

- Ubuntu 22.04
- ROS 2 Humble or newer
- CUDA Toolkit 10.2 or newer with VMM and POSIX file-descriptor support
- A CUDA-capable NVIDIA GPU
- `colcon`

Set `CUDACXX` if CMake cannot find `nvcc` automatically:

```bash
export CUDACXX=/usr/local/cuda/bin/nvcc
```

## Useful Launch Options

```bash
ros2 launch multi_process_image_fanout multi_process_image_fanout.launch.py \
  width:=1280 \
  height:=720 \
  publish_rate_hz:=60.0 \
  block_count:=4 \
  device_index:=0
```

The library uses CUDA VMM plus POSIX file descriptors as its only memory-sharing
backend. File descriptors are distributed through a Unix domain socket and
ready-event synchronization still uses CUDA IPC event handles.

## Viewing The Demo

```bash
ros2 run rqt_image_view rqt_image_view
```

Select `/fanout/preview/image`.

You can also inspect the lightweight status topics:

```bash
ros2 topic echo /fanout/encoder_like/status
ros2 topic echo /fanout/inference_like/status
```

## Repository Layout

- `ros2_cuda_ipc_msgs`: ROS 2 message definitions for GPU-backed buffers.
- `ros2_cuda_ipc_core`: untyped CUDA memory sharing, buffer lifetime, and synchronization APIs.
- `ros2_cuda_ipc_image`: typed `GpuImage` metadata validation, `ImageReader`, `ImageReadHandle`, and image message helpers.
- `ros2_cuda_ipc_pointcloud2`: typed `GpuPointCloud2`/`PointField` validation, `PointCloud2Reader`, `PointCloud2ReadHandle`, and point-cloud message helpers.
- `ros2_cuda_ipc_py`: `rclpy`/pybind11 subscriber mapping with zero-copy DLPack integration for Python frameworks such as PyTorch and CuPy.
- `examples/multi_process_image_fanout`: primary runnable demo.
- `utils/gpu_image_transport`: utility nodes that map `GpuImage` messages to CPU image topics.
- `utils/cuda_ipc_poc`: small CUDA IPC / VMM-FD environment checks.

The package dependency direction is:

```text
ros2_cuda_ipc_image ───────┐
                           ├─> ros2_cuda_ipc_core ──> ros2_cuda_ipc_msgs
ros2_cuda_ipc_pointcloud2 ─┘          │
                                     └─> CUDA IPC / lifetime / synchronization
```

`ros2_cuda_ipc_core` does not depend on or expose image or PointCloud2 typed
objects. Those layers keep their modality-specific validation and helpers in
the package that owns the corresponding type. `ros2_cuda_ipc_pointcloud2` also
depends directly on `sensor_msgs` for `PointField`.

## More Details

- Demo usage: [examples/multi_process_image_fanout/README.md](examples/multi_process_image_fanout/README.md)
- Development notes: [DEVELOPING.md](DEVELOPING.md)
- Core design: [doc/design.md](doc/design.md)
- Python subscriber: [ros2_cuda_ipc_py/README.md](ros2_cuda_ipc_py/README.md)
- Python binding design: [doc/python-subscriber-implementation.md](doc/python-subscriber-implementation.md)
- CUDA IPC / VMM-FD checks: [utils/cuda_ipc_poc/README.md](utils/cuda_ipc_poc/README.md)

## Python and DLPack

GPU data published from C++ can be consumed directly by DLPack-compatible
Python frameworks. Reuse one mapper across callbacks so its metadata and IPC
import caches remain effective:

```python
mapper = ImageMapper()

def on_image(msg):
    image = mapper.map(msg)
    tensor = torch.from_dlpack(image)
```

Synchronization is bound to the CUDA stream selected by the consumer framework
when the DLPack object is consumed. Each mapped object is one-shot and can be
consumed by one DLPack-compatible library, such as PyTorch or CuPy, without
adding a hard dependency on a particular framework.

## Publisher API

```cpp
auto block = manager.acquire_for_publish();
if (!block) {
  return;
}
launch_gpu_work(block->device_ptr(), stream);

auto descriptor = block->prepare_publish(stream);
if (!descriptor) {
  return;
}

publisher->publish(make_message(descriptor.value()));
```

Pass the stream that carries the producer work dependency to
`prepare_publish()`. On success, publish the returned descriptor. On failure,
that block is not reused until `GpuBufferManager::reset()`.

The API accepts Driver API `CUstream` values and Runtime API `cudaStream_t`
values directly. The application retains ownership of the stream.

## License

MIT License. See [LICENSE](LICENSE).
