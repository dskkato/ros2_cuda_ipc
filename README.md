# ros2_cuda_ipc

ROS 2 nodes can pass large GPU images between processes without copying the
full payload through host memory.

This repository provides CUDA-backed message types, mapping utilities, and a
small multi-process demo that shows one GPU image publisher feeding preview,
encoder-like, and inference-like consumers.

> [!NOTE]
> **ROS 2 Lyrical users**
>
> ROS 2 Lyrical introduced the new `rosidl::Buffer` abstraction together with the CUDA buffer backend, which provides functionality similar to this project.
>
> If you are starting a new project on ROS 2 Lyrical or later, I recommend evaluating the upstream implementation first, as it is the long-term supported solution within the ROS 2 ecosystem.
>
> This repository was developed before I became aware of the upstream effort, and it independently arrived at a very similar design. It is still useful as a reference implementation and for understanding the design trade-offs behind CUDA IPC-based zero-copy communication.
>
> See https://docs.ros.org/en/lyrical/Releases/Release-Lyrical-Luth.html#publish-messages-without-copying-data-using-rosidl-buffer

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
- CUDA Toolkit 11.8 or newer
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
  memory_backend:=cuda_ipc \
  slot_count:=4 \
  shm_name_prefix:=/ros2_cuda_ipc_fanout \
  device_index:=0
```

The memory backend can be selected at launch:

```bash
# Standard CUDA IPC path, typically used on x86_64 + dGPU systems.
ros2 launch multi_process_image_fanout multi_process_image_fanout.launch.py \
  memory_backend:=cuda_ipc

# VMM + file-descriptor path, intended for systems such as Jetson Orin.
ros2 launch multi_process_image_fanout multi_process_image_fanout.launch.py \
  memory_backend:=vmm_fd
```

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
- `ros2_cuda_ipc_core`: CUDA memory sharing plus modality-oriented mapping and view utilities.
- `ros2_cuda_ipc_py`: `rclpy`/pybind11 subscriber mapping with zero-copy CuPy views.
- `examples/multi_process_image_fanout`: primary runnable demo.
- `utils/gpu_image_transport`: utility nodes that map `GpuImage` messages to CPU image topics.
- `utils/cuda_ipc_poc`: small CUDA IPC / VMM-FD environment checks.

## More Details

- Demo usage: [examples/multi_process_image_fanout/README.md](examples/multi_process_image_fanout/README.md)
- Development notes: [DEVELOPING.md](DEVELOPING.md)
- Core design: [doc/design.md](doc/design.md)
- Python subscriber: [ros2_cuda_ipc_py/README.md](ros2_cuda_ipc_py/README.md)
- Python binding design: [doc/python-subscriber-implementation.md](doc/python-subscriber-implementation.md)
- CUDA IPC / VMM-FD checks: [utils/cuda_ipc_poc/README.md](utils/cuda_ipc_poc/README.md)

## Publisher API

```cpp
auto slot = manager.acquire_for_publish();
launch_gpu_work(slot->device_ptr(), stream);

auto descriptor = slot->prepare_publish(stream);
if (!descriptor) {
  return;
}

publisher->publish(make_message(descriptor.value()));
```

Pass the stream that carries the producer work dependency to
`prepare_publish()`. On success, publish the returned descriptor. On failure,
that slot is not reused until `GpuBufferManager::reset()`.

The API accepts Driver API `CUstream` values and Runtime API `cudaStream_t`
values directly. The application retains ownership of the stream.

## License

MIT License. See [LICENSE](LICENSE).
