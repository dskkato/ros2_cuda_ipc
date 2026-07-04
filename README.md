# ros2_cuda_ipc

ROS 2 nodes can pass large GPU images between processes without copying the
full payload through host memory.

This repository provides CUDA-backed message types, mapping utilities, and a
small multi-process demo that shows one GPU image publisher feeding preview,
encoder-like, and inference-like consumers.

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
  resolution:=720p \
  publish_rate_hz:=60.0 \
  memory_backend:=cuda_ipc \
  slot_count:=4 \
  pending_ttl_ms:=300 \
  shm_name:=/ros2_cuda_ipc_fanout \
  device_index:=0
```

`resolution` accepts `480p`, `720p`, `1080p`, `4K`, `8K`, and `16K`. If you
need a custom size, pass `width` and `height`.

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
- `ros2_cuda_ipc_core`: CUDA memory sharing, mapping, and view utilities.
- `examples/multi_process_image_fanout`: primary runnable demo.
- `utils/gpu_image_transport`: utility nodes that bridge GPU `ImageView` topics to CPU image topics.
- `utils/cuda_ipc_poc`: small CUDA IPC / VMM-FD environment checks.

## More Details

- Demo usage: [examples/multi_process_image_fanout/README.md](examples/multi_process_image_fanout/README.md)
- Development notes: [DEVELOPING.md](DEVELOPING.md)
- Core design: [doc/design.md](doc/design.md)
- CUDA IPC / VMM-FD checks: [utils/cuda_ipc_poc/README.md](utils/cuda_ipc_poc/README.md)

## License

MIT License. See [LICENSE](LICENSE).
