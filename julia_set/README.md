# julia_set

This package provides a GPU-rendered Julia set demo that streams CUDA images
between ROS 2 processes using the zero-copy primitives from
`ros2_cuda_ipc_core`. The publisher renders a fractal directly into device
memory and shares it via CUDA IPC handles, while the subscriber maps the shared
memory and samples pixels for verification.

## Build

```bash
colcon build --packages-select julia_set
```

Ensure the workspace is sourced before launching the demo:

```bash
source install/setup.bash
```

## Nodes

### `julia_set_publisher`

Publishes `ros2_cuda_ipc_core::ImageView` messages containing the Julia set
fractal. Key parameters:

- `publish_rate_hz` (double, default `30.0`): Output frame rate.
- `width` / `height` (int, defaults `1280`/`720`): Image resolution.
- `channels` (int, default `3`): Number of color channels (RGB or RGBA).
- `max_iterations` (int, default `300`): Iteration count for the fractal.
- `zoom`, `offset_x`, `offset_y` (double): Controls camera zoom and pan.
- `constant_real`, `constant_imag` (double): Complex plane constant for the
  Julia set.
- `animate` (bool, default `true`): Enables slow time-based animation of the
  complex constant.
- `animation_speed` (double, default `1.0`): Multiplier for the animation rate.
- `shm_name` (string, default `/ros2_cuda_ipc_julia`): Shared-memory namespace
  used for lease coordination.
- `slot_count` (int, default `4`): Number of CUDA buffers in the pool.
- `device_index` (int, default `0`): CUDA device used for rendering.

### `julia_set_subscriber`

Consumes the shared `ImageView`, waits on the exported CUDA event and samples a
configurable number of bytes for logging. Parameters:

- `topic_name` (string, default `julia_set/image`)
- `sample_bytes` (int, default `64`): Number of bytes to copy back for logs
  (minimum of 1).
- `log_full_copy` (bool, default `false`): If `true`, copies the full image for
  validation (large transfers on big frames).

### `gpu_image_transport`

Copies the GPU image to host memory and republishes it as a
`sensor_msgs::msg::Image`. Parameters:

- `input_topic_name` (string, default `image_gpu`): Topic carrying
  `ros2_cuda_ipc_core::ImageView` messages.
- `cpu_topic_name` (string, default `image`): Output topic name.

### `gpu_image_transport_compressed`

Publishes the CUDA image as a `sensor_msgs::msg::CompressedImage` using
OpenCV for encoding. Parameters:

- `input_topic_name` (string, default `image_gpu`): Topic carrying
  `ros2_cuda_ipc_core::ImageView` messages.
- `cpu_topic_name` (string, default `image`): Output topic name.
- `compressed_format` (string, default `jpeg`): Encoding passed to
  `cv::imencode` (e.g. `jpeg`, `png`, `bmp`).
- `jpeg_quality` (int, default `95`): JPEG quality (0–100) when
  `compressed_format` is `jpeg` or `jpg`.

## Launch demo

Start publisher and subscriber in separate processes using the provided launch
file:

```bash
ros2 launch julia_set julia_set_demo.launch.py
```

Override parameters at launch time, for example to zoom in and increase the
iteration depth:

```bash
ros2 launch julia_set julia_set_demo.launch.py zoom:=0.8 max_iterations:=600
```

To forward an additional compressed output from the transport node while
keeping the raw CPU topic, enable the launch flag to spawn the dedicated
compression executable and optionally specify the format or topic name:

```bash
ros2 launch julia_set julia_set_demo.launch.py \
  use_compressed_output:=true compressed_format:=jpeg jpeg_quality:=90 \
  compressed_topic_name:=julia_set/image_compressed
```

The subscriber logs sampled pixel values once the CUDA event signals that the
frame is ready, demonstrating cross-process GPU buffer sharing via CUDA IPC.
