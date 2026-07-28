# multi_process_image_fanout

`multi_process_image_fanout` is the primary `ros2_cuda_ipc` demo. It shows one
publisher process generating an RGBA image in GPU memory and three independent
subscriber processes consuming the same buffer through CUDA IPC without copying
the full image through host memory.

## Process graph

```text
gpu_image_publisher
  |-- preview_node
  |-- encoder_like_node
  `-- inference_like_node
```

The XML launch file starts all four nodes as separate processes. Intra-process
communication is disabled intentionally because this demo is about
inter-process CUDA IPC.

## Subscribers

The three subscribers model common fanout use cases:

* `preview_node` copies the GPU image to CPU and republishes
  `sensor_msgs::msg::Image` on `/fanout/preview/image` for tools such as
  `rqt_image_view`.
* `encoder_like_node` runs CUDA kernels that downscale to luma and compute a
  checksum, then publishes a small status string on
  `/fanout/encoder_like/status`.
* `inference_like_node` runs CUDA kernels that normalize grayscale pixels and
  compute simple stats, then publishes a small status string on
  `/fanout/inference_like/status`.

Only `preview_node` performs a full device-to-host image copy. The
encoder-like and inference-like nodes copy back only small checksum/stat
results.

## CUDA stream API

The example uses CUDA Runtime API streams for its kernels and passes the same
`cudaStream_t` values directly to the core's `prepare_publish()` and
`enqueue_ready_event()` APIs. Each application source includes
`<cuda_runtime_api.h>` explicitly. The core API is declared in terms of
`CUstream`, and the application remains responsible for stream ownership.

## Build

From the repository root:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to multi_process_image_fanout
source install/setup.bash
```

Use `--packages-up-to` so workspace dependencies are built as needed.

## Launch

```bash
ros2 launch multi_process_image_fanout multi_process_image_fanout.launch.py
```

Optional overrides:

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

On environments where the core package supports VMM-FD sharing, for example
Jetson Orin:

```bash
ros2 launch multi_process_image_fanout multi_process_image_fanout.launch.py \
  memory_backend:=vmm_fd
```

The `width`, `height`, `memory_backend`, and `slot_count`,
`shm_name_prefix`, and `device_index` arguments are publisher-side pseudo-camera
parameters.

## Visualize preview

```bash
ros2 run rqt_image_view rqt_image_view
```

Select `/fanout/preview/image`. This topic is CPU-backed by design so standard
ROS image tools can display it.

## Inspect topics

```bash
ros2 topic hz /fanout/image_gpu
ros2 topic hz /fanout/preview/image
ros2 topic echo /fanout/encoder_like/status
ros2 topic echo /fanout/inference_like/status
```

Expected status messages are one-line JSON-like strings with increasing
`received` counts and changing checksums.

## Test

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to multi_process_image_fanout \
  --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select multi_process_image_fanout --event-handlers console_direct+
colcon test-result --verbose
```

The CUDA kernel smoke test skips gracefully when no CUDA device is available.

## Profile

```bash
ros2 launch multi_process_image_fanout multi_process_image_fanout.launch.py \
  enable_nsys:=true \
  nsys_profile_label:=run1
```

This wraps each process with `nsys profile` and writes separate reports named
`fanout-<arch>-<width>x<height>-<rate>-<label>-<node>`. Override
`nsys_profile_flags` to change the default `--trace=osrt,nvtx,cuda` flags.
Nsight Systems should show NVTX ranges for slot acquisition, producer kernel
work, input event waits, preview image copy, encoder-like kernels, and
inference-like kernels. A large device-to-host image copy should appear only in
`preview_node`.
