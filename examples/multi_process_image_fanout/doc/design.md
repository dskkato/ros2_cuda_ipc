# multi_process_image_fanout design

## Goal

`multi_process_image_fanout` is the primary demo for `ros2_cuda_ipc`.

The demo shows the most important use case of `ros2_cuda_ipc_core`:

> One process creates a GPU image once, then multiple independent ROS 2
> subscriber processes consume the same GPU buffer without copying the image
> through host memory.

The demo intentionally avoids real camera drivers, OpenGL, TensorRT, NVENC,
image_transport plugins, and complex image processing. It should be easy to
read, easy to run, and useful as a reference implementation for
`ros2_cuda_ipc_core`.

The final process graph is:

```text
gpu_image_publisher
  ├── preview_node
  ├── encoder_like_node
  └── inference_like_node
```

Each node must run as a separate process from the XML launch file.

## Non-goals

Do not implement the following in this task:

* Real camera capture.
* Multi-camera fusion.
* OpenGL / EGL / CUDA graphics interop.
* OpenCV direct display.
* TensorRT, ONNX Runtime, DNN inference, or model loading.
* NVENC, V4L2 M2M, GStreamer, FFmpeg, or actual video encoding.
* A new ROS message package.
* Changes to `ros2_cuda_ipc_core` or `ros2_cuda_ipc_msgs`, unless a small bug fix
  is strictly necessary.

`encoder_like_node` and `inference_like_node` are intentionally CUDA-kernel-only
stand-ins for downstream GPU consumers.

## Package dependencies

`package.xml` should include at least:

* `ament_cmake`
* `rclcpp`
* `sensor_msgs`
* `std_msgs`
* `ros2_cuda_ipc_core`
* `ros2_cuda_ipc_msgs`

`CMakeLists.txt` should:

* use C++17
* enable CUDA
* use CUDA17 if available
* find `CUDAToolkit`
* link against `CUDA::cudart`
* link against `ros2_cuda_ipc_core::ros2_cuda_ipc_core`
* link against `ros2_cuda_ipc_msgs` in the same style as the existing packages
* install executables to `lib/${PROJECT_NAME}`
* install `launch/`, `doc/`, and `README.md`
* add a CUDA kernel smoke test when `BUILD_TESTING=ON`

Follow the existing CMake style used by the current examples where possible.

## Topics

Use these topic names by default:

```text
/fanout/image_gpu
/fanout/preview/image
/fanout/encoder_like/status
/fanout/inference_like/status
```

Topic types:

```text
/fanout/image_gpu
  ros2_cuda_ipc_msgs::msg::GpuImage

/fanout/preview/image
  sensor_msgs::msg::Image

/fanout/encoder_like/status
  std_msgs::msg::String

/fanout/inference_like/status
  std_msgs::msg::String
```

The status topics should publish one-line JSON-like strings. Do not introduce a
new custom status message.

Example status strings:

```json
{"node":"encoder_like","received":42,"stamp_ns":1234567890,"output_width":960,"output_height":540,"checksum":123456789,"kernel_ms":0.18}
```

```json
{"node":"inference_like","received":42,"stamp_ns":1234567890,"mean":0.4812,"min":0.0,"max":1.0,"checksum":987654321,"kernel_ms":0.22}
```

The strings only need to be stable enough for human inspection and simple tests.
They do not need to be parsed by production code.

## Common ROS 2 policy

All nodes must disable intra-process communication because this demo is about
inter-process VMM-FD sharing:

```cpp
rclcpp::NodeOptions().use_intra_process_comms(false)
```

Also set publisher/subscription options explicitly:

```cpp
options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
```

Use reliable QoS with `KeepLast(10)` for the GPU image topic and status topics.

## CUDA image format

Use one fixed default image format:

```text
width:     1920
height:    1080
channels:  4
dtype:     U8
encoding:  rgba8
layout:    HWC
strides:
  row:     width * 4
  column:  4
  channel: 1
```

The default frame size is:

```text
width * height * 4 bytes
```

The demo may expose `width`, `height`, and `publish_rate_hz` as parameters, but
the implementation does not need to support arbitrary encodings. Subscribers
should warn and skip frames that are not `U8` + 4 channels.

## Manual validation

### 1. Confirm GPU topic is published

```bash
ros2 topic list
ros2 topic hz /fanout/image_gpu
```

Expected:

* `/fanout/image_gpu` exists.
* Topic rate is close to `publish_rate_hz`.

### 2. Confirm CPU preview image is published

```bash
ros2 topic hz /fanout/preview/image
```

Expected:

* `/fanout/preview/image` exists.
* The rate is close to `publish_rate_hz / copy_every_n`.

### 3. Visualize preview

```bash
ros2 run rqt_image_view rqt_image_view
```

Select:

```text
/fanout/preview/image
```

Expected image:

* Animated RGBA test pattern.
* Checker/block pattern in blue channel.
* Smooth gradients in red and green channels.
* No need for OpenCV-specific display code.

### 4. Check encoder-like status

```bash
ros2 topic echo /fanout/encoder_like/status
```

Expected:

* JSON-like status lines.
* `received` increases.
* `checksum` changes as the generated pattern animates.
* `kernel_ms` is present.

### 5. Check inference-like status

```bash
ros2 topic echo /fanout/inference_like/status
```

Expected:

* JSON-like status lines.
* `received` increases.
* `mean` is between `0.0` and `1.0`.
* `min` is between `0.0` and `1.0`.
* `max` is between `0.0` and `1.0`.
* `checksum` changes as the generated pattern animates.
* `kernel_ms` is present.

## Profiling and NVTX validation

This demo must be useful with Nsight Systems.

Run:

```bash
nsys profile -t cuda,nvtx,osrt -o /tmp/ros2_cuda_ipc_fanout \
  ros2 launch multi_process_image_fanout multi_process_image_fanout.launch.xml
```

If the local Nsight Systems version needs process-tree or child-process options,
use the option names supported by that installed version. Do not hard-code
version-specific options in source code.

Expected profiler result:

* NVTX ranges appear for all four nodes.
* Publisher process shows:
  * slot acquisition
  * pattern generation kernel
  * CUDA event record
* Preview process shows:
  * input event wait
  * GPU-to-host copy
  * CPU image publish
* Encoder-like process shows:
  * input event wait
  * luma downscale kernel
  * checksum kernel
  * small checksum copy to host
* Inference-like process shows:
  * input event wait
  * normalized grayscale kernel
  * stats kernel
  * small stats copy to host

Important validation point:

* Large device-to-host image copy should appear only in `preview_node`.
* `encoder_like_node` and `inference_like_node` should copy back only small
  stat/checksum data.

## Automated tests

Add tests under:

```text
examples/multi_process_image_fanout/test/
```

### Required test: CUDA kernel smoke test

Implement:

```text
test/test_kernels.cpp
```

Use `ament_cmake_gtest`.

The test must:

1. Check `cudaGetDeviceCount`.
2. If no CUDA device is available, skip the test gracefully.
3. Allocate a small GPU buffer, for example `64x48 RGBA`.
4. Run `launch_generate_rgba_pattern_kernel` with a fixed `frame_index`.
5. Run `launch_rgba_to_luma_downscale2_kernel`.
6. Run `launch_checksum_u8_kernel`.
7. Run `launch_rgba_to_normalized_gray_kernel`.
8. Run `launch_inference_stats_kernel`.
9. Copy back only test-sized outputs or stats as needed.
10. Compare against CPU reference calculations using the same formulas from this
    design doc.

For the smoke test, full host copies of small test buffers are allowed because
the test verifies correctness. This exception applies only to tests, not runtime
nodes.

Assertions:

```text
generated RGBA pixels match CPU reference
downscaled luma output matches CPU reference
u8 checksum matches CPU reference
inference mean/min/max match CPU reference within tolerance
inference checksum matches CPU reference
```
