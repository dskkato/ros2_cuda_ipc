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
````

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

## Repository layout

Move the old samples under `examples/legacy` and make the new demo the main
example.

Expected layout:

```text
examples/
  legacy/
    julia_set/
    gpu_image_transport/

  multi_process_image_fanout/
    CMakeLists.txt
    package.xml
    README.md
    doc/
      design.md
    include/
      multi_process_image_fanout/
        cuda_checks.hpp
        image_publisher_helper.hpp
        kernels.hpp
        status_format.hpp
    launch/
      multi_process_image_fanout.launch.xml
    src/
      gpu_image_publisher.cpp
      preview_node.cpp
      encoder_like_node.cpp
      inference_like_node.cpp
      image_publisher_helper.cpp
      cuda/
        kernels.cu
    test/
      test_kernels.cpp
```

Notes:

* The new package name must be `multi_process_image_fanout`.
* Do not create a Python launch file. Use XML launch only.
* Keep the package self-contained under `examples/multi_process_image_fanout`.
* Update the root `README.md` to point to this demo as the main example.
  Do not spend time modernizing legacy content in this task.

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
  custom type:
    ros2_cuda_ipc_core::view::ImageView
  ROS message type through TypeAdapter:
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
inter-process CUDA IPC:

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

## Node: gpu_image_publisher

See gpu_image_publisher.md for detail.

## Node: preview_node

See preview_node.md for detail.

## Node: encoder_like_node

See encoder_like_node.md for detail.

## Node: inference_like_node

See inference_like_node.md for detail.

## Shared CUDA/helper code

Implement shared CUDA wrappers in:

```text
include/multi_process_image_fanout/kernels.hpp
src/cuda/kernels.cu
```

Suggested API:

```cpp
#pragma once

#include <cstddef>
#include <cstdint>
#include <cuda_runtime.h>

namespace multi_process_image_fanout
{

struct InferenceStats
{
  float mean = 0.0f;
  float min = 0.0f;
  float max = 0.0f;
  uint64_t checksum = 0;
};

cudaError_t launch_generate_rgba_pattern_kernel(
  uint8_t* output,
  int width,
  int height,
  uint64_t stride_bytes,
  uint64_t frame_index,
  cudaStream_t stream);

cudaError_t launch_rgba_to_luma_downscale2_kernel(
  const uint8_t* input_rgba,
  uint8_t* output_luma,
  int input_width,
  int input_height,
  uint64_t input_stride_bytes,
  cudaStream_t stream);

cudaError_t launch_checksum_u8_kernel(
  const uint8_t* input,
  size_t element_count,
  uint64_t* device_checksum,
  cudaStream_t stream);

cudaError_t launch_rgba_to_normalized_gray_kernel(
  const uint8_t* input_rgba,
  float* output_gray,
  int width,
  int height,
  uint64_t input_stride_bytes,
  cudaStream_t stream);

cudaError_t launch_inference_stats_kernel(
  const float* input,
  size_t element_count,
  InferenceStats* device_stats,
  cudaStream_t stream);

}  // namespace multi_process_image_fanout
```

Keep kernel implementations straightforward and readable.

## CUDA error handling

Add a small helper header:

```text
include/multi_process_image_fanout/cuda_checks.hpp
```

It should provide readable error handling around CUDA calls. Keep it simple.

Suggested helpers:

```cpp
std::string cuda_error_to_string(cudaError_t err);

bool log_cuda_error(
  const rclcpp::Logger& logger,
  const char* operation,
  cudaError_t err);
```

If an equivalent helper already exists in `ros2_cuda_ipc_core`, use the existing
one instead of duplicating logic.

Node behavior on CUDA errors:

* Publisher:
  * log error
  * skip the frame
  * do not publish an invalid view
* Subscribers:
  * log error
  * skip the frame
  * do not crash on a single bad frame
* Constructor-time failures such as `cudaStreamCreateWithFlags` may throw.

## Status formatting

Add:

```text
include/multi_process_image_fanout/status_format.hpp
```

Keep status formatting dependency-free. Do not add a JSON library.

Simple `std::ostringstream` formatting is enough.

Example API:

```cpp
std::string format_encoder_status(
  uint64_t received_count,
  int64_t stamp_ns,
  int output_width,
  int output_height,
  uint64_t checksum,
  float kernel_ms);

std::string format_inference_status(
  uint64_t received_count,
  int64_t stamp_ns,
  float mean,
  float min,
  float max,
  uint64_t checksum,
  float kernel_ms);
```

## XML launch file

Create:

```text
launch/multi_process_image_fanout.launch.xml
```

It must launch all four nodes as separate processes.

Use launch args:

```text
width
height
publish_rate_hz
memory_backend
device_index
image_topic
preview_topic
```

Default XML shape:

```xml
<launch>
  <arg name="width" default="1920"/>
  <arg name="height" default="1080"/>
  <arg name="publish_rate_hz" default="30.0"/>
  <arg name="memory_backend" default="cuda_ipc"/>
  <arg name="device_index" default="0"/>
  <arg name="image_topic" default="/fanout/image_gpu"/>
  <arg name="preview_topic" default="/fanout/preview/image"/>

  <node pkg="multi_process_image_fanout"
        exec="gpu_image_publisher"
        name="gpu_image_publisher"
        output="screen">
    <param name="topic_name" value="$(var image_topic)"/>
    <param name="width" value="$(var width)"/>
    <param name="height" value="$(var height)"/>
    <param name="publish_rate_hz" value="$(var publish_rate_hz)"/>
    <param name="memory_backend" value="$(var memory_backend)"/>
    <param name="device_index" value="$(var device_index)"/>
  </node>

  <node pkg="multi_process_image_fanout"
        exec="preview_node"
        name="preview_node"
        output="screen">
    <param name="input_topic_name" value="$(var image_topic)"/>
    <param name="output_topic_name" value="$(var preview_topic)"/>
  </node>

  <node pkg="multi_process_image_fanout"
        exec="encoder_like_node"
        name="encoder_like_node"
        output="screen">
    <param name="input_topic_name" value="$(var image_topic)"/>
  </node>

  <node pkg="multi_process_image_fanout"
        exec="inference_like_node"
        name="inference_like_node"
        output="screen">
    <param name="input_topic_name" value="$(var image_topic)"/>
  </node>
</launch>
```

Do not use composable nodes in this demo.

## README for the package

Create:

```text
examples/multi_process_image_fanout/README.md
```

The README should explain:

1. What this demo shows.
2. Why there are three subscribers.
3. Why preview copies to CPU but encoder/inference do not.
4. How to build.
5. How to launch.
6. How to visualize the preview image.
7. How to inspect status topics.
8. How to profile with Nsight Systems.

Keep README concise. The design details belong in this file, not in README.

## Build command

From the repository root:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to multi_process_image_fanout
source install/setup.bash
```

Use `--packages-up-to` rather than `--packages-select` so dependencies in the
workspace are built too.

## Run command

```bash
source install/setup.bash
ros2 launch multi_process_image_fanout multi_process_image_fanout.launch.xml
```

Optional overrides:

```bash
ros2 launch multi_process_image_fanout multi_process_image_fanout.launch.xml \
  width:=1280 \
  height:=720 \
  publish_rate_hz:=60.0 \
  memory_backend:=cuda_ipc \
  device_index:=0
```

For Jetson/VMM-FD environments, allow:

```bash
ros2 launch multi_process_image_fanout multi_process_image_fanout.launch.xml \
  memory_backend:=vmm_fd
```

Only include this in README if the backend is already supported by
`ros2_cuda_ipc_core` in the current repository.

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

### Test command

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to multi_process_image_fanout \
  --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select multi_process_image_fanout --event-handlers console_direct+
colcon test-result --verbose
```

### Optional test: launch smoke test

A ROS launch test is optional for this task.

If implemented, it should:

1. Launch all four nodes.
2. Wait for `/fanout/preview/image`.
3. Wait for both status topics.
4. Exit cleanly.

Do not let launch testing delay the main implementation.

## Runtime correctness checklist

The implementation is complete only if all of the following are true:

* `colcon build --packages-up-to multi_process_image_fanout` succeeds.
* `colcon test --packages-select multi_process_image_fanout` succeeds or skips
  CUDA tests gracefully when no CUDA device is available.
* `ros2 launch multi_process_image_fanout multi_process_image_fanout.launch.xml`
  starts four separate processes.
* `/fanout/image_gpu` is published.
* `/fanout/preview/image` is published as `sensor_msgs::msg::Image`.
* `/fanout/encoder_like/status` is published.
* `/fanout/inference_like/status` is published.
* `rqt_image_view` can display `/fanout/preview/image`.
* Encoder-like and inference-like nodes do not perform full image host copies.
* NVTX ranges are present in all nodes.
* The package README documents build, run, validation, and profiling steps.
* The root README points to this demo as the main sample.
* Legacy demos are moved under `examples/legacy`.

## Implementation order for Codex

Follow this order to avoid scope drift:

1. Move old demo packages into `examples/legacy`.
2. Add `examples/legacy/COLCON_IGNORE`.
3. Create the `examples/multi_process_image_fanout` package skeleton.
4. Add this `doc/design.md`.
5. Implement shared CUDA kernels and wrappers.
6. Implement the CUDA kernel smoke test.
7. Implement `gpu_image_publisher`.
8. Implement `preview_node`.
9. Implement `encoder_like_node`.
10. Implement `inference_like_node`.
11. Add XML launch file.
12. Add package README.
13. Update root README.
14. Run build and tests.
15. Run the manual validation commands.

Do not implement fusion, OpenGL, NVENC, TensorRT, or OpenCV display in this
change.

-----

Nsight Systems 側は、公式ドキュメントでも NVTX range を入れると CPU range とそこから起動された GPU work を Timeline View で追えると説明されているので、この要件はデモの解析性として入れておいてよいです。`nsys profile` のCLI形式も公式ユーザーガイドに載っています。:contentReference[oaicite:1]{index=1}

[1]: https://github.com/dskkato/ros2_cuda_ipc "GitHub - dskkato/ros2_cuda_ipc: ROS 2/CUDA IPC support · GitHub"
