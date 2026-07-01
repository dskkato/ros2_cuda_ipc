# multi_process_image_fanout progress

## 2026-07-01

### Done

* Read `doc/design.md` and `doc/gpu_image_publisher.md`.
* Replaced empty `CMakeLists.txt` and `package.xml` with a buildable package scaffold.
* Added package directories and initial files for:
  * `README.md`
  * `launch/multi_process_image_fanout.launch.xml`
  * `include/multi_process_image_fanout/`
  * `src/gpu_image_publisher.cpp`
  * `src/image_publisher_helper.cpp`
  * `src/cuda/kernels.cu`
  * `test/test_kernels.cpp`
* Implemented the first version of `gpu_image_publisher`:
  * parameter handling
  * CUDA stream creation
  * `GpuLeasePool` setup
  * RGBA pattern kernel launch
  * `ImageView` population and publish path
* Implemented `preview_node`:
  * subscribes to `ImageView`
  * validates layout and dtype
  * waits on the ready event
  * copies RGBA data to host with `cudaMemcpy2DAsync`
  * republishes `sensor_msgs::msg::Image`
* Added `preview_node` to the XML launch file and CMake targets.
* Implemented shared helper APIs:
  * `status_format.hpp` now formats encoder/inference status strings
  * `cuda_checks.hpp` now exposes readable CUDA error helpers
  * `kernels.hpp` / `src/cuda/kernels.cu` now declare and implement the encoder and inference kernels
* Implemented `encoder_like_node`:
  * subscribes to `ImageView`
  * validates layout and dtype
  * waits on the ready event
  * lazily allocates a downscaled luma buffer
  * launches RGBA-to-luma and checksum kernels
  * publishes `std_msgs::msg::String` status
* Added `encoder_like_node` to the package CMake target list and XML launch file.
* Extended the CUDA smoke test to cover the new kernel entry points and host reference checks.

### Not Started

* `inference_like_node`
* root `README.md` update to make this demo the main example

### Notes

* This step now covers the encoder path and the shared kernel helpers in addition to the original scaffolding work.
* XML launch currently starts `gpu_image_publisher`, `preview_node`, and `encoder_like_node`.
* `colcon build --packages-select multi_process_image_fanout --cmake-args -DBUILD_TESTING=ON` passed.
* `colcon test --packages-select multi_process_image_fanout --event-handlers console_direct+` passed.
* In the current environment, `test_kernels` is skipped when no CUDA device is available.
* Corrected `test_kernels` expected output for `(x=33, y=0)` to match the kernel formula.
