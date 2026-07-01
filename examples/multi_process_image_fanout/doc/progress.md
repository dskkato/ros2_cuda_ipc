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

### Not Started

* `preview_node`
* `encoder_like_node`
* `inference_like_node`
* root `README.md` update to make this demo the main example

### Notes

* This step intentionally focuses on package scaffolding and `gpu_image_publisher` only.
* XML launch currently starts `gpu_image_publisher` only until the other nodes exist.
* `colcon build --packages-select multi_process_image_fanout --cmake-args -DBUILD_TESTING=ON` passed.
* `colcon test --packages-select multi_process_image_fanout --return-code-on-test-failure` passed.
* In the current environment, `test_kernels` is skipped when no CUDA device is available.
* Corrected `test_kernels` expected output for `(x=33, y=0)` to match the kernel formula.
