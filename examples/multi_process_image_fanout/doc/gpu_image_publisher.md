## Node: gpu_image_publisher

### Purpose

Create a deterministic animated RGBA image directly in GPU memory and publish it
as `ros2_cuda_ipc_core::view::ImageView`.

This node is the main reference for publisher-side `ros2_cuda_ipc_core` usage.

### Responsibilities

Implement:

```text
gpu_image_publisher.cpp
image_publisher_helper.cpp
include/multi_process_image_fanout/image_publisher_helper.hpp
```

The publisher must:

1. Create a `ros2_cuda_ipc_core::cuda::GpuLeasePool`.
2. Allocate `slot_count` GPU buffers through the pool.
3. Use one non-blocking CUDA stream.
4. On each timer tick:

   * reclaim stale pending leases
   * acquire a slot using the current ROS subscription count
   * launch the image generation CUDA kernel into the acquired slot
   * record the slot ready event
   * fill `ros2_cuda_ipc_core::view::ImageView`
   * publish the view
5. Use `publisher_->get_subscription_count()` as the expected consumer count for
   the lease pool.
6. Do not copy the generated image to host memory.
7. Fill the `ImageView` fields correctly:

   * `header.stamp`
   * `header.frame_id`
   * `core.dev_ptr`
   * `core.ready_evt`
   * `core.device_id`
   * `core.byte_size`
   * `core.slot_id`
   * `core.generation`
   * `core.shm_name`
   * memory backend handles through `set_ipc_handles`
   * `dtype`
   * `shape`
   * `strides`
   * `encoding`

### Parameters

Default parameters:

```text
topic_name:        /fanout/image_gpu
publish_rate_hz:   30.0
width:             1920
height:            1080
frame_id:          fanout_camera_frame
slot_count:        4
pending_ttl_ms:    300
shm_name:          /ros2_cuda_ipc_fanout
device_index:      0
memory_backend:    cuda_ipc
```

Support `memory_backend` values using the existing parser from
`ros2_cuda_ipc_core`.

### CUDA kernel

Implement this fixed demo kernel:

```cpp
cudaError_t launch_generate_rgba_pattern_kernel(
  uint8_t* output,
  int width,
  int height,
  uint64_t stride_bytes,
  uint64_t frame_index,
  cudaStream_t stream);
```

Kernel behavior:

```text
for each pixel (x, y):

  block_x = x / 32
  block_y = y / 32

  r = (x + frame_index) & 0xff
  g = (y + 2 * frame_index) & 0xff
  b = ((block_x ^ block_y) * 37 + 3 * frame_index) & 0xff
  a = 255
```

Store RGBA in HWC order.

Reasoning:

* Visually recognizable.
* Deterministic.
* Animated.
* No fractal-specific code.
* No camera dependency.
* Easy to test with a CPU reference implementation.

### NVTX ranges

Add these NVTX ranges:

```text
GpuImagePublisherNode::on_timer
ImagePublisherHelper::produce
ImagePublisherHelper::acquire_slot
ImagePublisherHelper::generate_rgba_pattern_kernel
ImagePublisherHelper::cudaEventRecord
```

Use `ros2_cuda_ipc_core::cuda::NvtxScopedRange`.
