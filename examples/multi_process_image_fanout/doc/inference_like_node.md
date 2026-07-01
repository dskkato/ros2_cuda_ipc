## Node: inference_like_node

### Purpose

Represent a DNN inference preprocessing path without introducing TensorRT,
ONNX Runtime, or model files.

It consumes the shared GPU RGBA image and runs CUDA kernels that resemble image
normalization and simple feature/stat extraction.

### Responsibilities

Implement:

```text
inference_like_node.cpp
```

The inference-like node must:

1. Subscribe to `ros2_cuda_ipc_core::view::ImageView`.
2. Validate the view.
3. Set the CUDA device from `view.core.device_id`.
4. Use one non-blocking CUDA stream.
5. Wait on the input ready event with `view.enqueue_ready_event(stream_)`.
6. Lazily allocate an internal GPU float buffer when dimensions change.
7. Launch a kernel that converts RGBA8 into normalized grayscale float values.
8. Compute mean/min/max/checksum on GPU.
9. Copy only the small stat result to host.
10. Publish/log a status message.
11. Never copy the full image or full normalized tensor to host.

### Parameters

Default parameters:

```text
input_topic_name:   /fanout/image_gpu
status_topic_name:  /fanout/inference_like/status
log_every_n:        30
```

### CUDA kernels

Implement:

```cpp
cudaError_t launch_rgba_to_normalized_gray_kernel(
  const uint8_t* input_rgba,
  float* output_gray,
  int width,
  int height,
  uint64_t input_stride_bytes,
  cudaStream_t stream);
```

For each pixel:

```text
gray_u8 = (77 * R + 150 * G + 29 * B) >> 8
gray_f32 = gray_u8 / 255.0f
```

Implement stats reduction:

```cpp
struct InferenceStats
{
  float mean;
  float min;
  float max;
  uint64_t checksum;
};

cudaError_t launch_inference_stats_kernel(
  const float* input,
  size_t element_count,
  InferenceStats* device_stats,
  cudaStream_t stream);
```

Checksum formula:

```text
q = clamp(round(input[i] * 65535.0f), 0, 65535)
checksum = sum(q * ((i % 251) + 1))
```

`mean`, `min`, `max`, and `checksum` must be computed from the normalized
grayscale buffer.

The host may copy back only:

```text
sizeof(InferenceStats)
```

or a small partial-stats array.

### Status message

Publish a `std_msgs::msg::String` with fields:

```text
node
received
stamp_ns
mean
min
max
checksum
kernel_ms
```

`kernel_ms` should measure the normalization kernel plus stats kernels on the
CUDA stream. Use CUDA events, not wall-clock time.

### NVTX ranges

Add these NVTX ranges:

```text
InferenceLikeNode::on_image
InferenceLikeNode::wait_input_event
InferenceLikeNode::ensure_buffers
InferenceLikeNode::rgba_to_normalized_gray_kernel
InferenceLikeNode::stats_kernel
InferenceLikeNode::copy_stats_to_host
InferenceLikeNode::publish_status
```
