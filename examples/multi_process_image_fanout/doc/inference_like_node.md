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
