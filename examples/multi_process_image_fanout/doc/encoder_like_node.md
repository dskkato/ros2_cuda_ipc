## Node: encoder_like_node

### Purpose

Represent a video encoder input path without introducing NVENC or any encoder
dependency.

It consumes the shared GPU RGBA image and runs a CUDA kernel that resembles a
pre-encoder color conversion/downscale step.

### Responsibilities

Implement:

```text
encoder_like_node.cpp
```

The encoder-like node must:

1. Subscribe to `ros2_cuda_ipc_core::view::ImageView`.
2. Validate the view.
3. Set the CUDA device from `view.core.device_id`.
4. Use one non-blocking CUDA stream.
5. Wait on the input ready event with `view.enqueue_ready_event(stream_)`.
6. Lazily allocate an internal GPU output buffer when input dimensions change.
7. Launch a kernel that converts RGBA to a downscaled luma plane.
8. Compute a small checksum on GPU.
9. Copy only the small checksum/stat result to host.
10. Publish/log a status message.
11. Never copy the full image or full output plane to host.

### Parameters

Default parameters:

```text
input_topic_name:   /fanout/image_gpu
status_topic_name:  /fanout/encoder_like/status
downscale:          2
log_every_n:        30
```

Only support `downscale=2` in the first implementation. If another value is
provided, warn and use `2`.

### Status message

Publish a `std_msgs::msg::String` with fields:

```text
node
received
stamp_ns
output_width
output_height
checksum
kernel_ms
```

`kernel_ms` should measure the color-conversion kernel plus checksum kernels on
the CUDA stream. Use CUDA events, not wall-clock time.

### NVTX ranges

Add these NVTX ranges:

```text
EncoderLikeNode::on_image
EncoderLikeNode::wait_input_event
EncoderLikeNode::ensure_buffers
EncoderLikeNode::rgba_to_luma_downscale2_kernel
EncoderLikeNode::checksum_u8_kernel
EncoderLikeNode::copy_checksum_to_host
EncoderLikeNode::publish_status
```

