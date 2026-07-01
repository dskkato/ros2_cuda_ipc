## Node: preview_node

See preview_node.md for detail.

### Purpose

Consume the GPU image, wait on the CUDA ready event, copy the image to host
memory, and republish it as `sensor_msgs::msg::Image`.

This is the only node that performs a full GPU-to-CPU image copy.

### Responsibilities

Implement:

```text
preview_node.cpp
```

The preview node must:

1. Subscribe to `ros2_cuda_ipc_core::view::ImageView`.
2. Validate the view:

   * `view.core.valid()`
   * `view.sanity_check()`
   * dtype is `U8`
   * channels are `4`
3. Set the CUDA device from `view.core.device_id`.
4. Use one non-blocking CUDA stream.
5. Wait on the input ready event with `view.enqueue_ready_event(stream_)`.
6. Copy the image to host with `cudaMemcpy2DAsync`.
7. Synchronize only this stream before publishing the CPU image.
8. Publish `sensor_msgs::msg::Image` on `/fanout/preview/image`.
9. Preserve `header`, `height`, `width`, `encoding`, `step`, and `data`.

Use `rgba8` if the incoming encoding is empty.

### Parameters

Default parameters:

```text
input_topic_name:   /fanout/image_gpu
output_topic_name:  /fanout/preview/image
copy_every_n:       1
log_every_n:        30
```

`copy_every_n` may be used to reduce CPU copy load. Default must be `1` so the
demo works immediately with `rqt_image_view`.

### Host copy policy

The preview node is allowed to copy the full image to host memory.

No other node is allowed to copy the full image to host memory.

### NVTX ranges

Add these NVTX ranges:

```text
PreviewNode::on_image
PreviewNode::wait_input_event
PreviewNode::cudaMemcpy2DAsync_to_host
PreviewNode::publish_sensor_msgs_image
```

Also record CUDA events around the copy and log `copy_ms` every `log_every_n`
frames.
