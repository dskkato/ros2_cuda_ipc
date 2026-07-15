# gpu_image_transport

`gpu_image_transport` provides utility ROS 2 nodes for mapping
`ros2_cuda_ipc_msgs::msg::GpuImage` messages into imported Views and then
republishing CPU-backed image topics.

This package is not an `image_transport` plugin package. It does not register
pluginlib transports like the packages in
`ros-perception/image_transport_plugins`; it subscribes to GPU-backed
`GpuImage` messages, maps each message explicitly, waits on the exported CUDA
ready event, copies the image payload to pinned host memory, and republishes it
as standard ROS messages.

## Nodes

```bash
ros2 run gpu_image_transport gpu_image_transport \
  --ros-args \
  -p input_topic_name:=/fanout/image_gpu \
  -p cpu_topic_name:=/fanout/preview/image
```

Publishes `sensor_msgs::msg::Image`.

```bash
ros2 run gpu_image_transport gpu_image_transport_compressed \
  --ros-args \
  -p input_topic_name:=/fanout/image_gpu \
  -p cpu_topic_name:=/fanout/preview/compressed \
  -p compressed_format:=jpeg \
  -p jpeg_quality:=95
```

Publishes `sensor_msgs::msg::CompressedImage`.

## Parameters

- `input_topic_name`: GPU `GpuImage` input topic. Default: `image_gpu`.
- `cpu_topic_name`: CPU image output topic. Default: `image`.
- `compressed_format`: `jpeg`, `png`, or `bmp` for the compressed node.
- `jpeg_quality`: JPEG quality for the compressed node, clamped to `0..100`.
