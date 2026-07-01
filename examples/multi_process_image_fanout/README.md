## multi_process_image_fanout

`multi_process_image_fanout` is the primary `ros2_cuda_ipc` demo package.

Current scope in this step:

* package scaffold
* `gpu_image_publisher`
* CUDA RGBA pattern kernel
* `preview_node`
* `encoder_like_node`
* `inference_like_node`

### Build

```bash
colcon build --packages-up-to multi_process_image_fanout
```

### Run

```bash
ros2 launch multi_process_image_fanout multi_process_image_fanout.launch.xml
```
