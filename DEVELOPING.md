# Developing

This document collects implementation and maintainer-oriented details for
`ros2_cuda_ipc`. The top-level README is intentionally kept short for people
who want to try the demo first.

## Packages

- `ros2_cuda_ipc_msgs`: message definitions for GPU-backed buffers.
- `ros2_cuda_ipc_core`: CUDA memory sharing, lease handling, and modality-specific Views.
- `examples/multi_process_image_fanout`: primary multi-process sample.
- `utils/gpu_image_transport`: utility bridge from `GpuImage` messages to CPU image topics.
- `utils/cuda_ipc_poc`: CUDA IPC and VMM-FD environment checks.

## Core Components

- `ros2_cuda_ipc_core::subscriber::BufferView`: modality-independent view for an imported GPU resource and its lease.
- `ros2_cuda_ipc_core::image::ImageView` / `ros2_cuda_ipc_core::pointcloud2::PointCloud2View`: example modality views layered on `subscriber::BufferView`.
- `ros2_cuda_ipc_core::subscriber::BufferViewMapper` and `ros2_cuda_ipc_core::image::ImageViewMapper` / `ros2_cuda_ipc_core::pointcloud2::PointCloud2ViewMapper`: explicit mapping APIs from raw messages to imported views.
- `ros2_cuda_ipc_core::lease::LeaseHandle`: process-shared slot lease accounting.
- `ros2_cuda_ipc_core::publisher::GpuBufferPool`: publisher-side GPU resource ownership.
- `ros2_cuda_ipc_core::publisher::LeaseManager`: publisher reservation, generation, and grace-period state.
- `ros2_cuda_ipc_core::publisher::GpuBufferManager`: publisher-facing buffer manager.
- `ros2_cuda_ipc_core::publisher::PublishSlot`: one move-only publish attempt with RAII cancellation.

Publisher code should follow this order:

```cpp
auto slot = manager.acquire_for_publish();
launch_gpu_work(slot->device_ptr(), stream);
auto descriptor = slot->prepare_publish(stream);
if (!descriptor) {
  return;
}
publisher->publish(make_message(descriptor.value()));
```

`prepare_publish(stream)` builds the transport descriptor, records the ready
event, and commits the reservation. On failure, it returns no descriptor and
retains the reservation until `GpuBufferManager::reset()`.

The public ready-event APIs use the CUDA Driver API stream type `CUstream`.
Applications that use the CUDA Runtime API include
`<cuda_runtime_api.h>` themselves; a Runtime-created `cudaStream_t` can be
passed directly to `prepare_publish()` and `enqueue_ready_event()`. The stream is
owned by the application and is only borrowed by the core library.

Destroying a slot before preparation cancels its reservation. A successful
preparation commits it. The subsequent middleware publish result does not
affect slot lifecycle.
The protocol guarantees and known limitations are specified in
[doc/lease_protocol.md](doc/lease_protocol.md).

Receiving code subscribes to `ros2_cuda_ipc_msgs::msg::GpuImage` and calls the
mapper explicitly. Mapping acquires the lease and imports or looks up the GPU
resource before returning an `ros2_cuda_ipc_core::image::ImageView`.

## Memory Backends

`ros2_cuda_ipc` supports two GPU memory sharing backends.

### CUDA IPC

This is the default backend for x86_64 + dGPU systems that support CUDA IPC.
It uses `cudaIpcMemHandle_t` and `cudaIpcEventHandle_t` to share GPU memory and
ready events between processes.

### VMM + FD

This backend is intended for systems such as Jetson Orin where CUDA IPC memory
sharing is not available. It uses CUDA Driver API Virtual Memory Management:

- publisher allocates GPU memory with `cuMemCreate`
- publisher exports it with `cuMemExportToShareableHandle`
- file descriptors are distributed through a Unix domain socket
- subscribers import memory with `cuMemImportFromShareableHandle`
- ready-event synchronization still uses CUDA IPC event handles

Both publisher and subscribers must use the same backend.

Accepted backend names include `cuda_ipc`, `vmm_fd`, `vmm-fd`, `vmm`, and `fd`.

## Environment Checks

Before debugging the ROS path, check whether the platform supports the intended
memory sharing backend:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to cuda_ipc_poc
source install/setup.bash

ros2 launch cuda_ipc_poc cuda_ipc.launch.py
ros2 launch cuda_ipc_poc vmm_fd.launch.py
```

See [utils/cuda_ipc_poc/README.md](utils/cuda_ipc_poc/README.md).

## Build And Test

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to multi_process_image_fanout
```

Core unit tests:

```bash
colcon build --packages-select ros2_cuda_ipc_core --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select ros2_cuda_ipc_core
colcon test-result --verbose
```

Demo CUDA kernel smoke test:

```bash
colcon build --symlink-install --packages-up-to multi_process_image_fanout \
  --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select multi_process_image_fanout --event-handlers console_direct+
colcon test-result --verbose
```

CUDA-dependent tests may skip when no CUDA device is available or when the
platform does not support the selected CUDA memory sharing feature.

## Profiling

The fanout launch file can wrap each process with Nsight Systems:

```bash
ros2 launch multi_process_image_fanout multi_process_image_fanout.launch.py \
  enable_nsys:=true \
  nsys_profile_label:=run1
```

This writes separate reports named
`fanout-<arch>-<width>x<height>-<rate>-<label>-<node>`. Override
`nsys_profile_flags` to change the default `--trace=osrt,nvtx,cuda` flags.

Expected NVTX ranges include slot acquisition, producer kernel work, input
event waits, preview image copy, encoder-like kernels, and inference-like
kernels. A large device-to-host image copy should appear only in `preview_node`.

## CI Container Images

GitHub Actions expects images named
`ghcr.io/dskkato/ros2-cuda-ipc-dev:<ROS_DISTRO>`.

```bash
./scripts/build_container.sh --ros-distro humble --push
./scripts/build_container.sh --ros-distro jazzy --push
./scripts/build_container.sh --ros-distro lyrical --push
```

Run `docker login ghcr.io` before pushing. Use `--tag` to provide a full custom
repository tag when needed.

## Design References

- [doc/design.md](doc/design.md)
- [doc/lease_protocol.md](doc/lease_protocol.md)
- [doc/future_work.md](doc/future_work.md)
- [examples/multi_process_image_fanout/doc/design.md](examples/multi_process_image_fanout/doc/design.md)
