# Developing

This document collects implementation and maintainer-oriented details for
`ros2_cuda_ipc`. The top-level README is intentionally kept short for people
who want to try the demo first.

## Packages

- `ros2_cuda_ipc_msgs`: message definitions for GPU-backed buffers.
- `ros2_cuda_ipc_core`: CUDA memory sharing, buffer reference handling, and modality-specific Views.
- `examples/multi_process_image_fanout`: primary multi-process sample.
- `utils/gpu_image_transport`: utility bridge from `GpuImage` messages to CPU image topics.
- `utils/cuda_ipc_poc`: CUDA IPC and VMM-FD environment checks.

## Core Components

- `ros2_cuda_ipc_core::subscriber::BufferMapper`: maps a `BufferCore` and a consumer stream to an optional `ReadHandle`.
- `ros2_cuda_ipc_core::subscriber::ReadHandle`: the normal pointer API. It waits for producer readiness, owns the buffer reference, and defers resource/buffer reference release until consumer completion.
- `ros2_cuda_ipc_core::image::ImageView` / `ros2_cuda_ipc_core::pointcloud2::PointCloud2View`: typed adapters layered on `ReadHandle`; the Python image adapter is DLPack-only and binds its stream at export.
- `ros2_cuda_ipc_core::publisher::GpuBufferPool`: publisher-local allocation and reuse strategy for independent `GpuBufferBlock` resources.
- `ros2_cuda_ipc_core::publisher::BufferMetadataManager`: publisher reservation, uid, and grace-period state.
- `ros2_cuda_ipc_core::publisher::GpuBufferManager`: publisher-facing buffer manager.
- `ros2_cuda_ipc_core::publisher::PublishBlock`: one move-only publish attempt with RAII cancellation.
- `ros2_cuda_ipc_core::buffer_metadata::BlockMetadata`: shared per-block publication identity and refcount state. Subscriber buffer reference handles, import caches, completion events, and deferred queues are internal.

Publisher code should follow this order:

```cpp
auto block = manager.acquire_for_publish();
launch_gpu_work(block->device_ptr(), stream);
auto descriptor = block->prepare_publish(stream);
if (!descriptor) {
  return;
}
publisher->publish(make_message(descriptor.value()));
```

`prepare_publish(stream)` builds the transport descriptor, records the ready
event, and commits the reservation. On failure, it returns no descriptor and
retains the reservation until `GpuBufferManager::reset()`.

The public stream-taking APIs use the CUDA Driver API stream type `CUstream`.
Applications that use the CUDA Runtime API include
`<cuda_runtime_api.h>` themselves; a Runtime-created `cudaStream_t` can be
passed directly to `prepare_publish()`. The stream is owned by the application
and is only borrowed by the core library.

Destroying a block before preparation cancels its reservation. A successful
preparation commits it. The subsequent middleware publish result does not
affect block lifecycle.
The protocol guarantees and known limitations are specified in
[doc/buffer_metadata_protocol.md](doc/buffer_metadata_protocol.md).

Receiving code subscribes to `ros2_cuda_ipc_msgs::msg::BufferCore` and calls
`BufferMapper::map(message, consumer_stream)`. Mapping acquires the buffer reference and
imports or looks up the GPU resource before returning a `ReadHandle`.

The consumer stream must remain valid until the corresponding `ReadHandle` is
destroyed and its completion event has been recorded. A failed optional map
contains no public error detail; the mapper writes diagnostic detail to the
internal log.

## Memory Sharing

`ros2_cuda_ipc` uses CUDA VMM plus POSIX file descriptors as its only GPU memory
sharing backend. It uses CUDA Driver API Virtual Memory Management:

- publisher allocates GPU memory with `cuMemCreate`
- publisher exports it with `cuMemExportToShareableHandle`
- file descriptors are distributed through a Unix domain socket
- subscribers import memory with `cuMemImportFromShareableHandle`
- ready-event synchronization still uses CUDA IPC event handles

The library requires CUDA Toolkit 10.2 or newer. The standalone
`cuda_ipc_poc` package still contains CUDA IPC and VMM-FD comparison programs,
but CUDA IPC is not a supported backend of the library.

## Environment Checks

Before debugging the ROS path, check whether the platform supports VMM+FD:

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

The fanout demo uses the host GPU's native CUDA architecture by default with
CMake 3.24 or newer. GPU-less CI and package builds should pass an explicit
`CMAKE_CUDA_ARCHITECTURES` list instead.

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

Expected NVTX ranges include block acquisition, producer kernel work, input
event waits, preview image copy, encoder-like kernels, and inference-like
kernels. A large device-to-host image copy should appear only in `preview_node`.

## CI Container Images

GitHub Actions expects images named
`ghcr.io/dskkato/ros2-cuda-ipc-dev:<ROS_DISTRO>-20260802`. The date suffix
pins CI and devcontainer use to a specific image revision instead of a mutable
ROS-distribution tag.

```bash
./scripts/build_container.sh --ros-distro humble --push
./scripts/build_container.sh --ros-distro jazzy --push
./scripts/build_container.sh --ros-distro lyrical --push
```

Run `docker login ghcr.io` before pushing. Use `--tag` to provide a full custom
repository tag when needed.

## Design References

- [doc/design.md](doc/design.md)
- [doc/buffer_metadata_protocol.md](doc/buffer_metadata_protocol.md)
- [doc/future_work.md](doc/future_work.md)
- [examples/multi_process_image_fanout/doc/design.md](examples/multi_process_image_fanout/doc/design.md)
