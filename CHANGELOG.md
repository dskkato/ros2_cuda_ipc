# Changelog

This document records the user-visible changes between releases of
`ros2_cuda_ipc`.

## [0.3.0] - 2026-07-25

This release is primarily an internal refactoring release. The core CUDA
implementation now uses the CUDA Driver API, which removes the core library's
dependency on the CUDA Runtime library (`libcudart.so`).

### Highlights

- Migrated CUDA initialization, context handling, memory imports, IPC events,
  and stream synchronization in `ros2_cuda_ipc_core` to the CUDA Driver API.
- Added process-wide Driver API initialization, primary-context ownership,
  scoped context activation, and Driver API error reporting utilities.
- Reworked CUDA IPC and VMM-FD resource ownership around RAII, including
  shared ownership of imported resources and deterministic cleanup of cached
  resources.
- Unified the public stream-facing APIs on `CUstream`, which is compatible
  with stream handles created by the CUDA Runtime API.
- Strengthened ownership and cleanup semantics for the IPC handle cache and
  lease-mapping cache, and made cache keys device-aware.

### Breaking changes and migration notes

- `ros2_cuda_ipc_core` now links to the CUDA Driver library only; it no longer
  links to `CUDA::cudart`. Applications that use the CUDA Runtime API must link
  `CUDA::cudart` themselves.
- Public stream and ready-event methods now use `CUstream` and return the
  Driver API-oriented `ros2_cuda_ipc_core::detail::CudaResult` type instead of
  `cudaStream_t` and `cudaError_t`.
- Applications that create streams with the CUDA Runtime API should include
  `<cuda_runtime_api.h>` explicitly. Runtime-created `cudaStream_t` handles
  remain usable with the `CUstream`-based APIs.
- Applications using the core public headers should review error handling for
  Driver API result values and update any code that directly checks
  `cudaError_t` return values.

### Tests and documentation

- Added Driver API context, ready-event, CUDA IPC memory, public stream, and
  lease-mapping cache tests, including cross-process coverage.
- Expanded resource-lifetime and cache cleanup coverage.
- Updated the design, lease protocol, development, and demo documentation for
  the Driver API-based implementation and stream API.
- Added documentation pointing ROS 2 Lyrical users to the upstream
  `rosidl::Buffer` CUDA buffer implementation for new projects.

### Package versions

All packages are bumped from `0.2.0` to `0.3.0`:

- `ros2_cuda_ipc_core`
- `ros2_cuda_ipc_msgs`
- `multi_process_image_fanout`
- `gpu_image_transport`
- `cuda_ipc_poc`

[0.3.0]: https://github.com/dskkato/ros2_cuda_ipc/compare/v0.2.0...v0.3.0
