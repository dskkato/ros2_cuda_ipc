# Changelog

This document records the user-visible changes between releases of
`ros2_cuda_ipc`.

## Unreleased

- Replaced pool-wide shared metadata with one POSIX shared `BlockMetadata`
  object per GPU block. The object is named from `publisher_pid` and
  `block_id`, and contains only the block UID, reference count, and publication
  timestamp.
- Changed the wire descriptor to identify blocks with `publisher_pid`,
  `block_id`, and `uid`, while retaining only the CUDA resource information
  needed for import and synchronization. Pool identity, slot identity, and
  explicit shared-memory names are no longer part of the protocol.
- Removed the pool metadata header/array, metadata capacity APIs, publisher
  instance identity, `BufferMetadataManager`, and the legacy metadata lookup
  paths. This is an intentional breaking protocol and API change.
- Updated subscriber metadata and imported-resource caches to use block
  identity, invalidate stale mappings on UID mismatch, and reattach before
  retrying. Normal publisher shutdown now unlinks every block metadata object.
- Added per-block metadata, stale-UID, cache reattach, and cleanup coverage;
  updated Python bindings, examples, and ownership documentation.

- Removed the CUDA IPC memory-sharing backend. The core library now uses CUDA
  VMM plus POSIX file descriptors exclusively.
- Removed `BufferCore.backend` and the publisher/launch backend-selection APIs.
  Existing messages using the old wire shape are not compatible with this
  release.
- Changed `BufferCore.vmm_socket_path` to carry the VMM Unix socket path as a
  string, removed `MemoryHandlePayload`, and updated the internal descriptors,
  caches, Python API, and tests accordingly. The VMM FD test's fixed 64-byte
  payload is intentional: it exercises a raw child-process protocol without
  using a ROS message and is not a mirror of the message field.
- The standalone `cuda_ipc_poc` comparison programs remain available, but are
  no longer library backends.

## [0.4.0] - 2026-07-30

This release adds the first supported Python subscriber API and simplifies
the publisher and subscriber lifetime contracts around asynchronous CUDA
work.

### Highlights

- Added the `ros2_cuda_ipc_py` subscriber binding for zero-copy `GpuImage`
  access from Python, including CuPy and DLPack integration, CUDA stream
  validation, image metadata, examples, and regression tests.
- Reworked subscriber reads around `BufferMapper` and the move-only
  `ReadHandle` API. Imported resources, publication leases, producer waits,
  and completion handling now follow the read-handle lifecycle, with release
  deferred until asynchronous work completes.
- Simplified `PublishSlot` preparation to the single
  `prepare_publish(CUstream)` operation, which builds the descriptor,
  records the ready event, and commits the reservation.
- Simplified lease slot reuse to generation, reference-count, and publication
  timestamp tracking with a fixed grace period; removed pending metadata and
  pending-TTL configuration.
- Replaced core `RCLCPP_*` logging and propagated logger objects with named
  `rcutils` logging macros.

### Breaking changes and migration notes

- Update publisher code to use `PublishSlot::prepare_publish(CUstream)`;
  `record_ready()`, `descriptor()`, and `commit_publish()` are no longer part
  of the public `PublishSlot` API.
- Remove uses of pending-lease metadata and pending-TTL configuration APIs.
  Slot reuse is now protected by the fixed grace period and active lease
  reference counts.
- Update subscriber code to use `BufferMapper` and `ReadHandle`. Legacy
  buffer-view, lease-handle, and import-cache implementation headers are no
  longer part of the installed subscriber surface.
- Core APIs no longer carry `rclcpp::Logger` instances or logger parameters;
  diagnostics use fixed named `rcutils` loggers.

### Tests and documentation

- Added Python subscriber, DLPack, stream-validation, ownership, and
  asynchronous lifetime regression coverage.
- Expanded publisher, lease, subscriber, and cache tests for the revised
  ownership and failure semantics.
- Updated the publisher, lease protocol, subscriber, development, and demo
  documentation and added Python subscriber examples.

### Package versions

All ROS 2 packages and the Python native extension are bumped from `0.3.0`
to `0.4.0`:

- `ros2_cuda_ipc_core`
- `ros2_cuda_ipc_msgs`
- `ros2_cuda_ipc_py`
- `multi_process_image_fanout`
- `gpu_image_transport`
- `cuda_ipc_poc`

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
[0.4.0]: https://github.com/dskkato/ros2_cuda_ipc/compare/v0.3.0...v0.4.0
