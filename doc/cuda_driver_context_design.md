# CUDA Driver context foundation

This note describes the internal context contract implemented in
`ros2_cuda_ipc_core/detail/cuda_driver_context.hpp`.

## Contract

`ros2_cuda_ipc` uses each CUDA device's primary context. A primary context is
shared with the CUDA Runtime API and with frameworks such as PyTorch and CuPy;
the library therefore retains one reference, never calls a reset API, and does
not explicitly destroy the primary context.

`CudaDriver::initialize()` calls `cuInit(0)` once per process through
`std::call_once`. All callers observe the same Driver API result. A
`CudaDeviceContext` validates the device ordinal, resolves its `CUdevice`, and
retains the corresponding primary context. Its destructor releases only that
retain reference.

Driver API operations that need a context must use a short-lived
`CudaContextGuard`. The guard uses `cuCtxPushCurrent()` and
`cuCtxPopCurrent()`, so the caller's previous current context is restored on
the same CPU thread. Guards are move-only and must not be moved to a different
thread for destruction because CUDA context stacks are thread-local. Cleanup
errors are logged and never thrown from the destructor.

The foundation does not migrate resource operations in this change. In
particular, it does not change CUDA Runtime streams or error types, and it does
not reset contexts with `cuDevicePrimaryCtxReset()` or `cudaDeviceReset()`.

## Current VMM assumptions

The VMM publisher path currently calls `cudaSetDevice()` in
`GpuBufferPool::initialise()` before invoking the backend. That Runtime API
call prepares and makes the device primary context current on the allocation
thread. The VMM backend then calls `cuMemGetAllocationGranularity`,
`cuMemAddressReserve`, `cuMemCreate`, `cuMemMap`, `cuMemSetAccess`, and
`cuMemExportToShareableHandle`; these calls currently rely on that current
context. The backend separately calls `cuInit(0)` through a local
`std::call_once`.

The VMM importer calls `cuInit(0)` before
`cuMemImportFromShareableHandle`, allocation-granularity lookup, address
reservation, mapping, and access setup, but does not explicitly activate the
message's device primary context. It therefore has an implicit current-context
precondition that should be removed during later migration.

VMM slot cleanup runs from `GpuBufferPool::destroy_slots()` and the
`VmmSlotState` destructor. The state can outlive the callback that created it;
the current code does not attach a Driver context to the state or establish a
thread-local context before its `cuMemUnmap`, `cuMemAddressFree`, and
`cuMemRelease` calls. The Unix FD server has its own worker thread, but that
thread only serves file descriptors and does not call CUDA APIs. Later cleanup
work must make the context and thread ownership explicit.

## Later integration points

Future Driver API migration should add scoped guards around:

* VMM allocation, import, map, unmap, and release operations;
* CUDA IPC event creation, recording, export, import, waiting, and destruction;
* CUDA IPC memory export, import, and close operations;
* Publisher and Subscriber resource classes that own those CUDA objects.

Publisher and Subscriber resource ownership, ROS messages, lease handling, and
the mapping cache are intentionally unchanged here.
