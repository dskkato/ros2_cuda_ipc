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

Ready-event operations use this foundation: each publisher pool and imported
subscriber event retains the device primary context, and each Driver API call
is enclosed by a short-lived guard. The public stream type is `CUstream`, while
CUDA Runtime-created streams remain valid because `cudaStream_t` and `CUstream`
are compatible handles. The library never resets contexts with
`cuDevicePrimaryCtxReset()` or `cudaDeviceReset()`.

## Current VMM assumptions

The VMM publisher path currently calls `cudaSetDevice()` in
`GpuBufferPool::initialise()` before invoking the backend. That Runtime API
call prepares and makes the device primary context current on the allocation
thread. The VMM backend then calls `cuMemGetAllocationGranularity`,
`cuMemAddressReserve`, `cuMemCreate`, `cuMemMap`, `cuMemSetAccess`, and
`cuMemExportToShareableHandle`; these calls currently rely on that current
context. The backend separately calls `cuInit(0)` through a local
`std::call_once`.

The VMM importer retains and activates the message's device primary context
before `cuMemImportFromShareableHandle`, allocation-granularity lookup, address
reservation, mapping, access setup, and Driver API event import. The retained
context is carried with the imported resource so cleanup can establish the same
context later.

VMM slot cleanup runs from `GpuBufferPool::destroy_slots()` and the
`VmmSlotState` destructor. The Unix FD server has its own worker thread, but
that thread only serves file descriptors and does not call CUDA APIs. The
imported-resource cleanup path now carries the retained context for its Driver
API cleanup; VMM publisher-side allocation cleanup remains part of the memory
path migration.

## Later integration points

Future Driver API migration should add scoped guards around:

* VMM allocation, import, map, unmap, and release operations;
* CUDA IPC memory export, import, and close operations;
* Publisher and Subscriber resource classes that own those CUDA objects.

Publisher and Subscriber resource ownership, ROS messages, lease handling, and
the mapping cache are intentionally unchanged here.
