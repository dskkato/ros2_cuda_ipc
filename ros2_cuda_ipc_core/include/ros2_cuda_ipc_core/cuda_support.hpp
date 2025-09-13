#ifndef ROS2_CUDA_IPC_CORE_CUDA_SUPPORT_HPP_
#define ROS2_CUDA_IPC_CORE_CUDA_SUPPORT_HPP_

#include <cstddef>

namespace ros2_cuda_ipc_core {

// A portable representation of a CUDA IPC memory handle (64 bytes).
struct CudaIpcMemHandle {
  unsigned char reserved[64];
};

// A portable representation of a CUDA IPC event handle (64 bytes).
struct CudaIpcEventHandle {
  unsigned char reserved[64];
};

// Returns true if CUDA runtime is available (device count > 0)
bool cuda_is_available();

// Allocates device memory of given size. Returns nullptr on failure.
void* cuda_allocate(std::size_t bytes);

// Frees device memory previously allocated with cuda_allocate.
// Returns true if freed successfully.
bool cuda_free(void* ptr);

// Exports an IPC handle for a device pointer.
// Returns true on success.
bool cuda_ipc_get_mem_handle(void* device_ptr, CudaIpcMemHandle* out_handle);

// Opens an IPC handle and returns a mapped device pointer, or nullptr on
// failure.
void* cuda_ipc_open_mem_handle(const CudaIpcMemHandle& handle);

// Closes a previously opened IPC mapping. Returns true on success.
bool cuda_ipc_close_mem_handle(void* device_ptr);

// Creates a CUDA interprocess-capable event. Returns opaque handle or nullptr
// on failure.
void* cuda_event_create();

// Destroys a CUDA event previously created or opened. Returns true on success.
bool cuda_event_destroy(void* evt);

// Records the event on the default stream (0). Returns true on success.
bool cuda_event_record(void* evt);

// Exports an IPC event handle for a given event. Returns true on success.
bool cuda_event_get_ipc_handle(void* evt, CudaIpcEventHandle* out_handle);

// Opens an IPC event handle in the current process. Returns event or nullptr on
// failure.
void* cuda_ipc_open_event_handle(const CudaIpcEventHandle& handle);

// Queries event status. Returns true if event has completed (or if
// unsupported), false if not ready.
bool cuda_event_query(void* evt);

// CUDA stream helpers (opaque void* to avoid leaking CUDA headers)
void* cuda_stream_create();
bool cuda_stream_destroy(void* stream);
// Waits for evt on the given stream. Returns true on success.
bool cuda_stream_wait_event(void* stream, void* evt);
// Optional: synchronize stream (blocks until complete). Returns true on
// success.
bool cuda_stream_synchronize(void* stream);

}  // namespace ros2_cuda_ipc_core

#endif  // ROS2_CUDA_IPC_CORE_CUDA_SUPPORT_HPP_
