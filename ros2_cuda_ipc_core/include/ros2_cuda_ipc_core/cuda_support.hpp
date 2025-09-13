#ifndef ROS2_CUDA_IPC_CORE_CUDA_SUPPORT_HPP_
#define ROS2_CUDA_IPC_CORE_CUDA_SUPPORT_HPP_

#include <cstddef>

namespace ros2_cuda_ipc_core {

// A portable representation of a CUDA IPC memory handle (64 bytes).
struct CudaIpcMemHandle {
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

}  // namespace ros2_cuda_ipc_core

#endif  // ROS2_CUDA_IPC_CORE_CUDA_SUPPORT_HPP_
