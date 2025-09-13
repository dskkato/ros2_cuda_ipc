#ifndef ROS2_CUDA_IPC_CORE_CUDA_SUPPORT_HPP_
#define ROS2_CUDA_IPC_CORE_CUDA_SUPPORT_HPP_

#include <cstddef>

namespace ros2_cuda_ipc_core {

// Returns true if CUDA runtime is available (device count > 0)
bool cuda_is_available();

// Allocates device memory of given size. Returns nullptr on failure.
void* cuda_allocate(std::size_t bytes);

// Frees device memory previously allocated with cuda_allocate.
// Returns true if freed successfully.
bool cuda_free(void* ptr);

}  // namespace ros2_cuda_ipc_core

#endif  // ROS2_CUDA_IPC_CORE_CUDA_SUPPORT_HPP_
