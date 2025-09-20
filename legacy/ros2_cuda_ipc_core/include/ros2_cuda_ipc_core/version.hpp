#ifndef ROS2_CUDA_IPC_CORE_VERSION_HPP_
#define ROS2_CUDA_IPC_CORE_VERSION_HPP_

#include <cstdint>

namespace ros2_cuda_ipc_core {

// ABI version for wire compatibility (embedded in messages)
inline constexpr std::uint32_t kAbiVersion = 1;

}  // namespace ros2_cuda_ipc_core

#endif  // ROS2_CUDA_IPC_CORE_VERSION_HPP_
