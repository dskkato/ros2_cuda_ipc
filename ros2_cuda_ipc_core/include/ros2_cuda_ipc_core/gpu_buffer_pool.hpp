#ifndef ROS2_CUDA_IPC_CORE_GPU_BUFFER_POOL_HPP_
#define ROS2_CUDA_IPC_CORE_GPU_BUFFER_POOL_HPP_

#include <cstddef>
#include <vector>
#include <mutex>
#include <optional>

namespace ros2_cuda_ipc_core {

struct Slot {
  bool in_use{false};
};

class GpuBufferPool {
public:
  explicit GpuBufferPool(std::size_t size);
  std::optional<std::size_t> borrow();
  bool release(std::size_t id);
  std::size_t capacity() const { return slots_.size(); }
private:
  std::vector<Slot> slots_;
  std::mutex mutex_;
};

}  // namespace ros2_cuda_ipc_core

#endif  // ROS2_CUDA_IPC_CORE_GPU_BUFFER_POOL_HPP_
