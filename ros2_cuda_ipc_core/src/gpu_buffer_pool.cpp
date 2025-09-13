#include "ros2_cuda_ipc_core/gpu_buffer_pool.hpp"

namespace ros2_cuda_ipc_core {

GpuBufferPool::GpuBufferPool(std::size_t size) : slots_(size) {}

std::optional<std::size_t> GpuBufferPool::borrow() {
  std::lock_guard<std::mutex> lock(mutex_);
  for (std::size_t i = 0; i < slots_.size(); ++i) {
    if (!slots_[i].in_use) {
      slots_[i].in_use = true;
      return i;
    }
  }
  return std::nullopt;
}

void GpuBufferPool::release(std::size_t id) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (id < slots_.size()) {
    slots_[id].in_use = false;
  }
}

}  // namespace ros2_cuda_ipc_core
