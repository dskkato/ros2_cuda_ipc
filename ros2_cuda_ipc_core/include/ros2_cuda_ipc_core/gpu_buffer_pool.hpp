#ifndef ROS2_CUDA_IPC_CORE_GPU_BUFFER_POOL_HPP_
#define ROS2_CUDA_IPC_CORE_GPU_BUFFER_POOL_HPP_

#include <cstddef>
#include <mutex>
#include <optional>
#include <vector>

#include "ros2_cuda_ipc_core/cuda_support.hpp"

namespace ros2_cuda_ipc_core {

struct Slot {
  bool in_use{false};
};

class GpuBufferPool {
 public:
  explicit GpuBufferPool(std::size_t size);
  // Optional CUDA-aware constructor. If use_cuda is true and CUDA is available,
  // pre-allocates device buffers of bytes_per_slot for each slot.
  GpuBufferPool(std::size_t size, std::size_t bytes_per_slot, bool use_cuda);
  ~GpuBufferPool();
  std::optional<std::size_t> borrow();
  bool release(std::size_t id);
  std::size_t capacity() const { return slots_.size(); }
  // Returns device pointer for a slot (nullptr if not CUDA or invalid).
  void* device_ptr(std::size_t id) const;
  // Exports CUDA IPC handle for a slot's device buffer.
  // Returns true on success, false if CUDA disabled or invalid id.
  bool ipc_handle(std::size_t id, CudaIpcMemHandle& out_handle) const;

  // Records the slot's ready event (default stream). Returns true on success.
  bool record_ready(std::size_t id);

  // Exports CUDA IPC handle for a slot's ready event.
  bool ipc_event_handle(std::size_t id, CudaIpcEventHandle& out_handle) const;

 private:
  std::vector<Slot> slots_;
  std::vector<void*> device_ptrs_;
  std::vector<void*> events_;
  std::size_t bytes_per_slot_{0};
  bool using_cuda_{false};
  std::mutex mutex_;
};

}  // namespace ros2_cuda_ipc_core

#endif  // ROS2_CUDA_IPC_CORE_GPU_BUFFER_POOL_HPP_
