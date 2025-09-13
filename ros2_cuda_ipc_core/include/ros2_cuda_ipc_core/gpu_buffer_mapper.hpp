#ifndef ROS2_CUDA_IPC_CORE_GPU_BUFFER_MAPPER_HPP_
#define ROS2_CUDA_IPC_CORE_GPU_BUFFER_MAPPER_HPP_

#include <cstdint>
#include <mutex>
#include <unordered_map>

#include "ros2_cuda_ipc_core/cuda_support.hpp"

namespace ros2_cuda_ipc_core {

// Caches opened CUDA IPC memory/event handles per pool slot id to avoid
// repeated cudaIpcOpen* overhead.
class GpuBufferMapper {
 public:
  GpuBufferMapper() = default;
  ~GpuBufferMapper();

  // Opens and caches a device memory mapping for a given slot.
  // Returns device pointer, or nullptr on failure.
  void* open_memory(uint32_t slot_id, const CudaIpcMemHandle& mem_handle);

  // Opens and caches an event for a given slot. Returns event pointer, or
  // nullptr on failure.
  void* open_event(uint32_t slot_id, const CudaIpcEventHandle& evt_handle);

  // Returns cached pointers (nullptr if none).
  void* get_memory(uint32_t slot_id) const;
  void* get_event(uint32_t slot_id) const;

  // Waits on cached event for slot on the provided stream. Returns false if
  // event not cached or wait fails.
  bool wait_ready(uint32_t slot_id, void* stream) const;

  // Closes and removes cached resources for a slot.
  void close_slot(uint32_t slot_id);

  // Closes all cached mappings/events.
  void reset();

 private:
  struct Entry {
    void* mem{nullptr};
    void* evt{nullptr};
  };

  mutable std::mutex mutex_;
  std::unordered_map<uint32_t, Entry> cache_;
};

}  // namespace ros2_cuda_ipc_core

#endif  // ROS2_CUDA_IPC_CORE_GPU_BUFFER_MAPPER_HPP_
