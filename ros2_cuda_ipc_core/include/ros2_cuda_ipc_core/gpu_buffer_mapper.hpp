#ifndef ROS2_CUDA_IPC_CORE_GPU_BUFFER_MAPPER_HPP_
#define ROS2_CUDA_IPC_CORE_GPU_BUFFER_MAPPER_HPP_

#include <cstdint>
#include <mutex>
#include <string>
#include <string_view>
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
  cudaEvent_t open_event(uint32_t slot_id,
                         const CudaIpcEventHandle& evt_handle);

  // Returns cached pointers (nullptr if none).
  void* get_memory(uint32_t slot_id) const;
  cudaEvent_t get_event(uint32_t slot_id) const;

  // Waits on cached event for slot on the provided stream. Returns false if
  // event not cached or wait fails.
  bool wait_ready(uint32_t slot_id, cudaStream_t stream) const;

  // Validates ABI version and device UUID for the slot. If a mismatch is
  // detected compared to cached values, the slot is closed. Returns true when
  // handles remain valid, false when they were reset due to mismatch.
  bool validate_handles(uint32_t slot_id, uint32_t abi_version,
                        std::string_view device_uuid);

  // Closes and removes cached resources for a slot.
  void close_slot(uint32_t slot_id);

  // Closes all cached mappings/events.
  void reset();

 private:
  struct Entry {
    void* mem{nullptr};
    cudaEvent_t evt{nullptr};
    uint32_t abi_version{0};
    std::string device_uuid;
  };

  // Internal helper expecting mutex_ to be locked.
  void close_slot_locked(uint32_t slot_id);

  mutable std::mutex mutex_;
  std::unordered_map<uint32_t, Entry> cache_;
};

}  // namespace ros2_cuda_ipc_core

#endif  // ROS2_CUDA_IPC_CORE_GPU_BUFFER_MAPPER_HPP_
