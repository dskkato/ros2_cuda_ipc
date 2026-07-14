// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda_runtime_api.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/logger.hpp"
#include "ros2_cuda_ipc_core/cuda/gpu_buffer_pool.hpp"
#include "ros2_cuda_ipc_core/memory_types.hpp"
#include "ros2_cuda_ipc_core/slot_controller.hpp"
#include "ros2_cuda_ipc_core/view/buffer_view.hpp"

namespace ros2_cuda_ipc_core::cuda {

// Deprecated compatibility facade. New Publisher code must use
// GpuBufferController and PublishSlot. Kept temporarily for source migration.
class GpuLeasePool {
 public:
  struct Config {
    std::string shm_name;
    std::size_t slot_count = 0;
    std::chrono::milliseconds pending_ttl{0};
    ros2_cuda_ipc_core::MemoryBackendKind backend =
        ros2_cuda_ipc_core::MemoryBackendKind::CUDA_IPC;
  };

  struct Slot {
    uint32_t index = 0;
    void* device_ptr = nullptr;
    cudaEvent_t event = nullptr;
    cudaIpcEventHandle_t event_handle{};
    uint32_t generation = 0;
    std::chrono::steady_clock::time_point pending_deadline{};
    ros2_cuda_ipc_core::MemoryBackendKind backend =
        ros2_cuda_ipc_core::MemoryBackendKind::CUDA_IPC;
    ros2_cuda_ipc_core::MemoryHandlePayload mem_handle{};
  };

  explicit GpuLeasePool(Config config, rclcpp::Logger logger);
  ~GpuLeasePool();

  GpuLeasePool(const GpuLeasePool&) = delete;
  GpuLeasePool& operator=(const GpuLeasePool&) = delete;
  GpuLeasePool(GpuLeasePool&&) = delete;
  GpuLeasePool& operator=(GpuLeasePool&&) = delete;

  bool initialise(uint64_t frame_size_bytes, int device_index);
  void reset() noexcept;

  bool is_initialised() const noexcept { return initialised_; }
  bool matches(uint64_t frame_size_bytes, int device_index) const noexcept;

  Slot* acquire(std::size_t subscriber_count);
  void reclaim_stale_pending();
  bool cancel_pending(Slot& slot);

  uint64_t frame_size_bytes() const noexcept { return frame_size_bytes_; }
  int device_index() const noexcept { return device_index_; }

  view::BufferView buffer_view_from(const Slot& slot) const;

 private:
  void sync_slot(uint32_t slot_id);
  void destroy_slots() noexcept;

  Config config_;
  std::vector<Slot> slots_;
  uint64_t frame_size_bytes_ = 0;
  int device_index_ = -1;
  bool initialised_ = false;
  rclcpp::Logger logger_;
  GpuBufferPool buffer_pool_;
  ros2_cuda_ipc_core::SlotController slot_controller_;
};

}  // namespace ros2_cuda_ipc_core::cuda
