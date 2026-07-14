// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda_runtime_api.h>

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "rclcpp/logger.hpp"
#include "ros2_cuda_ipc_core/buffer_descriptor.hpp"
#include "ros2_cuda_ipc_core/cuda/gpu_buffer_pool.hpp"
#include "ros2_cuda_ipc_core/memory_types.hpp"
#include "ros2_cuda_ipc_core/slot_controller.hpp"

namespace ros2_cuda_ipc_core::cuda {

class GpuBufferController;

class PublishSlot {
 public:
  PublishSlot(PublishSlot&& other) noexcept;
  PublishSlot& operator=(PublishSlot&& other) noexcept;
  PublishSlot(const PublishSlot&) = delete;
  PublishSlot& operator=(const PublishSlot&) = delete;
  ~PublishSlot();

  void* device_ptr() const noexcept;
  cudaError_t record_ready(cudaStream_t stream) noexcept;
  std::optional<BufferDescriptor> descriptor() const;
  bool commit_publish() noexcept;
  void cancel() noexcept;
  bool valid() const noexcept;

 private:
  friend class GpuBufferController;
  enum class State {
    reserved,
    ready_recorded,
    committed,
    cancelled,
    moved_from
  };

  PublishSlot(GpuBufferController* owner,
              SlotController::Reservation reservation) noexcept;
  void move_from(PublishSlot&& other) noexcept;

  GpuBufferController* owner_ = nullptr;
  SlotController::Reservation reservation_{};
  State state_ = State::moved_from;
};

class GpuBufferController {
 public:
  struct Config {
    std::string shm_name;
    std::size_t slot_count = 0;
    uint64_t byte_size = 0;
    int device_index = 0;
    std::chrono::milliseconds pending_ttl{0};
    MemoryBackendKind backend = MemoryBackendKind::CUDA_IPC;
  };

  GpuBufferController(Config config, rclcpp::Logger logger);
  ~GpuBufferController() = default;
  GpuBufferController(const GpuBufferController&) = delete;
  GpuBufferController& operator=(const GpuBufferController&) = delete;
  GpuBufferController(GpuBufferController&&) = delete;
  GpuBufferController& operator=(GpuBufferController&&) = delete;

  bool initialise();
  void reset() noexcept;
  bool is_initialised() const noexcept;
  std::optional<PublishSlot> acquire_for_publish(uint32_t pending_count);
  void reclaim_stale_pending();

 private:
  friend class PublishSlot;
  void* device_ptr(
      const SlotController::Reservation& reservation) const noexcept;
  cudaError_t record_ready(const SlotController::Reservation& reservation,
                           cudaStream_t stream) noexcept;
  std::optional<BufferDescriptor> descriptor(
      const SlotController::Reservation& reservation) const;
  void cancel(const SlotController::Reservation& reservation) noexcept;

  Config config_;
  rclcpp::Logger logger_;
  GpuBufferPool buffer_pool_;
  SlotController slot_controller_;
};

}  // namespace ros2_cuda_ipc_core::cuda
