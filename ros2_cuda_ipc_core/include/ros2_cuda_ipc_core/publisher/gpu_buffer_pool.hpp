// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda_runtime_api.h>

#include <cstdint>
#include <optional>
#include <vector>

#include "rclcpp/logger.hpp"
#include "ros2_cuda_ipc_core/backend/memory_backend.hpp"
#include "ros2_cuda_ipc_core/transport/memory_types.hpp"

namespace ros2_cuda_ipc_core::publisher {

class GpuBufferPool {
 public:
  using SlotBackendState = backend::SlotBackendState;
  using SlotResources = backend::SlotResources;
  using MemoryBackend = backend::MemoryBackend;

  GpuBufferPool(std::size_t slot_count, transport::MemoryBackendKind backend,
                rclcpp::Logger logger);
  GpuBufferPool(std::size_t slot_count, transport::MemoryBackendKind backend,
                rclcpp::Logger logger,
                std::unique_ptr<MemoryBackend> memory_backend);
  ~GpuBufferPool();

  GpuBufferPool(const GpuBufferPool&) = delete;
  GpuBufferPool& operator=(const GpuBufferPool&) = delete;
  GpuBufferPool(GpuBufferPool&&) = delete;
  GpuBufferPool& operator=(GpuBufferPool&&) = delete;

  bool initialise(uint64_t byte_size, int device_index);
  void reset() noexcept;
  bool is_initialised() const noexcept { return initialised_; }
  bool matches(uint64_t byte_size, int device_index) const noexcept;
  std::size_t size() const noexcept { return slots_.size(); }
  uint64_t byte_size() const noexcept { return byte_size_; }
  int device_index() const noexcept { return device_index_; }

  void* device_ptr(uint32_t slot_id) const noexcept;
  cudaError_t record_ready(uint32_t slot_id, cudaStream_t stream) noexcept;
  const SlotResources* resources(uint32_t slot_id) const noexcept;

 private:
  bool allocate_slots();
  void destroy_slots() noexcept;

  std::size_t slot_count_;
  transport::MemoryBackendKind backend_kind_;
  rclcpp::Logger logger_;
  std::vector<SlotResources> slots_;
  uint64_t byte_size_ = 0;
  int device_index_ = -1;
  bool initialised_ = false;
  std::unique_ptr<MemoryBackend> memory_backend_;
};

}  // namespace ros2_cuda_ipc_core::publisher
