// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda.h>

#include <cstdint>
#include <vector>

#include "ros2_cuda_ipc_core/backend/memory_backend.hpp"
#include "ros2_cuda_ipc_core/detail/cuda_driver_context.hpp"

namespace ros2_cuda_ipc_core::publisher {

class GpuBufferPool {
 public:
  using SlotBackendState = backend::SlotBackendState;
  using SlotResources = backend::SlotResources;
  using MemoryBackend = backend::MemoryBackend;

  explicit GpuBufferPool(std::size_t slot_count);
  GpuBufferPool(std::size_t slot_count,
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
  detail::CudaResult<void> record_ready(uint32_t slot_id,
                                        CUstream stream) noexcept;
  const SlotResources* resources(uint32_t slot_id) const noexcept;

 private:
  bool allocate_slots();
  void destroy_slots() noexcept;

  std::size_t slot_count_;
  std::vector<SlotResources> slots_;
  uint64_t byte_size_ = 0;
  int device_index_ = -1;
  bool initialised_ = false;
  std::unique_ptr<MemoryBackend> memory_backend_;
  std::shared_ptr<detail::CudaDeviceContext> context_;
};

}  // namespace ros2_cuda_ipc_core::publisher
