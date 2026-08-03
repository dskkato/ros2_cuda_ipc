// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda.h>

#include <cstdint>
#include <memory>
#include <vector>

#include "ros2_cuda_ipc_core/backend/memory_backend.hpp"
#include "ros2_cuda_ipc_core/detail/cuda_driver_context.hpp"

namespace ros2_cuda_ipc_core::publisher {

class GpuBufferPool {
 public:
  /// One independently allocated GPU resource managed by this local pool.
  using GpuBufferBlock = backend::GpuBufferBlock;

  explicit GpuBufferPool(std::size_t block_count);
  ~GpuBufferPool();

  GpuBufferPool(const GpuBufferPool&) = delete;
  GpuBufferPool& operator=(const GpuBufferPool&) = delete;
  GpuBufferPool(GpuBufferPool&&) = delete;
  GpuBufferPool& operator=(GpuBufferPool&&) = delete;

  bool initialise(uint64_t byte_size, int device_index);
  void reset() noexcept;
  bool is_initialised() const noexcept { return initialised_; }
  bool matches(uint64_t byte_size, int device_index) const noexcept;
  std::size_t size() const noexcept { return blocks_.size(); }
  uint64_t byte_size() const noexcept { return byte_size_; }
  int device_index() const noexcept { return device_index_; }

  void* device_ptr(uint32_t pool_index) const noexcept;
  detail::CudaResult<void> record_ready(uint32_t pool_index,
                                        CUstream stream) noexcept;
  const GpuBufferBlock* resources(uint32_t pool_index) const noexcept;
  bool set_shared_metadata(
      uint32_t pool_index, uint32_t block_id,
      std::shared_ptr<buffer_metadata::BufferMetadata> metadata) noexcept;

 private:
  /// Pool is a publisher-local block allocation and reuse strategy.
  bool allocate_blocks();
  void destroy_blocks() noexcept;

  std::size_t block_count_;
  std::vector<GpuBufferBlock> blocks_;
  uint64_t byte_size_ = 0;
  int device_index_ = -1;
  bool initialised_ = false;
  std::shared_ptr<detail::CudaDeviceContext> context_;
};

}  // namespace ros2_cuda_ipc_core::publisher
