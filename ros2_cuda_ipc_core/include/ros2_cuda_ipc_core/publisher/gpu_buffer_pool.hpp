// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "ros2_cuda_ipc_core/backend/memory_backend.hpp"
#include "ros2_cuda_ipc_core/buffer_metadata/buffer_metadata.hpp"
#include "ros2_cuda_ipc_core/detail/cuda_driver_context.hpp"

namespace ros2_cuda_ipc_core::publisher {

/// One GPU allocation and its one-and-only shared BlockMetadata object.
struct GpuBufferBlock {
  uint32_t publisher_pid = 0;
  uint32_t block_id = 0;
  std::string metadata_shm_name;
  std::shared_ptr<buffer_metadata::BufferMetadata> metadata;
  backend::BlockResources resources;
};

class GpuBufferPool {
 public:
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

  GpuBufferBlock* block(uint32_t index) noexcept;
  const GpuBufferBlock* block(uint32_t index) const noexcept;
  void* device_ptr(uint32_t block_index) const noexcept;
  detail::CudaResult<void> record_ready(uint32_t block_index,
                                        CUstream stream) noexcept;
  const backend::BlockResources* resources(uint32_t block_index) const noexcept;

 private:
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
