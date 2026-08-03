// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda.h>

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

#include "ros2_cuda_ipc_core/backend/memory_backend.hpp"
#include "ros2_cuda_ipc_core/buffer_metadata/buffer_ref.hpp"
#include "ros2_cuda_ipc_core/detail/cuda_driver_context.hpp"
#include "ros2_cuda_ipc_core/transport/buffer_descriptor.hpp"

namespace ros2_cuda_ipc_core::publisher {

class GpuBufferPool {
 public:
  /// One independently allocated GPU resource managed by this local pool.
  using GpuBufferBlock = backend::GpuBufferBlock;

  /// Temporary ownership of one publisher reservation.
  ///
  /// pool_index is an implementation detail of this pool. block_id is the
  /// process-global identity carried on the wire and must not be substituted
  /// with pool_index.
  struct BlockReservation {
    std::shared_ptr<buffer_metadata::BufferMetadata> mapping;
    uint32_t block_id = 0;
    uint64_t uid = 0;
    uint32_t publisher_pid = 0;
    uint32_t pool_index = 0;
  };

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
  uint32_t publisher_pid() const noexcept { return publisher_pid_; }

  /// Access a block resource by local pool index for backend-level tooling.
  /// Publisher reservation lifecycle should use the BlockReservation APIs.
  void* device_ptr(uint32_t pool_index) const noexcept;
  detail::CudaResult<void> record_ready(uint32_t pool_index,
                                        CUstream stream) noexcept;
  const GpuBufferBlock* resources(uint32_t pool_index) const noexcept;

  std::optional<BlockReservation> reserve_for_publish();
  void* device_ptr(const BlockReservation& reservation) const noexcept;
  detail::CudaResult<void> record_ready(const BlockReservation& reservation,
                                        CUstream stream) noexcept;
  std::optional<transport::BufferDescriptor> try_build_descriptor(
      const BlockReservation& reservation) const noexcept;
  bool commit(const BlockReservation& reservation) noexcept;
  bool cancel(const BlockReservation& reservation) noexcept;

 private:
  /// Pool is a publisher-local block allocation and reuse strategy.
  bool allocate_blocks();
  void destroy_blocks() noexcept;
  bool owns(const BlockReservation& reservation) const noexcept;

  std::size_t block_count_;
  std::vector<GpuBufferBlock> blocks_;
  mutable std::mutex mutex_;
  std::size_t next_block_ = 0;
  uint64_t byte_size_ = 0;
  int device_index_ = -1;
  uint32_t publisher_pid_ = 0;
  bool initialised_ = false;
  std::shared_ptr<detail::CudaDeviceContext> context_;
};

}  // namespace ros2_cuda_ipc_core::publisher
