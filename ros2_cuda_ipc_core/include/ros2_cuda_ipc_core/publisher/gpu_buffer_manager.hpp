// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda.h>

#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>

#include "ros2_cuda_ipc_core/buffer_metadata/buffer_metadata.hpp"
#include "ros2_cuda_ipc_core/publisher/gpu_buffer_pool.hpp"
#include "ros2_cuda_ipc_core/transport/buffer_descriptor.hpp"

namespace ros2_cuda_ipc_core::publisher {

class GpuBufferManager;

class PublishSlot {
 public:
  PublishSlot(PublishSlot&& other) noexcept;
  PublishSlot& operator=(PublishSlot&& other) noexcept;
  PublishSlot(const PublishSlot&) = delete;
  PublishSlot& operator=(const PublishSlot&) = delete;
  ~PublishSlot() noexcept;

  void* device_ptr() const noexcept;
  [[nodiscard]] detail::CudaResult<transport::BlockDescriptor> prepare_publish(
      CUstream stream) noexcept;
  void cancel() noexcept;
  bool valid() const noexcept { return owner_ != nullptr; }

 private:
  friend class GpuBufferManager;
  struct Reservation {
    std::shared_ptr<buffer_metadata::BufferMetadata> mapping;
    uint32_t block_index = 0;
    uint32_t publisher_pid = 0;
    uint32_t block_id = 0;
    uint64_t uid = 0;
  };

  PublishSlot(GpuBufferManager* owner, Reservation reservation) noexcept;
  void move_from(PublishSlot&& other) noexcept;
  void quarantine(const char* step,
                  const detail::CudaDriverError* error = nullptr) noexcept;

  GpuBufferManager* owner_ = nullptr;
  Reservation reservation_{};
};

class GpuBufferManager {
 public:
  struct Config {
    std::size_t block_count = 0;
    uint64_t byte_size = 0;
    int device_index = 0;
  };

  explicit GpuBufferManager(Config config);
  ~GpuBufferManager();
  GpuBufferManager(const GpuBufferManager&) = delete;
  GpuBufferManager& operator=(const GpuBufferManager&) = delete;
  GpuBufferManager(GpuBufferManager&&) = delete;
  GpuBufferManager& operator=(GpuBufferManager&&) = delete;

  bool initialise();
  void reset() noexcept;
  bool is_initialised() const noexcept;
  [[nodiscard]] std::optional<PublishSlot> acquire_for_publish();

 private:
  friend class PublishSlot;
  void* device_ptr(const PublishSlot::Reservation& reservation) const noexcept;
  detail::CudaResult<void> record_ready(
      const PublishSlot::Reservation& reservation, CUstream stream) noexcept;
  std::optional<transport::BlockDescriptor> try_build_descriptor(
      const PublishSlot::Reservation& reservation) const noexcept;
  bool commit(const PublishSlot::Reservation& reservation) noexcept;
  bool cancel(const PublishSlot::Reservation& reservation) noexcept;

  Config config_;
  GpuBufferPool buffer_pool_;
  std::atomic<uint32_t> next_block_index_{0};
};

}  // namespace ros2_cuda_ipc_core::publisher
