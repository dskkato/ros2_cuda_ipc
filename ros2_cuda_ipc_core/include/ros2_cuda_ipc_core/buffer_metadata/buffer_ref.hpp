// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>
#include <memory>
#include <optional>

#include "ros2_cuda_ipc_core/buffer_metadata/buffer_metadata.hpp"

namespace ros2_cuda_ipc_core::buffer_metadata {

/// Internal RAII owner for one process-shared reference to one GPU block.
class BufferRef {
 public:
  struct PublisherReservation {
    std::shared_ptr<BufferMetadata> mapping;
    uint64_t uid = 0;
  };

  static std::optional<uint64_t> current_uid(
      const std::shared_ptr<BufferMetadata>& mapping);
  static std::optional<uint32_t> current_refcount(
      const std::shared_ptr<BufferMetadata>& mapping);
  static std::optional<uint64_t> current_publish_timestamp_us(
      const std::shared_ptr<BufferMetadata>& mapping);

  static std::optional<PublisherReservation> reserve_for_publish(
      const std::shared_ptr<BufferMetadata>& mapping);
  static bool commit_publish(const std::shared_ptr<BufferMetadata>& mapping,
                             uint64_t uid) noexcept;
  static bool cancel_publish(const std::shared_ptr<BufferMetadata>& mapping,
                             uint64_t uid) noexcept;
  static BufferRef acquire(const std::shared_ptr<BufferMetadata>& mapping,
                           uint64_t uid);

  ~BufferRef();

  BufferRef(BufferRef&& other) noexcept;
  BufferRef& operator=(BufferRef&& other) noexcept;
  BufferRef(const BufferRef&) = delete;
  BufferRef& operator=(const BufferRef&) = delete;

  bool valid() const noexcept { return block_meta_ != nullptr; }
  uint64_t uid() const noexcept { return uid_; }

 private:
  BufferRef() = default;
  BufferRef(std::shared_ptr<BufferMetadata> mapping, BlockMetadata* block,
            uint64_t uid);

  void release() noexcept;

  std::shared_ptr<BufferMetadata> mapping_;
  BlockMetadata* block_meta_ = nullptr;
  uint64_t uid_ = 0;
};

}  // namespace ros2_cuda_ipc_core::buffer_metadata
