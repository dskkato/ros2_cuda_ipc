// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>
#include <memory>
#include <optional>

#include "ros2_cuda_ipc_core/buffer_metadata/buffer_metadata.hpp"

namespace ros2_cuda_ipc_core::buffer_metadata {

/// Internal RAII owner for one process-shared buffer reference.
class BufferRef {
 public:
  struct PublisherReservation {
    std::shared_ptr<BufferMetadata> mapping;
    uint32_t slot_id = 0;
    uint32_t generation = 0;
  };

  static std::optional<uint32_t> current_generation(
      const std::shared_ptr<BufferMetadata>& mapping, uint32_t slot_id);
  static std::optional<uint32_t> current_refcount(
      const std::shared_ptr<BufferMetadata>& mapping, uint32_t slot_id);
  static std::optional<uint64_t> current_publish_timestamp_us(
      const std::shared_ptr<BufferMetadata>& mapping, uint32_t slot_id);

  static std::optional<PublisherReservation> reserve_for_publish(
      const std::shared_ptr<BufferMetadata>& mapping);
  static bool commit_publish(const std::shared_ptr<BufferMetadata>& mapping,
                             uint32_t slot_id, uint32_t generation) noexcept;
  static bool cancel_publish(const std::shared_ptr<BufferMetadata>& mapping,
                             uint32_t slot_id, uint32_t generation) noexcept;
  static BufferRef acquire(const std::shared_ptr<BufferMetadata>& mapping,
                           uint32_t slot_id, uint32_t generation);

  ~BufferRef();

  BufferRef(BufferRef&& other) noexcept;
  BufferRef& operator=(BufferRef&& other) noexcept;
  BufferRef(const BufferRef&) = delete;
  BufferRef& operator=(const BufferRef&) = delete;

  bool valid() const noexcept { return slot_meta_ != nullptr; }
  uint32_t slot_id() const noexcept { return slot_id_; }
  uint32_t generation() const noexcept { return generation_; }

 private:
  BufferRef() = default;
  BufferRef(std::shared_ptr<BufferMetadata> mapping, SlotMetadata* slot,
            uint32_t slot_id, uint32_t generation);

  void release() noexcept;

  std::shared_ptr<BufferMetadata> mapping_;
  SlotMetadata* slot_meta_ = nullptr;
  uint32_t slot_id_ = 0;
  uint32_t generation_ = 0;
};

}  // namespace ros2_cuda_ipc_core::buffer_metadata
