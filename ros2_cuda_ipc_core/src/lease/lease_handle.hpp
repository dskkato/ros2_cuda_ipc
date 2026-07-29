// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>
#include <memory>
#include <optional>

#include "ros2_cuda_ipc_core/lease/lease_mapping.hpp"

namespace ros2_cuda_ipc_core::lease {

/// Internal RAII owner for one shared-memory publication slot.
class LeaseHandle {
 public:
  struct PublisherReservation {
    std::shared_ptr<LeaseMapping> mapping;
    uint32_t slot_id = 0;
    uint32_t generation = 0;
  };

  static std::optional<uint32_t> current_generation(
      const std::shared_ptr<LeaseMapping>& mapping, uint32_t slot_id);
  static std::optional<uint32_t> current_refcount(
      const std::shared_ptr<LeaseMapping>& mapping, uint32_t slot_id);
  static std::optional<uint64_t> current_publish_timestamp_us(
      const std::shared_ptr<LeaseMapping>& mapping, uint32_t slot_id);

  static std::optional<PublisherReservation> reserve_for_publish(
      const std::shared_ptr<LeaseMapping>& mapping);
  static bool commit_publish(const std::shared_ptr<LeaseMapping>& mapping,
                             uint32_t slot_id, uint32_t generation) noexcept;
  static bool cancel_publish(const std::shared_ptr<LeaseMapping>& mapping,
                             uint32_t slot_id, uint32_t generation) noexcept;
  static LeaseHandle acquire(const std::shared_ptr<LeaseMapping>& mapping,
                             uint32_t slot_id, uint32_t generation);

  ~LeaseHandle();

  LeaseHandle(LeaseHandle&& other) noexcept;
  LeaseHandle& operator=(LeaseHandle&& other) noexcept;
  LeaseHandle(const LeaseHandle&) = delete;
  LeaseHandle& operator=(const LeaseHandle&) = delete;

  bool valid() const noexcept { return slot_meta_ != nullptr; }
  uint32_t slot_id() const noexcept { return slot_id_; }
  uint32_t generation() const noexcept { return generation_; }

 private:
  LeaseHandle() = default;
  LeaseHandle(std::shared_ptr<LeaseMapping> mapping, SlotMeta* slot,
              uint32_t slot_id, uint32_t generation);

  void release() noexcept;

  std::shared_ptr<LeaseMapping> mapping_;
  SlotMeta* slot_meta_ = nullptr;
  uint32_t slot_id_ = 0;
  uint32_t generation_ = 0;
};

}  // namespace ros2_cuda_ipc_core::lease
