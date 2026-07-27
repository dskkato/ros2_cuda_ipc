// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>
#include <memory>
#include <optional>

#include "ros2_cuda_ipc_core/lease/lease_mapping.hpp"

namespace ros2_cuda_ipc_core::lease {

/// LeaseHandle manages the lifetime of a shared-memory slot using reference
/// counting and generation checks.
class LeaseHandle {
 public:
  /// Holds the mapping needed to complete a Publisher reservation later.
  struct PublisherReservation {
    std::shared_ptr<LeaseMapping> mapping;
    uint32_t slot_id = 0;
    uint32_t generation = 0;
  };

  /// Read the current generation value for a slot.
  ///
  /// @param mapping Shared-memory mapping containing the slot metadata.
  /// @param slot_id Slot index inside the mapping.
  /// @return Generation number, or std::nullopt when the mapping is null or
  /// the slot is out of range.
  static std::optional<uint32_t> current_generation(
      const std::shared_ptr<LeaseMapping>& mapping, uint32_t slot_id);

  /// Read the current reference count for a slot. The count includes the
  /// temporary Publisher reservation while a publish attempt is in progress.
  ///
  /// @param mapping Shared-memory mapping containing the slot metadata.
  /// @param slot_id Slot index inside the mapping.
  /// @return Reference count, or std::nullopt when the mapping is null or the
  /// slot is out of range.
  static std::optional<uint32_t> current_refcount(
      const std::shared_ptr<LeaseMapping>& mapping, uint32_t slot_id);

  /// Read the current publication timestamp for a slot.
  static std::optional<uint64_t> current_publish_timestamp_us(
      const std::shared_ptr<LeaseMapping>& mapping, uint32_t slot_id);

  /// Atomically claim an idle slot and advance its generation.
  ///
  /// The returned reservation retains the mapping so it can be cancelled even
  /// after the Publisher's current mapping has been reset.
  ///
  /// @param mapping Shared-memory mapping on which to reserve a slot.
  /// @return Reservation when a slot is available; std::nullopt otherwise.
  static std::optional<PublisherReservation> reserve_for_publish(
      const std::shared_ptr<LeaseMapping>& mapping);

  /// Start the reuse grace period and release a Publisher reservation.
  ///
  /// @param mapping Shared-memory mapping containing the slot metadata.
  /// @param slot_id Slot index inside the mapping.
  static bool commit_publish(const std::shared_ptr<LeaseMapping>& mapping,
                             uint32_t slot_id, uint32_t generation) noexcept;

  /// Cancel a Publisher reservation and release its temporary reference.
  ///
  /// @param mapping Shared-memory mapping containing the slot metadata.
  /// @param slot_id Slot index inside the mapping.
  static bool cancel_publish(const std::shared_ptr<LeaseMapping>& mapping,
                             uint32_t slot_id, uint32_t generation) noexcept;

  /// Acquire a lease when the slot generation matches and increment its
  /// reference count.
  ///
  /// @param mapping Shared-memory mapping containing the slot metadata.
  /// @param slot_id Slot index that should be leased.
  /// @param generation Expected generation for the slot.
  /// @return Valid LeaseHandle when the slot is obtained; otherwise an invalid
  /// handle.
  static LeaseHandle acquire(const std::shared_ptr<LeaseMapping>& mapping,
                             uint32_t slot_id, uint32_t generation);

  /// Release any held lease on destruction.
  ~LeaseHandle();

  LeaseHandle(LeaseHandle&& other) noexcept;
  LeaseHandle& operator=(LeaseHandle&& other) noexcept;
  LeaseHandle(const LeaseHandle&) = delete;
  LeaseHandle& operator=(const LeaseHandle&) = delete;

  /// Check whether this handle currently owns a lease.
  bool valid() const noexcept { return slot_meta_ != nullptr; }

  /// Slot identifier associated with the lease (undefined when invalid()).
  uint32_t slot_id() const noexcept { return slot_id_; }

  /// Publisher generation that this lease corresponds to (undefined when
  /// invalid()).
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
