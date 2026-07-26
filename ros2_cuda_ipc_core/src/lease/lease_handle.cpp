// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/lease/lease_handle.hpp"

#include <rcutils/logging_macros.h>

#include <atomic>
#include <cstdint>
#include <optional>
#include <thread>

namespace ros2_cuda_ipc_core::lease {
namespace {

constexpr uint32_t kCancelReservationAttempts = 1024;

inline std::atomic<uint32_t>& as_atomic(uint32_t& value) {
  return reinterpret_cast<std::atomic<uint32_t>&>(value);
}

}  // namespace

LeaseHandle::LeaseHandle(std::shared_ptr<LeaseMapping> mapping, SlotMeta* slot,
                         uint32_t slot_id, uint32_t generation)
    : mapping_(std::move(mapping)),
      slot_meta_(slot),
      slot_id_(slot_id),
      generation_(generation) {}

LeaseHandle::LeaseHandle(LeaseHandle&& other) noexcept {
  *this = std::move(other);
}

LeaseHandle& LeaseHandle::operator=(LeaseHandle&& other) noexcept {
  if (this == &other) return *this;
  release();
  mapping_ = std::move(other.mapping_);
  slot_meta_ = other.slot_meta_;
  slot_id_ = other.slot_id_;
  generation_ = other.generation_;
  other.slot_meta_ = nullptr;
  other.slot_id_ = 0;
  other.generation_ = 0;
  return *this;
}

LeaseHandle::~LeaseHandle() { release(); }

void LeaseHandle::release() noexcept {
  if (!slot_meta_) return;
  auto& ref = as_atomic(slot_meta_->refcnt);
  const uint32_t previous = ref.fetch_sub(1, std::memory_order_acq_rel);
  if (previous == 0) {
    RCUTILS_LOG_ERROR_NAMED("ros2_cuda_ipc_core.lease_handle",
                            "lease:refcnt_underflow slot=%u", slot_id_);
    ref.store(0, std::memory_order_release);
  }
  slot_meta_ = nullptr;
  slot_id_ = 0;
  generation_ = 0;
  mapping_.reset();
}

std::optional<uint32_t> LeaseHandle::current_generation(
    const std::shared_ptr<LeaseMapping>& mapping, uint32_t slot_id) {
  if (!mapping || slot_id >= mapping->capacity()) return std::nullopt;
  return as_atomic(mapping->slot(slot_id)->generation)
      .load(std::memory_order_acquire);
}

std::optional<uint32_t> LeaseHandle::current_refcount(
    const std::shared_ptr<LeaseMapping>& mapping, uint32_t slot_id) {
  if (!mapping || slot_id >= mapping->capacity()) return std::nullopt;
  return as_atomic(mapping->slot(slot_id)->refcnt)
      .load(std::memory_order_acquire);
}

std::optional<uint32_t> LeaseHandle::current_pending(
    const std::shared_ptr<LeaseMapping>& mapping, uint32_t slot_id) {
  if (!mapping || slot_id >= mapping->capacity()) return std::nullopt;
  return as_atomic(mapping->slot(slot_id)->pending)
      .load(std::memory_order_acquire);
}

std::optional<LeaseHandle::PublisherReservation>
LeaseHandle::reserve_for_publish(const std::shared_ptr<LeaseMapping>& mapping,
                                 uint32_t pending_count) {
  if (!mapping || mapping->capacity() == 0) return std::nullopt;
  const uint32_t capacity = mapping->capacity();
  const uint32_t start =
      mapping->next_slot().fetch_add(1, std::memory_order_relaxed) % capacity;
  for (uint32_t offset = 0; offset < capacity; ++offset) {
    const uint32_t slot_id = (start + offset) % capacity;
    SlotMeta& slot = *mapping->slot(slot_id);
    auto& ref = as_atomic(slot.refcnt);
    auto& pending = as_atomic(slot.pending);
    if (ref.load(std::memory_order_acquire) != 0 ||
        pending.load(std::memory_order_acquire) != 0)
      continue;
    auto& reserved = as_atomic(slot.reserved);
    uint32_t expected = 0;
    if (!reserved.compare_exchange_strong(
            expected, 1, std::memory_order_acq_rel, std::memory_order_acquire))
      continue;
    if (ref.load(std::memory_order_acquire) != 0 ||
        pending.load(std::memory_order_acquire) != 0) {
      reserved.store(0, std::memory_order_release);
      continue;
    }
    auto& generation = as_atomic(slot.generation);
    const uint32_t next = generation.load(std::memory_order_relaxed) + 1;
    generation.store(next, std::memory_order_release);
    pending.store(pending_count, std::memory_order_release);
    reserved.store(0, std::memory_order_release);
    mapping->next_slot().store((slot_id + 1) % capacity,
                               std::memory_order_relaxed);
    return PublisherReservation{mapping, slot_id, next};
  }
  return std::nullopt;
}

bool LeaseHandle::force_clear_pending(
    const std::shared_ptr<LeaseMapping>& mapping, uint32_t slot_id) {
  if (!mapping || slot_id >= mapping->capacity()) return false;
  SlotMeta* slot = mapping->slot(slot_id);
  auto& reserved = as_atomic(slot->reserved);
  uint32_t expected = 0;
  if (!reserved.compare_exchange_strong(expected, 1, std::memory_order_acq_rel,
                                        std::memory_order_acquire))
    return false;
  auto& pending = as_atomic(slot->pending);
  if (pending.load(std::memory_order_acquire) == 0) {
    reserved.store(0, std::memory_order_release);
    return true;
  }
  if (as_atomic(slot->refcnt).load(std::memory_order_acquire) != 0) {
    reserved.store(0, std::memory_order_release);
    return false;
  }
  pending.store(0, std::memory_order_release);
  reserved.store(0, std::memory_order_release);
  return true;
}

bool LeaseHandle::cancel_pending(const std::shared_ptr<LeaseMapping>& mapping,
                                 uint32_t slot_id, uint32_t generation) {
  if (!mapping || slot_id >= mapping->capacity()) return false;
  SlotMeta& slot = *mapping->slot(slot_id);
  auto& reserved = as_atomic(slot.reserved);
  bool acquired = false;
  for (uint32_t attempt = 0; attempt < kCancelReservationAttempts; ++attempt) {
    uint32_t expected = 0;
    if (reserved.compare_exchange_strong(expected, 1, std::memory_order_acq_rel,
                                         std::memory_order_acquire)) {
      acquired = true;
      break;
    }
    std::this_thread::yield();
  }
  if (!acquired) {
    RCUTILS_LOG_ERROR_NAMED(
        "ros2_cuda_ipc_core.lease_handle",
        "lease:cancel_reservation_contention slot=%u gen=%u", slot_id,
        generation);
    return false;
  }
  if (as_atomic(slot.generation).load(std::memory_order_acquire) !=
          generation ||
      as_atomic(slot.refcnt).load(std::memory_order_acquire) != 0) {
    reserved.store(0, std::memory_order_release);
    return false;
  }
  as_atomic(slot.pending).store(0, std::memory_order_release);
  reserved.store(0, std::memory_order_release);
  return true;
}

LeaseHandle LeaseHandle::acquire(const std::shared_ptr<LeaseMapping>& mapping,
                                 uint32_t slot_id, uint32_t generation) {
  if (!mapping || slot_id >= mapping->capacity()) return LeaseHandle{};
  SlotMeta* slot = mapping->slot(slot_id);
  auto& gen = as_atomic(slot->generation);
  auto& ref = as_atomic(slot->refcnt);
  auto& pending = as_atomic(slot->pending);
  auto& reserved = as_atomic(slot->reserved);
  if (reserved.load(std::memory_order_acquire) != 0 ||
      gen.load(std::memory_order_acquire) != generation)
    return LeaseHandle{};

  uint32_t observed_ref = ref.load(std::memory_order_acquire);
  while (true) {
    if (observed_ref == UINT32_MAX) {
      RCUTILS_LOG_ERROR_NAMED("ros2_cuda_ipc_core.lease_handle",
                              "lease:ref_overflow slot=%u", slot_id);
      return LeaseHandle{};
    }
    if (ref.compare_exchange_weak(observed_ref, observed_ref + 1,
                                  std::memory_order_acq_rel,
                                  std::memory_order_acquire))
      break;
  }
  const uint32_t recheck_gen = gen.load(std::memory_order_acquire);
  if (reserved.load(std::memory_order_acquire) != 0 ||
      recheck_gen != generation) {
    ref.fetch_sub(1, std::memory_order_acq_rel);
    return LeaseHandle{};
  }
  uint32_t observed_pending = pending.load(std::memory_order_acquire);
  while (observed_pending != 0) {
    if (pending.compare_exchange_weak(observed_pending, observed_pending - 1,
                                      std::memory_order_acq_rel,
                                      std::memory_order_acquire))
      break;
  }
  return LeaseHandle(mapping, slot, slot_id, generation);
}

}  // namespace ros2_cuda_ipc_core::lease
