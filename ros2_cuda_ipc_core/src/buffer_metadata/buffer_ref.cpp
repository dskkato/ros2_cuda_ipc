// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/buffer_metadata/buffer_ref.hpp"

#include <rcutils/logging_macros.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <optional>

namespace ros2_cuda_ipc_core::buffer_metadata {
namespace {

constexpr uint64_t kGracePeriodUs = 100000;

uint64_t now_us() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

bool release_publisher_reservation(
    const std::shared_ptr<BufferMetadata>& mapping, uint32_t slot_id,
    uint32_t generation, bool published) noexcept {
  if (!mapping || slot_id >= mapping->capacity()) return false;
  SlotMetadata& slot = *mapping->slot(slot_id);
  if (slot.generation.load(std::memory_order_acquire) != generation) {
    RCUTILS_LOG_ERROR_NAMED(
        "ros2_cuda_ipc_core.buffer_ref",
        "buffer_ref:publisher_reservation_generation_mismatch slot=%u gen=%u",
        slot_id, generation);
    return false;
  }
  if (published) {
    slot.publish_timestamp_us.store(now_us(), std::memory_order_release);
  }
  const uint32_t previous =
      slot.refcount.fetch_sub(1, std::memory_order_acq_rel);
  if (previous == 0) {
    RCUTILS_LOG_ERROR_NAMED("ros2_cuda_ipc_core.buffer_ref",
                            "buffer_ref:refcount_underflow slot=%u", slot_id);
    slot.refcount.store(0, std::memory_order_release);
    return false;
  }
  return true;
}

}  // namespace

BufferRef::BufferRef(std::shared_ptr<BufferMetadata> mapping,
                     SlotMetadata* slot, uint32_t slot_id, uint32_t generation)
    : mapping_(std::move(mapping)),
      slot_meta_(slot),
      slot_id_(slot_id),
      generation_(generation) {}

BufferRef::BufferRef(BufferRef&& other) noexcept { *this = std::move(other); }

BufferRef& BufferRef::operator=(BufferRef&& other) noexcept {
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

BufferRef::~BufferRef() { release(); }

void BufferRef::release() noexcept {
  if (!slot_meta_) return;
  const uint32_t previous =
      slot_meta_->refcount.fetch_sub(1, std::memory_order_acq_rel);
  if (previous == 0) {
    RCUTILS_LOG_ERROR_NAMED("ros2_cuda_ipc_core.buffer_ref",
                            "buffer_ref:refcount_underflow slot=%u", slot_id_);
    slot_meta_->refcount.store(0, std::memory_order_release);
  }
  slot_meta_ = nullptr;
  slot_id_ = 0;
  generation_ = 0;
  mapping_.reset();
}

std::optional<uint32_t> BufferRef::current_generation(
    const std::shared_ptr<BufferMetadata>& mapping, uint32_t slot_id) {
  if (!mapping || slot_id >= mapping->capacity()) return std::nullopt;
  return mapping->slot(slot_id)->generation.load(std::memory_order_acquire);
}

std::optional<uint32_t> BufferRef::current_refcount(
    const std::shared_ptr<BufferMetadata>& mapping, uint32_t slot_id) {
  if (!mapping || slot_id >= mapping->capacity()) return std::nullopt;
  return mapping->slot(slot_id)->refcount.load(std::memory_order_acquire);
}

std::optional<uint64_t> BufferRef::current_publish_timestamp_us(
    const std::shared_ptr<BufferMetadata>& mapping, uint32_t slot_id) {
  if (!mapping || slot_id >= mapping->capacity()) return std::nullopt;
  return mapping->slot(slot_id)->publish_timestamp_us.load(
      std::memory_order_acquire);
}

std::optional<BufferRef::PublisherReservation> BufferRef::reserve_for_publish(
    const std::shared_ptr<BufferMetadata>& mapping) {
  if (!mapping || mapping->capacity() == 0) return std::nullopt;
  const uint32_t capacity = mapping->capacity();
  const uint32_t start =
      mapping->next_slot().fetch_add(1, std::memory_order_relaxed) % capacity;
  const uint64_t now = now_us();
  for (uint32_t offset = 0; offset < capacity; ++offset) {
    const uint32_t slot_id = (start + offset) % capacity;
    SlotMetadata& slot = *mapping->slot(slot_id);
    if (slot.refcount.load(std::memory_order_acquire) != 0) continue;
    const uint64_t published_at =
        slot.publish_timestamp_us.load(std::memory_order_acquire);
    if (published_at != 0 &&
        (now < published_at || now - published_at < kGracePeriodUs))
      continue;
    uint32_t expected = 0;
    if (!slot.refcount.compare_exchange_strong(
            expected, 1, std::memory_order_acq_rel, std::memory_order_acquire))
      continue;
    const uint32_t next = slot.generation.load(std::memory_order_relaxed) + 1;
    slot.generation.store(next, std::memory_order_release);
    slot.publish_timestamp_us.store(0, std::memory_order_release);
    mapping->next_slot().store((slot_id + 1) % capacity,
                               std::memory_order_relaxed);
    return PublisherReservation{mapping, slot_id, next};
  }
  return std::nullopt;
}

bool BufferRef::commit_publish(const std::shared_ptr<BufferMetadata>& mapping,
                               uint32_t slot_id, uint32_t generation) noexcept {
  return release_publisher_reservation(mapping, slot_id, generation, true);
}

bool BufferRef::cancel_publish(const std::shared_ptr<BufferMetadata>& mapping,
                               uint32_t slot_id, uint32_t generation) noexcept {
  return release_publisher_reservation(mapping, slot_id, generation, false);
}

BufferRef BufferRef::acquire(const std::shared_ptr<BufferMetadata>& mapping,
                             uint32_t slot_id, uint32_t generation) {
  if (!mapping || slot_id >= mapping->capacity()) return BufferRef{};
  SlotMetadata* slot = mapping->slot(slot_id);
  if (slot->generation.load(std::memory_order_acquire) != generation)
    return BufferRef{};

  uint32_t observed_ref = slot->refcount.load(std::memory_order_acquire);
  while (true) {
    if (observed_ref == UINT32_MAX) {
      RCUTILS_LOG_ERROR_NAMED("ros2_cuda_ipc_core.buffer_ref",
                              "buffer_ref:refcount_overflow slot=%u", slot_id);
      return BufferRef{};
    }
    if (slot->refcount.compare_exchange_weak(observed_ref, observed_ref + 1,
                                             std::memory_order_acq_rel,
                                             std::memory_order_acquire))
      break;
  }
  const uint32_t recheck_gen = slot->generation.load(std::memory_order_acquire);
  if (recheck_gen != generation) {
    slot->refcount.fetch_sub(1, std::memory_order_acq_rel);
    return BufferRef{};
  }
  return BufferRef(mapping, slot, slot_id, generation);
}

}  // namespace ros2_cuda_ipc_core::buffer_metadata
