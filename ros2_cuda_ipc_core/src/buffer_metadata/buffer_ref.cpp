// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/buffer_metadata/buffer_ref.hpp"

#include <rcutils/logging_macros.h>

#include <atomic>
#include <chrono>
#include <limits>

namespace ros2_cuda_ipc_core::buffer_metadata {
namespace {

constexpr uint64_t kGracePeriodUs = 100000;
// The high bit temporarily marks a publisher reservation. Subscribers must
// not acquire while a block is being overwritten and its UID is changing.
constexpr uint32_t kPublisherReservationBit = 1u << 31;

uint64_t now_us() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

bool release_reservation(const std::shared_ptr<BufferMetadata>& mapping,
                         uint64_t uid, bool published) noexcept {
  if (!mapping || mapping->block() == nullptr) return false;
  BlockMetadata& block = *mapping->block();
  if (block.uid.load(std::memory_order_acquire) != uid) {
    RCUTILS_LOG_ERROR_NAMED("ros2_cuda_ipc_core.buffer_ref",
                            "reservation uid mismatch uid=%llu",
                            static_cast<unsigned long long>(uid));
    return false;
  }
  if (published) {
    block.publish_timestamp_us.store(now_us(), std::memory_order_release);
  }
  uint32_t expected = kPublisherReservationBit;
  return block.refcount.compare_exchange_strong(
      expected, 0, std::memory_order_acq_rel, std::memory_order_acquire);
}

}  // namespace

BufferRef::BufferRef(std::shared_ptr<BufferMetadata> mapping,
                     BlockMetadata* block, uint64_t uid)
    : mapping_(std::move(mapping)), block_(block), uid_(uid) {}

BufferRef::BufferRef(BufferRef&& other) noexcept { *this = std::move(other); }

BufferRef& BufferRef::operator=(BufferRef&& other) noexcept {
  if (this == &other) return *this;
  release();
  mapping_ = std::move(other.mapping_);
  block_ = other.block_;
  uid_ = other.uid_;
  other.block_ = nullptr;
  other.uid_ = 0;
  return *this;
}

BufferRef::~BufferRef() { release(); }

void BufferRef::release() noexcept {
  if (block_ != nullptr) {
    const uint32_t previous =
        block_->refcount.fetch_sub(1, std::memory_order_acq_rel);
    if (previous == 0) block_->refcount.store(0, std::memory_order_release);
  }
  block_ = nullptr;
  uid_ = 0;
  mapping_.reset();
}

std::optional<uint64_t> BufferRef::current_uid(
    const std::shared_ptr<BufferMetadata>& mapping) {
  if (!mapping || !mapping->block()) return std::nullopt;
  return mapping->block()->uid.load(std::memory_order_acquire);
}

std::optional<uint32_t> BufferRef::current_refcount(
    const std::shared_ptr<BufferMetadata>& mapping) {
  if (!mapping || !mapping->block()) return std::nullopt;
  return mapping->block()->refcount.load(std::memory_order_acquire) &
         ~kPublisherReservationBit;
}

std::optional<uint64_t> BufferRef::current_publish_timestamp_us(
    const std::shared_ptr<BufferMetadata>& mapping) {
  if (!mapping || !mapping->block()) return std::nullopt;
  return mapping->block()->publish_timestamp_us.load(std::memory_order_acquire);
}

std::optional<BufferRef::PublisherReservation> BufferRef::reserve_for_publish(
    const std::shared_ptr<BufferMetadata>& mapping) {
  if (!mapping || !mapping->block()) return std::nullopt;
  BlockMetadata& block = *mapping->block();
  const uint64_t now = now_us();
  if (block.refcount.load(std::memory_order_acquire) != 0) return std::nullopt;
  const uint64_t published_at =
      block.publish_timestamp_us.load(std::memory_order_acquire);
  if (published_at != 0 &&
      (now < published_at || now - published_at < kGracePeriodUs))
    return std::nullopt;
  uint32_t expected = 0;
  if (!block.refcount.compare_exchange_strong(
          expected, kPublisherReservationBit, std::memory_order_acq_rel,
          std::memory_order_acquire))
    return std::nullopt;
  uint64_t next = block.uid.load(std::memory_order_relaxed);
  if (next == std::numeric_limits<uint64_t>::max())
    next = 1;
  else
    ++next;
  block.uid.store(next, std::memory_order_release);
  block.publish_timestamp_us.store(0, std::memory_order_release);
  return PublisherReservation{mapping, next};
}

bool BufferRef::commit_publish(const std::shared_ptr<BufferMetadata>& mapping,
                               uint64_t uid) noexcept {
  return release_reservation(mapping, uid, true);
}

bool BufferRef::cancel_publish(const std::shared_ptr<BufferMetadata>& mapping,
                               uint64_t uid) noexcept {
  return release_reservation(mapping, uid, false);
}

BufferRef BufferRef::acquire(const std::shared_ptr<BufferMetadata>& mapping,
                             uint64_t uid) {
  if (!mapping || !mapping->block()) return BufferRef{};
  BlockMetadata* block = mapping->block();
  if (block->uid.load(std::memory_order_acquire) != uid) return BufferRef{};
  uint32_t observed = block->refcount.load(std::memory_order_acquire);
  while (true) {
    if ((observed & kPublisherReservationBit) != 0 ||
        (observed & ~kPublisherReservationBit) == UINT32_MAX)
      return BufferRef{};
    if (block->refcount.compare_exchange_weak(observed, observed + 1,
                                              std::memory_order_acq_rel,
                                              std::memory_order_acquire))
      break;
  }
  if (block->uid.load(std::memory_order_acquire) != uid) {
    block->refcount.fetch_sub(1, std::memory_order_acq_rel);
    return BufferRef{};
  }
  return BufferRef(mapping, block, uid);
}

}  // namespace ros2_cuda_ipc_core::buffer_metadata
