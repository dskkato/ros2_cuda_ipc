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
    const std::shared_ptr<BufferMetadata>& mapping, uint32_t block_id,
    uint32_t uid, bool published) noexcept {
  if (!mapping || block_id >= mapping->capacity()) return false;
  BlockMetadata& block = *mapping->block(block_id);
  if (block.uid.load(std::memory_order_acquire) != uid) {
    RCUTILS_LOG_ERROR_NAMED(
        "ros2_cuda_ipc_core.buffer_ref",
        "buffer_ref:publisher_reservation_uid_mismatch block=%u uid=%u",
        block_id, uid);
    return false;
  }
  if (published) {
    block.publish_timestamp_us.store(now_us(), std::memory_order_release);
  }
  const uint32_t previous =
      block.refcount.fetch_sub(1, std::memory_order_acq_rel);
  if (previous == 0) {
    RCUTILS_LOG_ERROR_NAMED("ros2_cuda_ipc_core.buffer_ref",
                            "buffer_ref:refcount_underflow block=%u", block_id);
    block.refcount.store(0, std::memory_order_release);
    return false;
  }
  return true;
}

}  // namespace

BufferRef::BufferRef(std::shared_ptr<BufferMetadata> mapping,
                     BlockMetadata* block, uint32_t block_id, uint32_t uid)
    : mapping_(std::move(mapping)),
      block_meta_(block),
      block_id_(block_id),
      uid_(uid) {}

BufferRef::BufferRef(BufferRef&& other) noexcept { *this = std::move(other); }

BufferRef& BufferRef::operator=(BufferRef&& other) noexcept {
  if (this == &other) return *this;
  release();
  mapping_ = std::move(other.mapping_);
  block_meta_ = other.block_meta_;
  block_id_ = other.block_id_;
  uid_ = other.uid_;
  other.block_meta_ = nullptr;
  other.block_id_ = 0;
  other.uid_ = 0;
  return *this;
}

BufferRef::~BufferRef() { release(); }

void BufferRef::release() noexcept {
  if (!block_meta_) return;
  const uint32_t previous =
      block_meta_->refcount.fetch_sub(1, std::memory_order_acq_rel);
  if (previous == 0) {
    RCUTILS_LOG_ERROR_NAMED("ros2_cuda_ipc_core.buffer_ref",
                            "buffer_ref:refcount_underflow block=%u",
                            block_id_);
    block_meta_->refcount.store(0, std::memory_order_release);
  }
  block_meta_ = nullptr;
  block_id_ = 0;
  uid_ = 0;
  mapping_.reset();
}

std::optional<uint32_t> BufferRef::current_uid(
    const std::shared_ptr<BufferMetadata>& mapping, uint32_t block_id) {
  if (!mapping || block_id >= mapping->capacity()) return std::nullopt;
  return mapping->block(block_id)->uid.load(std::memory_order_acquire);
}

std::optional<uint32_t> BufferRef::current_refcount(
    const std::shared_ptr<BufferMetadata>& mapping, uint32_t block_id) {
  if (!mapping || block_id >= mapping->capacity()) return std::nullopt;
  return mapping->block(block_id)->refcount.load(std::memory_order_acquire);
}

std::optional<uint64_t> BufferRef::current_publish_timestamp_us(
    const std::shared_ptr<BufferMetadata>& mapping, uint32_t block_id) {
  if (!mapping || block_id >= mapping->capacity()) return std::nullopt;
  return mapping->block(block_id)->publish_timestamp_us.load(
      std::memory_order_acquire);
}

std::optional<BufferRef::PublisherReservation> BufferRef::reserve_for_publish(
    const std::shared_ptr<BufferMetadata>& mapping) {
  if (!mapping || mapping->capacity() == 0) return std::nullopt;
  const uint32_t capacity = mapping->capacity();
  const uint32_t start =
      mapping->next_block().fetch_add(1, std::memory_order_relaxed) % capacity;
  const uint64_t now = now_us();
  for (uint32_t offset = 0; offset < capacity; ++offset) {
    const uint32_t block_id = (start + offset) % capacity;
    BlockMetadata& block = *mapping->block(block_id);
    if (block.refcount.load(std::memory_order_acquire) != 0) continue;
    const uint64_t published_at =
        block.publish_timestamp_us.load(std::memory_order_acquire);
    if (published_at != 0 &&
        (now < published_at || now - published_at < kGracePeriodUs))
      continue;
    uint32_t expected = 0;
    if (!block.refcount.compare_exchange_strong(
            expected, 1, std::memory_order_acq_rel, std::memory_order_acquire))
      continue;
    const uint32_t next = block.uid.load(std::memory_order_relaxed) + 1;
    block.uid.store(next, std::memory_order_release);
    block.publish_timestamp_us.store(0, std::memory_order_release);
    mapping->next_block().store((block_id + 1) % capacity,
                                std::memory_order_relaxed);
    return PublisherReservation{mapping, block_id, next};
  }
  return std::nullopt;
}

bool BufferRef::commit_publish(const std::shared_ptr<BufferMetadata>& mapping,
                               uint32_t block_id, uint32_t uid) noexcept {
  return release_publisher_reservation(mapping, block_id, uid, true);
}

bool BufferRef::cancel_publish(const std::shared_ptr<BufferMetadata>& mapping,
                               uint32_t block_id, uint32_t uid) noexcept {
  return release_publisher_reservation(mapping, block_id, uid, false);
}

BufferRef BufferRef::acquire(const std::shared_ptr<BufferMetadata>& mapping,
                             uint32_t block_id, uint32_t uid) {
  if (!mapping || block_id >= mapping->capacity()) return BufferRef{};
  BlockMetadata* block = mapping->block(block_id);
  if (block->uid.load(std::memory_order_acquire) != uid) return BufferRef{};

  uint32_t observed_ref = block->refcount.load(std::memory_order_acquire);
  while (true) {
    if (observed_ref == UINT32_MAX) {
      RCUTILS_LOG_ERROR_NAMED("ros2_cuda_ipc_core.buffer_ref",
                              "buffer_ref:refcount_overflow block=%u",
                              block_id);
      return BufferRef{};
    }
    if (block->refcount.compare_exchange_weak(observed_ref, observed_ref + 1,
                                              std::memory_order_acq_rel,
                                              std::memory_order_acquire))
      break;
  }
  const uint32_t recheck_uid = block->uid.load(std::memory_order_acquire);
  if (recheck_uid != uid) {
    block->refcount.fetch_sub(1, std::memory_order_acq_rel);
    return BufferRef{};
  }
  return BufferRef(mapping, block, block_id, uid);
}

}  // namespace ros2_cuda_ipc_core::buffer_metadata
