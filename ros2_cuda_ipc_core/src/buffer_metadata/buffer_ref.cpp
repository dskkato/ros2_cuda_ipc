// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/buffer_metadata/buffer_ref.hpp"

#include <rcutils/logging_macros.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <limits>

namespace ros2_cuda_ipc_core::buffer_metadata {
namespace {

constexpr uint64_t kGracePeriodUs = 100000;

uint64_t now_us() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

BlockMetadata* block(const std::shared_ptr<BufferMetadata>& mapping) {
  return mapping ? mapping->metadata() : nullptr;
}

bool release_publisher_reservation(
    const std::shared_ptr<BufferMetadata>& mapping, uint64_t uid,
    bool published) noexcept {
  auto* metadata = block(mapping);
  if (metadata == nullptr ||
      metadata->uid.load(std::memory_order_acquire) != uid) {
    RCUTILS_LOG_ERROR_NAMED(
        "ros2_cuda_ipc_core.buffer_ref",
        "buffer_ref:publisher_reservation_uid_mismatch uid=%llu",
        static_cast<unsigned long long>(uid));
    return false;
  }
  if (published) {
    metadata->publish_timestamp_us.store(now_us(), std::memory_order_release);
  }
  const uint32_t previous =
      metadata->refcount.fetch_sub(1, std::memory_order_acq_rel);
  if (previous == 0) {
    RCUTILS_LOG_ERROR_NAMED("ros2_cuda_ipc_core.buffer_ref",
                            "buffer_ref:refcount_underflow");
    metadata->refcount.store(0, std::memory_order_release);
    return false;
  }
  return true;
}

}  // namespace

BufferRef::BufferRef(std::shared_ptr<BufferMetadata> mapping,
                     BlockMetadata* block, uint64_t uid)
    : mapping_(std::move(mapping)), block_meta_(block), uid_(uid) {}

BufferRef::BufferRef(BufferRef&& other) noexcept { *this = std::move(other); }

BufferRef& BufferRef::operator=(BufferRef&& other) noexcept {
  if (this == &other) return *this;
  release();
  mapping_ = std::move(other.mapping_);
  block_meta_ = other.block_meta_;
  uid_ = other.uid_;
  other.block_meta_ = nullptr;
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
                            "buffer_ref:refcount_underflow");
    block_meta_->refcount.store(0, std::memory_order_release);
  }
  block_meta_ = nullptr;
  uid_ = 0;
  mapping_.reset();
}

std::optional<uint64_t> BufferRef::current_uid(
    const std::shared_ptr<BufferMetadata>& mapping) {
  const auto* metadata = block(mapping);
  if (metadata == nullptr) return std::nullopt;
  return metadata->uid.load(std::memory_order_acquire);
}

std::optional<uint32_t> BufferRef::current_refcount(
    const std::shared_ptr<BufferMetadata>& mapping) {
  const auto* metadata = block(mapping);
  if (metadata == nullptr) return std::nullopt;
  return metadata->refcount.load(std::memory_order_acquire);
}

std::optional<uint64_t> BufferRef::current_publish_timestamp_us(
    const std::shared_ptr<BufferMetadata>& mapping) {
  const auto* metadata = block(mapping);
  if (metadata == nullptr) return std::nullopt;
  return metadata->publish_timestamp_us.load(std::memory_order_acquire);
}

std::optional<BufferRef::PublisherReservation> BufferRef::reserve_for_publish(
    const std::shared_ptr<BufferMetadata>& mapping) {
  auto* metadata = block(mapping);
  if (metadata == nullptr) return std::nullopt;
  const uint64_t now = now_us();
  if (metadata->refcount.load(std::memory_order_acquire) != 0) {
    return std::nullopt;
  }
  const uint64_t published_at =
      metadata->publish_timestamp_us.load(std::memory_order_acquire);
  if (published_at != 0 &&
      (now < published_at || now - published_at < kGracePeriodUs)) {
    return std::nullopt;
  }
  uint32_t expected = 0;
  if (!metadata->refcount.compare_exchange_strong(
          expected, 1, std::memory_order_acq_rel, std::memory_order_acquire)) {
    return std::nullopt;
  }
  const uint64_t current_uid = metadata->uid.load(std::memory_order_relaxed);
  if (current_uid == std::numeric_limits<uint64_t>::max()) {
    metadata->refcount.store(0, std::memory_order_release);
    RCUTILS_LOG_ERROR_NAMED("ros2_cuda_ipc_core.buffer_ref",
                            "buffer_ref:uid_overflow");
    return std::nullopt;
  }
  const uint64_t next_uid = current_uid + 1;
  metadata->uid.store(next_uid, std::memory_order_release);
  metadata->publish_timestamp_us.store(0, std::memory_order_release);
  return PublisherReservation{mapping, next_uid};
}

bool BufferRef::commit_publish(const std::shared_ptr<BufferMetadata>& mapping,
                               uint64_t uid) noexcept {
  return release_publisher_reservation(mapping, uid, true);
}

bool BufferRef::cancel_publish(const std::shared_ptr<BufferMetadata>& mapping,
                               uint64_t uid) noexcept {
  return release_publisher_reservation(mapping, uid, false);
}

BufferRef BufferRef::acquire(const std::shared_ptr<BufferMetadata>& mapping,
                             uint64_t uid) {
  auto* metadata = block(mapping);
  if (metadata == nullptr ||
      metadata->uid.load(std::memory_order_acquire) != uid) {
    return BufferRef{};
  }

  uint32_t observed_ref = metadata->refcount.load(std::memory_order_acquire);
  while (true) {
    if (observed_ref == std::numeric_limits<uint32_t>::max()) {
      RCUTILS_LOG_ERROR_NAMED("ros2_cuda_ipc_core.buffer_ref",
                              "buffer_ref:refcount_overflow");
      return BufferRef{};
    }
    if (metadata->refcount.compare_exchange_weak(observed_ref, observed_ref + 1,
                                                 std::memory_order_acq_rel,
                                                 std::memory_order_acquire))
      break;
  }
  if (metadata->uid.load(std::memory_order_acquire) != uid) {
    metadata->refcount.fetch_sub(1, std::memory_order_acq_rel);
    return BufferRef{};
  }
  return BufferRef(mapping, metadata, uid);
}

}  // namespace ros2_cuda_ipc_core::buffer_metadata
