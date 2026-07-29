// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "ros2_cuda_ipc_core/publisher_instance_id.hpp"

namespace ros2_cuda_ipc_core::lease {

/// Metadata for one slot in the shared-memory lease pool.
struct SlotMeta {
  std::atomic<uint32_t> generation{0};
  std::atomic<uint32_t> refcnt{0};
  std::atomic<uint64_t> publish_timestamp_us{0};
};

static_assert(std::atomic<uint32_t>::is_always_lock_free);
static_assert(std::atomic<uint64_t>::is_always_lock_free);
static_assert(sizeof(SlotMeta) == 16);
static_assert(alignof(SlotMeta) >= alignof(std::atomic<uint64_t>));

/// Owns one POSIX shared-memory mapping.
///
/// The object deliberately has no process-global cache. Callers that need to
/// share a mapping must retain the returned shared_ptr. The subscriber's
/// internal mapping cache is one such owner and retains mappings between
/// messages.
class LeaseMapping {
 public:
  /// Create and map a new shared-memory lease pool exclusively.
  ///
  /// @param shm_name POSIX shared-memory name.
  /// @param instance_id Publisher instance identity stored in the header.
  /// @param capacity Number of slots to allocate.
  /// @return Owning mapping, or nullptr when creation or initialization fails.
  static std::shared_ptr<LeaseMapping> create(
      const std::string& shm_name, const PublisherInstanceId& instance_id,
      uint32_t capacity);

  /// Open and map an existing shared-memory lease pool.
  ///
  /// The header's publisher instance identity must match the expected value.
  ///
  /// @param shm_name POSIX shared-memory name.
  /// @param expected_instance_id Publisher identity expected in the header.
  /// @return Owning mapping, or nullptr when opening or validation fails.
  static std::shared_ptr<LeaseMapping> attach(
      const std::string& shm_name,
      const PublisherInstanceId& expected_instance_id);

  ~LeaseMapping();

  LeaseMapping(const LeaseMapping&) = delete;
  LeaseMapping& operator=(const LeaseMapping&) = delete;

  const std::string& shm_name() const noexcept { return shm_name_; }
  const PublisherInstanceId& publisher_instance_id() const noexcept {
    return publisher_instance_id_;
  }
  uint32_t capacity() const noexcept { return capacity_; }
  SlotMeta* slot(uint32_t slot_id) noexcept {
    return slot_id < capacity_ ? &slots_[slot_id] : nullptr;
  }
  std::atomic<uint32_t>& next_slot() noexcept { return next_slot_; }

 private:
  LeaseMapping() = default;

  std::string shm_name_;
  PublisherInstanceId publisher_instance_id_{};
  uint32_t capacity_ = 0;
  std::size_t mapped_size_ = 0;
  void* addr_ = nullptr;
  SlotMeta* slots_ = nullptr;
  std::atomic<uint32_t> next_slot_{0};
};

}  // namespace ros2_cuda_ipc_core::lease
