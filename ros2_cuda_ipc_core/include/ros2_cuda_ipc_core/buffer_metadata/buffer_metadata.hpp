// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

namespace ros2_cuda_ipc_core::buffer_metadata {

/// The complete process-shared state owned by one GPU block.
struct BlockMetadata {
  std::atomic<uint64_t> uid{0};
  std::atomic<uint32_t> refcount{0};
  std::atomic<uint64_t> publish_timestamp_us{0};
};

static_assert(std::atomic<uint32_t>::is_always_lock_free);
static_assert(std::atomic<uint64_t>::is_always_lock_free);
static_assert(sizeof(BlockMetadata) == 24);
static_assert(alignof(BlockMetadata) >= alignof(std::atomic<uint64_t>));

/// Owns one POSIX shared-memory mapping containing exactly one BlockMetadata.
class BufferMetadata {
 public:
  /// Create a block metadata object exclusively.
  static std::shared_ptr<BufferMetadata> create(const std::string& shm_name,
                                                uint64_t initial_uid);

  /// Open an existing block metadata object.
  static std::shared_ptr<BufferMetadata> attach(const std::string& shm_name);

  ~BufferMetadata();

  BufferMetadata(const BufferMetadata&) = delete;
  BufferMetadata& operator=(const BufferMetadata&) = delete;

  const std::string& shm_name() const noexcept { return shm_name_; }
  BlockMetadata* block() noexcept { return block_; }
  const BlockMetadata* block() const noexcept { return block_; }

 public:  // mapping construction is kept internal by the implementation
  BufferMetadata() = default;

  std::string shm_name_;
  std::size_t mapped_size_ = 0;
  void* addr_ = nullptr;
  BlockMetadata* block_ = nullptr;
};

/// Derive the only shared-memory name used by the block protocol.
std::string block_metadata_shm_name(uint32_t publisher_pid, uint32_t block_id);

}  // namespace ros2_cuda_ipc_core::buffer_metadata
