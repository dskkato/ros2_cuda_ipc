// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

namespace ros2_cuda_ipc_core::buffer_metadata {

/// Return the POSIX name used for one publisher block's metadata object.
///
/// This is part of the metadata protocol and is shared by publishers and
/// subscribers; it does not imply ownership of the named object.
std::string shm_name_for_block(uint32_t publisher_pid, uint32_t block_id);

/// The complete contents of one per-GPU-block shared-memory object.
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
///
/// The object deliberately has no pool header, capacity, block array, or
/// publisher-instance identity. The block locator is encoded in the POSIX
/// name and carried by the transport descriptor.
class BufferMetadata {
 public:
  static std::shared_ptr<BufferMetadata> create(const std::string& shm_name);
  static std::shared_ptr<BufferMetadata> attach(const std::string& shm_name);

  ~BufferMetadata();

  BufferMetadata(const BufferMetadata&) = delete;
  BufferMetadata& operator=(const BufferMetadata&) = delete;

  const std::string& shm_name() const noexcept { return shm_name_; }
  BlockMetadata* metadata() noexcept { return metadata_; }
  const BlockMetadata* metadata() const noexcept { return metadata_; }

 private:
  BufferMetadata() = default;
  static std::shared_ptr<BufferMetadata> make_mapping(
      const std::string& shm_name, void* addr, std::size_t mapped_size);

  std::string shm_name_;
  std::size_t mapped_size_ = 0;
  void* addr_ = nullptr;
  BlockMetadata* metadata_ = nullptr;
};

}  // namespace ros2_cuda_ipc_core::buffer_metadata
