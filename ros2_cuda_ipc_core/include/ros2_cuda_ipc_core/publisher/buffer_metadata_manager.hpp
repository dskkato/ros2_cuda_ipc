// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "ros2_cuda_ipc_core/buffer_metadata/buffer_metadata.hpp"

namespace ros2_cuda_ipc_core::publisher {

/// Owns one shared BlockMetadata mapping for every local GPU block.
class BufferMetadataManager {
 public:
  struct Reservation {
    std::shared_ptr<buffer_metadata::BufferMetadata> mapping;
    uint32_t block_id = 0;
    uint64_t uid = 0;
    uint32_t publisher_pid = 0;
    std::string shm_name;  // Publisher-local diagnostic/cleanup handle.
    std::size_t pool_index = 0;
  };

  explicit BufferMetadataManager(std::size_t block_count);
  ~BufferMetadataManager();

  bool initialise();
  void reset() noexcept;
  bool is_initialised() const noexcept;
  std::optional<Reservation> reserve_for_publish();
  bool commit(const Reservation& reservation) noexcept;
  bool cancel(const Reservation& reservation) noexcept;

  uint32_t publisher_pid() const noexcept { return publisher_pid_; }
  std::shared_ptr<buffer_metadata::BufferMetadata> metadata_for_pool_index(
      std::size_t pool_index) const;
  std::optional<uint32_t> block_id_for_pool_index(std::size_t pool_index) const;
  static std::string shm_name_for_block(uint32_t publisher_pid,
                                        uint32_t block_id);

 private:
  struct Entry {
    uint32_t block_id = 0;
    std::string shm_name;
    std::shared_ptr<buffer_metadata::BufferMetadata> mapping;
  };

  std::size_t block_count_;
  uint32_t publisher_pid_ = 0;
  mutable std::mutex mutex_;
  std::vector<Entry> entries_;
  std::size_t next_entry_ = 0;
  bool initialised_ = false;
};

}  // namespace ros2_cuda_ipc_core::publisher
