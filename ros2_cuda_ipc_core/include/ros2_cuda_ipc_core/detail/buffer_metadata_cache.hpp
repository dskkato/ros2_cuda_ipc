// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "ros2_cuda_ipc_core/buffer_metadata/buffer_metadata.hpp"

namespace ros2_cuda_ipc_core::subscriber::detail {

class BufferMetadataCache {
 public:
  using AttachFn =
      std::function<std::shared_ptr<buffer_metadata::BufferMetadata>(
          const std::string&)>;

  explicit BufferMetadataCache(
      AttachFn attach_fn = buffer_metadata::BufferMetadata::attach);
  ~BufferMetadataCache();

  std::shared_ptr<buffer_metadata::BufferMetadata> get_or_attach(
      uint32_t publisher_pid, uint32_t block_id) const;
  void invalidate(uint32_t publisher_pid, uint32_t block_id) const;
  void clear() const;
  std::size_t size() const;

 private:
  struct Key {
    uint32_t publisher_pid;
    uint32_t block_id;
    bool operator==(const Key& other) const noexcept {
      return publisher_pid == other.publisher_pid && block_id == other.block_id;
    }
  };
  struct KeyHash {
    std::size_t operator()(const Key& key) const noexcept {
      return static_cast<std::size_t>(key.publisher_pid) * 31u + key.block_id;
    }
  };

  AttachFn attach_fn_;
  mutable std::mutex mutex_;
  mutable std::unordered_map<
      Key, std::shared_ptr<buffer_metadata::BufferMetadata>, KeyHash>
      mappings_;
};

}  // namespace ros2_cuda_ipc_core::subscriber::detail
