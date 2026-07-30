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

/// Internal subscriber cache for shared-memory buffer metadata mappings.
class BufferMetadataCache {
 public:
  using AttachFn =
      std::function<std::shared_ptr<buffer_metadata::BufferMetadata>(
          const std::string&, const PublisherInstanceId&)>;

  explicit BufferMetadataCache(
      AttachFn attach_fn = buffer_metadata::BufferMetadata::attach);
  ~BufferMetadataCache();

  std::shared_ptr<buffer_metadata::BufferMetadata> get_or_attach(
      const std::string& shm_name,
      const PublisherInstanceId& publisher_instance_id) const;

  void clear() const;
  std::size_t size() const;

 private:
  struct Key {
    std::string shm_name;
    PublisherInstanceId publisher_instance_id{};

    bool operator==(const Key& other) const noexcept {
      return shm_name == other.shm_name &&
             publisher_instance_id == other.publisher_instance_id;
    }
  };

  struct KeyHash {
    std::size_t operator()(const Key& key) const noexcept;
  };

  AttachFn attach_fn_;
  mutable std::mutex mutex_;
  mutable std::unordered_map<
      Key, std::shared_ptr<buffer_metadata::BufferMetadata>, KeyHash>
      mappings_;
};

}  // namespace ros2_cuda_ipc_core::subscriber::detail
