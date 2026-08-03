// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/detail/buffer_metadata_cache.hpp"

#include <utility>

#include "ros2_cuda_ipc_core/buffer_metadata/buffer_ref.hpp"

namespace ros2_cuda_ipc_core::subscriber::detail {

BufferMetadataCache::BufferMetadataCache(AttachFn attach_fn)
    : attach_fn_(std::move(attach_fn)) {}

BufferMetadataCache::~BufferMetadataCache() { clear(); }

std::shared_ptr<buffer_metadata::BufferMetadata>
BufferMetadataCache::get_or_attach(uint32_t publisher_pid, uint32_t block_id,
                                   uint64_t expected_uid) const {
  const Key key{publisher_pid, block_id};
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = mappings_.find(key);
    if (it != mappings_.end()) {
      const auto current_uid =
          buffer_metadata::BufferRef::current_uid(it->second);
      if (current_uid && *current_uid == expected_uid) {
        return it->second;
      }
      // The descriptor may belong to a new object after publisher restart, or
      // it may simply be an old publication of this block. In both cases the
      // cached mapping must not be trusted for the next attach attempt.
      mappings_.erase(it);
    }
  }

  auto candidate = attach_fn_(publisher_pid, block_id);
  if (!candidate) return nullptr;
  const auto candidate_uid = buffer_metadata::BufferRef::current_uid(candidate);
  if (!candidate_uid || *candidate_uid != expected_uid) return nullptr;

  std::shared_ptr<buffer_metadata::BufferMetadata> result;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = mappings_.find(key);
    if (it != mappings_.end()) {
      const auto current_uid =
          buffer_metadata::BufferRef::current_uid(it->second);
      if (current_uid && *current_uid == expected_uid) return it->second;
      mappings_.erase(it);
    }
    mappings_.emplace(key, candidate);
    result = std::move(candidate);
  }
  return result;
}

void BufferMetadataCache::clear() const {
  decltype(mappings_) entries;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    entries.swap(mappings_);
  }
  entries.clear();
}

std::size_t BufferMetadataCache::size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return mappings_.size();
}

}  // namespace ros2_cuda_ipc_core::subscriber::detail
