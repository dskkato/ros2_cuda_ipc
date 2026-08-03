// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/detail/buffer_metadata_cache.hpp"

#include <utility>

namespace ros2_cuda_ipc_core::subscriber::detail {

BufferMetadataCache::BufferMetadataCache(AttachFn attach_fn)
    : attach_fn_(std::move(attach_fn)) {}

BufferMetadataCache::~BufferMetadataCache() { clear(); }

std::shared_ptr<buffer_metadata::BufferMetadata>
BufferMetadataCache::get_or_attach(uint32_t publisher_pid,
                                   uint32_t block_id) const {
  if (publisher_pid == 0 || block_id == 0) return nullptr;
  const Key key{publisher_pid, block_id};
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = mappings_.find(key);
    if (it != mappings_.end()) return it->second;
  }
  auto candidate = attach_fn_(
      buffer_metadata::block_metadata_shm_name(publisher_pid, block_id));
  if (!candidate) return nullptr;
  std::lock_guard<std::mutex> lock(mutex_);
  const auto [it, inserted] = mappings_.emplace(key, std::move(candidate));
  (void)inserted;
  return it->second;
}

void BufferMetadataCache::invalidate(uint32_t publisher_pid,
                                     uint32_t block_id) const {
  const Key key{publisher_pid, block_id};
  std::shared_ptr<buffer_metadata::BufferMetadata> removed;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = mappings_.find(key);
    if (it != mappings_.end()) {
      removed = std::move(it->second);
      mappings_.erase(it);
    }
  }
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
