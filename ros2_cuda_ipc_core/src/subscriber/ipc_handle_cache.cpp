// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/subscriber/ipc_handle_cache.hpp"

#include <vector>

namespace ros2_cuda_ipc_core::subscriber {

std::size_t IpcHandleKeyHash::operator()(
    const IpcHandleKey& key) const noexcept {
  constexpr std::size_t PRIME{131};
  std::size_t hash = key.backend;
  for (uint8_t byte : key.publisher_instance_id) {
    hash = hash * PRIME + byte;
  }
  for (uint8_t byte : key.mem) {
    hash = hash * PRIME + byte;
  }
  for (uint8_t byte : key.event) {
    hash = hash * PRIME + byte;
  }
  return hash;
}

IpcHandleCache::IpcHandleCache(ReleaseFn release_fn)
    : release_fn_(std::move(release_fn)) {}

IpcHandleCache::~IpcHandleCache() noexcept {
  std::vector<backend::ImportedMemory> resources;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    resources.reserve(cache_.size());
    for (const auto& entry : cache_) {
      resources.push_back(entry.second);
    }
    cache_.clear();
  }
  for (const auto& resource : resources) {
    try {
      release_fn_(resource);
    } catch (...) {
      // Cleanup is best effort and must never throw during process teardown.
    }
  }
}

IpcHandleCache& IpcHandleCache::instance() {
  static IpcHandleCache cache;
  return cache;
}

std::optional<backend::ImportedMemory> IpcHandleCache::find(
    const IpcHandleKey& key) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = cache_.find(key);
  if (it == cache_.end()) {
    return std::nullopt;
  }
  return it->second;
}

backend::ImportedMemory IpcHandleCache::insert_or_discard_duplicate(
    const IpcHandleKey& key, backend::ImportedMemory imported) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto [it, inserted] = cache_.emplace(key, imported);
  if (!inserted) {
    release_fn_(imported);
    return it->second;
  }
  return imported;
}

std::size_t IpcHandleCache::size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return cache_.size();
}

}  // namespace ros2_cuda_ipc_core::subscriber
