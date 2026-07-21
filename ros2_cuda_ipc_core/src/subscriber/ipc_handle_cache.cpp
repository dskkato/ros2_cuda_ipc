// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/subscriber/ipc_handle_cache.hpp"

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

IpcHandleCache::~IpcHandleCache() { clear(); }

IpcHandleCache& IpcHandleCache::instance() {
  static IpcHandleCache cache;
  return cache;
}

std::optional<IpcHandleCache::ImportedMemoryResource> IpcHandleCache::find(
    const IpcHandleKey& key) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = cache_.find(key);
  if (it == cache_.end()) {
    return std::nullopt;
  }
  return it->second;
}

IpcHandleCache::ImportedMemoryResource
IpcHandleCache::insert_or_discard_duplicate(const IpcHandleKey& key,
                                            backend::ImportedMemory imported) {
  ImportedMemoryResource existing;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = cache_.find(key);
    if (it != cache_.end()) {
      existing = it->second;
    } else {
      auto resource = ImportedMemoryResource(
          new backend::ImportedMemory(std::move(imported)),
          [release_fn = release_fn_](const backend::ImportedMemory* memory) {
            release_fn(*memory);
            delete memory;
          });
      cache_.emplace(key, resource);
      return resource;
    }
  }

  // A duplicate is never shared with a view, so it can be released promptly.
  // This is intentionally outside mutex_ because the callback may enter cache.
  release_fn_(imported);
  return existing;
}

void IpcHandleCache::clear() {
  decltype(cache_) entries;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    entries.swap(cache_);
  }
  // Resource deleters may invoke CUDA/Driver API calls, so destroy entries only
  // after releasing mutex_. Views can retain resources past this point.
}

std::size_t IpcHandleCache::size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return cache_.size();
}

}  // namespace ros2_cuda_ipc_core::subscriber
