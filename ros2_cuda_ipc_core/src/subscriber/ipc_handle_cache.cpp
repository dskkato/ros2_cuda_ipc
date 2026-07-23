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

IpcHandleCache::Entry IpcHandleCache::find(const IpcHandleKey& key) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = cache_.find(key);
  if (it == cache_.end()) {
    return {};
  }
  return it->second;
}

IpcHandleCache::Entry IpcHandleCache::insert_or_discard_duplicate(
    const IpcHandleKey& key, backend::ImportedMemory imported) {
  Entry candidate;
  try {
    ReleaseFn release = release_fn_;
    auto deleter = [release = std::move(release)](
                       const backend::ImportedMemory* resource) noexcept {
      try {
        release(*resource);
      } catch (...) {
        // shared_ptr deleters must not throw.
      }
      delete resource;
    };
    auto* resource = new backend::ImportedMemory(std::move(imported));
    candidate = Entry(resource, std::move(deleter));
  } catch (...) {
    // Ownership was not transferred when resource construction failed.
    // After a successful move, imported is empty and this is a no-op.
    release_fn_(imported);
    throw;
  }

  Entry result;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto [it, inserted] = cache_.emplace(key, candidate);
    result = it->second;
  }
  // A duplicate candidate is released here, after dropping the cache lock.
  return result;
}

void IpcHandleCache::clear() {
  decltype(cache_) entries;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    entries.swap(cache_);
  }
  // Dropping cache ownership does not invalidate entries still held by views.
  entries.clear();
}

std::size_t IpcHandleCache::size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return cache_.size();
}

}  // namespace ros2_cuda_ipc_core::subscriber
