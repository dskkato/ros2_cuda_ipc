// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/subscriber/ipc_handle_cache.hpp"

#include <memory>

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
  auto resource = it->second.lock();
  if (!resource) {
    cache_.erase(it);
  }
  return resource;
}

IpcHandleCache::Entry IpcHandleCache::insert_or_discard_duplicate(
    const IpcHandleKey& key, backend::ImportedResources imported) {
  Entry candidate;
  bool resource_constructed = false;
  try {
    ReleaseFn release = release_fn_;
    auto deleter = [release = std::move(release)](
                       const backend::ImportedResources* resource) noexcept {
      try {
        release(*resource);
      } catch (...) {
        // shared_ptr deleters must not throw.
      }
      delete resource;
    };
    using OwnedResource =
        std::unique_ptr<backend::ImportedResources, decltype(deleter)>;
    OwnedResource resource(new backend::ImportedResources(std::move(imported)),
                           deleter);
    resource_constructed = true;
    candidate = Entry(resource.get(), deleter);
    resource.release();
  } catch (...) {
    // If allocation failed before the resource took ownership, release the
    // caller-provided value.  Once the resource exists, its local guard has
    // already performed the release while unwinding.
    if (!resource_constructed) {
      try {
        release_fn_(imported);
      } catch (...) {
        // Preserve the original allocation/construction exception.  Cleanup
        // callbacks are best-effort and must not terminate stack unwinding.
      }
    }
    throw;
  }

  Entry result;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = cache_.find(key);
    if (it != cache_.end()) {
      result = it->second.lock();
      if (result) {
        // Keep the first successfully imported resource for this key.  The
        // candidate is released after the lock is dropped.
      } else {
        it->second = candidate;
        result = candidate;
      }
    } else {
      cache_.emplace(key, candidate);
      result = candidate;
    }
  }
  // A duplicate candidate is released here, after dropping the cache lock;
  // for a new key this is only the local shared_ptr copy.
  return result;
}

void IpcHandleCache::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  cache_.clear();
}

std::size_t IpcHandleCache::size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  prune_expired_locked();
  return cache_.size();
}

void IpcHandleCache::prune_expired_locked() const {
  for (auto it = cache_.begin(); it != cache_.end();) {
    if (it->second.expired()) {
      it = cache_.erase(it);
    } else {
      ++it;
    }
  }
}

}  // namespace ros2_cuda_ipc_core::subscriber
