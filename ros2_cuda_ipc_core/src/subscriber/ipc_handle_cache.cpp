// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/subscriber/ipc_handle_cache.hpp"

#include <memory>

namespace ros2_cuda_ipc_core::subscriber {

std::size_t IpcHandleKeyHash::operator()(
    const IpcHandleKey& key) const noexcept {
  constexpr std::size_t PRIME{131};
  std::size_t hash = 0;
  for (unsigned int shift = 0; shift < sizeof(key.device_id) * 8; shift += 8) {
    hash = hash * PRIME + ((key.device_id >> shift) & 0xffU);
  }
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
  const auto it = cache_.find(key);
  if (it == cache_.end()) {
    return {};
  }
  return it->second;
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
                           std::move(deleter));
    resource_constructed = true;
    // The unique_ptr owns the resource while shared_ptr allocates its control
    // block.  If this copy throws, the unique_ptr still invokes the moved-in
    // deleter during unwinding.
    candidate = Entry(resource.get(), resource.get_deleter());
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
    const auto [it, inserted] = cache_.emplace(key, candidate);
    (void)inserted;
    // Keep the first imported resource for this key.  If this was a
    // duplicate, candidate remains a local shared_ptr and is released after
    // this scope drops the cache mutex.
    result = it->second;
  }
  // A duplicate candidate is released here, after dropping the cache lock;
  // for a new key this is only the local shared_ptr copy.
  return result;
}

void IpcHandleCache::clear() {
  decltype(cache_) entries;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    entries.swap(cache_);
  }
  // Imported resource cleanup can call arbitrary backend code.  Do not run
  // it while holding the cache mutex.
  entries.clear();
}

std::size_t IpcHandleCache::size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return cache_.size();
}

}  // namespace ros2_cuda_ipc_core::subscriber
