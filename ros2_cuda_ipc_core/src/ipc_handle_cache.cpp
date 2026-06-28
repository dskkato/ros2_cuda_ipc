#include "ros2_cuda_ipc_core/ipc_handle_cache.hpp"

namespace ros2_cuda_ipc_core {

std::size_t IpcHandleKeyHash::operator()(
    const IpcHandleKey& key) const noexcept {
  std::size_t hash = key.backend;
  for (uint8_t byte : key.mem) {
    hash = hash * 131U + byte;
  }
  for (uint8_t byte : key.event) {
    hash = hash * 131U + byte;
  }
  return hash;
}

IpcHandleCache::IpcHandleCache(ReleaseFn release_fn)
    : release_fn_(std::move(release_fn)) {}

IpcHandleCache& IpcHandleCache::instance() {
  static IpcHandleCache cache;
  return cache;
}

std::optional<detail::ImportedBuffer> IpcHandleCache::find(
    const IpcHandleKey& key) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = cache_.find(key);
  if (it == cache_.end()) {
    return std::nullopt;
  }
  return it->second;
}

detail::ImportedBuffer IpcHandleCache::insert_or_discard_duplicate(
    const IpcHandleKey& key, detail::ImportedBuffer imported) {
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

}  // namespace ros2_cuda_ipc_core
