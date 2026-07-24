// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <utility>

#include "ros2_cuda_ipc_core/backend/memory_importer.hpp"
#include "ros2_cuda_ipc_core/publisher_instance_id.hpp"
#include "ros2_cuda_ipc_core/transport/memory_types.hpp"

namespace ros2_cuda_ipc_core::subscriber {

struct IpcHandleKey {
  PublisherInstanceId publisher_instance_id{};
  uint8_t backend = 0;
  uint32_t device_id = 0;
  transport::MemoryHandlePayload mem{};
  transport::EventHandlePayload event{};

  bool operator==(const IpcHandleKey& other) const noexcept {
    return publisher_instance_id == other.publisher_instance_id &&
           backend == other.backend && device_id == other.device_id &&
           mem == other.mem && event == other.event;
  }
};

struct IpcHandleKeyHash {
  std::size_t operator()(const IpcHandleKey& key) const noexcept;
};

class IpcHandleCache {
 public:
  // The callback is copied into each Entry's shared_ptr deleter.  It may run
  // after this cache instance has been destroyed, so callers must provide a
  // self-contained callback and must not capture references to the cache or
  // other state with a shorter lifetime than the returned Entry.
  using ReleaseFn = std::function<void(const backend::ImportedResources&)>;
  using Entry = std::shared_ptr<const backend::ImportedResources>;

  explicit IpcHandleCache(
      ReleaseFn release_fn = backend::release_imported_resources_best_effort);

  ~IpcHandleCache();

  static IpcHandleCache& instance();

  Entry find(const IpcHandleKey& key) const;

  Entry insert_or_discard_duplicate(const IpcHandleKey& key,
                                    backend::ImportedResources imported);

  /// Remove all cached references without affecting resources held by views.
  void clear();

  std::size_t size() const;

 private:
  ReleaseFn release_fn_;
  mutable std::mutex mutex_;
  // Keep imported resources alive between messages.  BufferView also keeps a
  // shared ownership reference so clearing the cache cannot invalidate an
  // active view.
  std::unordered_map<IpcHandleKey, Entry, IpcHandleKeyHash> cache_;
};

}  // namespace ros2_cuda_ipc_core::subscriber
