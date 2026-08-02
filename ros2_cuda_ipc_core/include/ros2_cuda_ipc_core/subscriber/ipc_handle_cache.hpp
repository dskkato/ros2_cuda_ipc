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

#include "ros2_cuda_ipc_core/backend/vmm_fd/memory_importer.hpp"
#include "ros2_cuda_ipc_core/publisher_instance_id.hpp"
#include "ros2_cuda_ipc_core/transport/memory_types.hpp"

namespace ros2_cuda_ipc_core::subscriber {

struct IpcHandleKey {
  PublisherInstanceId publisher_instance_id{};
  uint32_t device_id = 0;
  transport::MemoryHandlePayload mem{};
  transport::EventHandlePayload event{};

  bool operator==(const IpcHandleKey& other) const noexcept {
    return publisher_instance_id == other.publisher_instance_id &&
           device_id == other.device_id &&
           mem == other.mem && event == other.event;
  }
};

struct IpcHandleKeyHash {
  std::size_t operator()(const IpcHandleKey& key) const noexcept;
};

/// Internal cache for imported CUDA resources.
class IpcHandleCache {
 public:
  using ReleaseFn =
      std::function<void(const backend::vmm_fd::ImportedResources&)>;
  using Entry = std::shared_ptr<const backend::vmm_fd::ImportedResources>;

  explicit IpcHandleCache(
      ReleaseFn release_fn =
          backend::vmm_fd::release_imported_resources_best_effort);
  ~IpcHandleCache();

  static IpcHandleCache& instance();

  Entry find(const IpcHandleKey& key) const;
  Entry insert_or_discard_duplicate(const IpcHandleKey& key,
                                    backend::vmm_fd::ImportedResources imported);
  void clear();
  std::size_t size() const;

 private:
  ReleaseFn release_fn_;
  mutable std::mutex mutex_;
  std::unordered_map<IpcHandleKey, Entry, IpcHandleKeyHash> cache_;
};

}  // namespace ros2_cuda_ipc_core::subscriber
