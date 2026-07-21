// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda_runtime_api.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <unordered_map>

#include "ros2_cuda_ipc_core/backend/memory_importer.hpp"
#include "ros2_cuda_ipc_core/publisher_instance_id.hpp"
#include "ros2_cuda_ipc_core/transport/memory_types.hpp"

namespace ros2_cuda_ipc_core::subscriber {

struct IpcHandleKey {
  PublisherInstanceId publisher_instance_id{};
  uint8_t backend = 0;
  transport::MemoryHandlePayload mem{};
  std::array<uint8_t, sizeof(cudaIpcEventHandle_t)> event{};

  bool operator==(const IpcHandleKey& other) const noexcept {
    return publisher_instance_id == other.publisher_instance_id &&
           backend == other.backend && mem == other.mem && event == other.event;
  }
};

struct IpcHandleKeyHash {
  std::size_t operator()(const IpcHandleKey& key) const noexcept;
};

class IpcHandleCache {
 public:
  using ReleaseFn = std::function<void(const backend::ImportedMemory&)>;
  // Cache entries and BufferViews share this object.  Releasing a cache entry
  // therefore cannot close an IPC handle while a view still references it.
  using ImportedMemoryResource = std::shared_ptr<const backend::ImportedMemory>;

  explicit IpcHandleCache(
      ReleaseFn release_fn = backend::release_imported_memory);

  static IpcHandleCache& instance();

  ~IpcHandleCache();

  std::optional<ImportedMemoryResource> find(const IpcHandleKey& key) const;

  ImportedMemoryResource insert_or_discard_duplicate(
      const IpcHandleKey& key, backend::ImportedMemory imported);

  // Removes cached ownership. Resources retained by BufferViews are released
  // only after their leases have been released.
  void clear();

  std::size_t size() const;

 private:
  ReleaseFn release_fn_;
  mutable std::mutex mutex_;
  std::unordered_map<IpcHandleKey, ImportedMemoryResource, IpcHandleKeyHash>
      cache_;
};

}  // namespace ros2_cuda_ipc_core::subscriber
