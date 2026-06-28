#pragma once

#include <cuda_runtime_api.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <unordered_map>

#include "ros2_cuda_ipc_core/backend/backend_importer.hpp"
#include "ros2_cuda_ipc_core/memory_types.hpp"

namespace ros2_cuda_ipc_core {

struct IpcHandleKey {
  uint8_t backend = 0;
  MemoryHandlePayload mem{};
  std::array<uint8_t, sizeof(cudaIpcEventHandle_t)> event{};

  bool operator==(const IpcHandleKey& other) const noexcept {
    return backend == other.backend && mem == other.mem && event == other.event;
  }
};

struct IpcHandleKeyHash {
  std::size_t operator()(const IpcHandleKey& key) const noexcept;
};

class IpcHandleCache {
 public:
  using ReleaseFn = std::function<void(const backend::ImportedBuffer&)>;

  explicit IpcHandleCache(
      ReleaseFn release_fn = backend::release_imported_buffer);

  static IpcHandleCache& instance();

  std::optional<backend::ImportedBuffer> find(const IpcHandleKey& key) const;

  backend::ImportedBuffer insert_or_discard_duplicate(
      const IpcHandleKey& key, backend::ImportedBuffer imported);

  std::size_t size() const;

 private:
  ReleaseFn release_fn_;
  mutable std::mutex mutex_;
  std::unordered_map<IpcHandleKey, backend::ImportedBuffer, IpcHandleKeyHash>
      cache_;
};

}  // namespace ros2_cuda_ipc_core
