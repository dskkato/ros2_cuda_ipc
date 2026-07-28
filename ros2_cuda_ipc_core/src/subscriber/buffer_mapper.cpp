// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/subscriber/buffer_mapper.hpp"

#include <rcutils/logging_macros.h>

#include <cstring>
#include <functional>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>

#include "read_handle_factory.hpp"
#include "ros2_cuda_ipc_core/backend/memory_importer.hpp"
#include "ros2_cuda_ipc_core/lease/lease_handle.hpp"
#include "ros2_cuda_ipc_core/lease/lease_mapping.hpp"
#include "ros2_cuda_ipc_core/subscriber/ipc_handle_cache.hpp"
#include "ros2_cuda_ipc_core/transport/memory_types.hpp"

namespace ros2_cuda_ipc_core::subscriber {
namespace {

class LeaseMappingCache {
 public:
  using AttachFn = std::function<std::shared_ptr<lease::LeaseMapping>(
      const std::string&, const PublisherInstanceId&)>;

  explicit LeaseMappingCache(AttachFn attach_fn = lease::LeaseMapping::attach)
      : attach_fn_(std::move(attach_fn)) {}

  std::shared_ptr<lease::LeaseMapping> get_or_attach(
      const std::string& shm_name,
      const PublisherInstanceId& publisher_instance_id) const {
    const Key key{shm_name, publisher_instance_id};
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto it = mappings_.find(key);
      if (it != mappings_.end()) {
        return it->second;
      }
    }

    auto candidate = attach_fn_(shm_name, publisher_instance_id);
    if (!candidate) {
      return nullptr;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    const auto [it, inserted] = mappings_.emplace(key, std::move(candidate));
    (void)inserted;
    return it->second;
  }

 private:
  struct Key {
    std::string shm_name;
    PublisherInstanceId publisher_instance_id{};

    bool operator==(const Key& other) const noexcept {
      return shm_name == other.shm_name &&
             publisher_instance_id == other.publisher_instance_id;
    }
  };

  struct KeyHash {
    std::size_t operator()(const Key& key) const noexcept {
      std::size_t hash = std::hash<std::string>{}(key.shm_name);
      for (const uint8_t byte : key.publisher_instance_id) {
        hash ^= static_cast<std::size_t>(byte) +
                static_cast<std::size_t>(0x9e3779b9) + (hash << 6) +
                (hash >> 2);
      }
      return hash;
    }
  };

  AttachFn attach_fn_;
  mutable std::mutex mutex_;
  mutable std::unordered_map<Key, std::shared_ptr<lease::LeaseMapping>, KeyHash>
      mappings_;
};

CUipcEventHandle to_cuda_event_handle(
    const ros2_cuda_ipc_msgs::msg::BufferCore& msg) {
  CUipcEventHandle handle{};
  static_assert(sizeof(handle) == transport::EventHandlePayload{}.size(),
                "CUDA IPC event handle payload size changed");
  std::memcpy(&handle, msg.event_handle.data(), sizeof(handle));
  return handle;
}

bool is_supported_backend(uint8_t backend) noexcept {
  return backend == transport::to_backend_byte(
                        transport::MemoryBackendKind::CUDA_IPC) ||
         backend ==
             transport::to_backend_byte(transport::MemoryBackendKind::VMM_FD);
}

std::optional<ReadHandle> map_descriptor(
    const std::shared_ptr<LeaseMappingCache>& mapping_cache,
    const ros2_cuda_ipc_msgs::msg::BufferCore& msg, CUstream consumer_stream,
    bool bind_stream) {
  if (!is_supported_backend(static_cast<uint8_t>(msg.backend))) {
    RCUTILS_LOG_WARN_NAMED("ros2_cuda_ipc_core.subscriber.buffer_mapper",
                           "Unsupported BufferCore.backend=%u",
                           static_cast<unsigned>(msg.backend));
    return std::nullopt;
  }

  const PublisherInstanceId instance_id = msg.publisher_instance_id;
  if (is_nil(instance_id)) {
    RCUTILS_LOG_WARN_NAMED("ros2_cuda_ipc_core.subscriber.buffer_mapper",
                           "BufferCore publisher_instance_id is nil");
    return std::nullopt;
  }

  auto mapping = mapping_cache->get_or_attach(msg.shm_name, instance_id);
  auto lease =
      lease::LeaseHandle::acquire(mapping, msg.slot_id, msg.generation);
  if (!lease.valid()) {
    RCUTILS_LOG_WARN_NAMED("ros2_cuda_ipc_core.subscriber.buffer_mapper",
                           "Failed to acquire lease shm=%s slot=%u gen=%u",
                           msg.shm_name.c_str(), msg.slot_id, msg.generation);
    return std::nullopt;
  }
  auto lease_ptr = std::make_unique<lease::LeaseHandle>(std::move(lease));

  const CUipcEventHandle event_handle = to_cuda_event_handle(msg);
  IpcHandleKey key{};
  key.publisher_instance_id = instance_id;
  key.backend = static_cast<uint8_t>(msg.backend);
  key.device_id = msg.device_id;
  key.mem = msg.mem_handle;
  std::memcpy(key.event.data(), msg.event_handle.data(), key.event.size());

  auto imported = IpcHandleCache::instance().find(key);
  if (!imported) {
    const auto& importer =
        backend::get_memory_importer(static_cast<uint8_t>(msg.backend));
    auto opened = importer.import(msg, event_handle);
    if (!opened.has_value()) {
      RCUTILS_LOG_WARN_NAMED(
          "ros2_cuda_ipc_core.subscriber.buffer_mapper",
          "Failed to import GPU resource shm=%s slot=%u gen=%u",
          msg.shm_name.c_str(), msg.slot_id, msg.generation);
      return std::nullopt;
    }
    imported = IpcHandleCache::instance().insert_or_discard_duplicate(
        key, std::move(*opened));
  }

  std::optional<ReadHandle> read;
  if (bind_stream) {
    read = detail::ReadHandleFactory::make(
        std::move(imported), std::move(lease_ptr),
        static_cast<std::size_t>(msg.byte_size),
        static_cast<int>(msg.device_id), consumer_stream);
  } else {
    read = detail::ReadHandleFactory::make_unbound(
        std::move(imported), std::move(lease_ptr),
        static_cast<std::size_t>(msg.byte_size),
        static_cast<int>(msg.device_id));
  }
  if (!read) {
    RCUTILS_LOG_WARN_NAMED(
        "ros2_cuda_ipc_core.subscriber.buffer_mapper",
        "Failed to create GPU read for shm=%s slot=%u gen=%u",
        msg.shm_name.c_str(), msg.slot_id, msg.generation);
  }
  return read;
}

}  // namespace

class BufferMapper::Impl {
 public:
  std::shared_ptr<LeaseMappingCache> mapping_cache =
      std::make_shared<LeaseMappingCache>();
};

BufferMapper::BufferMapper() : impl_(std::make_unique<Impl>()) {}
BufferMapper::~BufferMapper() = default;
BufferMapper::BufferMapper(BufferMapper&&) noexcept = default;
BufferMapper& BufferMapper::operator=(BufferMapper&&) noexcept = default;

std::optional<ReadHandle> BufferMapper::map(
    const ros2_cuda_ipc_msgs::msg::BufferCore& msg,
    CUstream consumer_stream) const {
  return map_descriptor(impl_->mapping_cache, msg, consumer_stream, true);
}

std::optional<ReadHandle> BufferMapper::map_unbound(
    const ros2_cuda_ipc_msgs::msg::BufferCore& msg) const {
  return map_descriptor(impl_->mapping_cache, msg, nullptr, false);
}

}  // namespace ros2_cuda_ipc_core::subscriber
