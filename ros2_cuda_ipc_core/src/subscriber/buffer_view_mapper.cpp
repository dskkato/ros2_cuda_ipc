// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/subscriber/buffer_view_mapper.hpp"

#include <cstring>
#include <memory>
#include <utility>

#include "ros2_cuda_ipc_core/backend/memory_importer.hpp"
#include "ros2_cuda_ipc_core/lease/lease_handle.hpp"
#include "ros2_cuda_ipc_core/subscriber/ipc_handle_cache.hpp"
#include "ros2_cuda_ipc_core/transport/memory_types.hpp"

namespace ros2_cuda_ipc_core::subscriber {

namespace {

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

BufferViewMapper& default_buffer_view_mapper() {
  static BufferViewMapper mapper;
  return mapper;
}

}  // namespace

std::size_t LeaseMappingCache::KeyHash::operator()(
    const Key& key) const noexcept {
  std::size_t hash = std::hash<std::string>{}(key.shm_name);
  for (const uint8_t byte : key.publisher_instance_id) {
    hash ^= static_cast<std::size_t>(byte) +
            static_cast<std::size_t>(0x9e3779b9) + (hash << 6) + (hash >> 2);
  }
  return hash;
}

LeaseMappingCache::LeaseMappingCache(AttachFn attach_fn)
    : attach_fn_(std::move(attach_fn)) {}

LeaseMappingCache::~LeaseMappingCache() { clear(); }

std::shared_ptr<lease::LeaseMapping> LeaseMappingCache::get_or_attach(
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

  std::shared_ptr<lease::LeaseMapping> result;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto [it, inserted] = mappings_.emplace(key, candidate);
    (void)inserted;
    // Keep the first mapping for this key. If this was a duplicate, the
    // candidate remains local and is released after this scope drops the
    // cache mutex.
    result = it->second;
  }
  return result;
}

void LeaseMappingCache::clear() const {
  decltype(mappings_) entries;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    entries.swap(mappings_);
  }
  // LeaseMapping destruction calls munmap(2). Do not run it while holding the
  // cache mutex, because destruction may call code that needs this cache.
  entries.clear();
}

std::size_t LeaseMappingCache::size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return mappings_.size();
}

BufferViewMapper::BufferViewMapper(BufferViewMapperOptions options)
    : options_(std::move(options)),
      mapping_cache_(std::make_shared<LeaseMappingCache>()) {}

BufferView BufferViewMapper::map(
    const ros2_cuda_ipc_msgs::msg::BufferCore& msg) const {
  if (!is_supported_backend(static_cast<uint8_t>(msg.backend))) {
    RCLCPP_WARN(options_.logger, "Unsupported BufferCore.backend=%u",
                static_cast<unsigned>(msg.backend));
    return {};
  }

  const PublisherInstanceId instance_id = msg.publisher_instance_id;
  if (is_nil(instance_id)) {
    RCLCPP_WARN(options_.logger, "BufferCore publisher_instance_id is nil");
    return {};
  }

  auto mapping = mapping_cache_->get_or_attach(msg.shm_name, instance_id);
  auto lease =
      lease::LeaseHandle::acquire(mapping, msg.slot_id, msg.generation);
  if (!lease.valid()) {
    RCLCPP_WARN(options_.logger,
                "Failed to acquire lease shm=%s slot=%u gen=%u",
                msg.shm_name.c_str(), msg.slot_id, msg.generation);
    return {};
  }

  auto lease_ptr = std::make_shared<lease::LeaseHandle>(std::move(lease));
  const CUipcEventHandle event_handle = to_cuda_event_handle(msg);
  transport::EventHandlePayload event_payload{};
  std::memcpy(event_payload.data(), msg.event_handle.data(),
              event_payload.size());

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
    auto opened = importer.import(msg, event_handle, options_.logger);
    if (!opened.has_value()) {
      return {};
    }
    imported = IpcHandleCache::instance().insert_or_discard_duplicate(
        key, std::move(*opened));
  }

  BufferView view;
  view.set_imported_resource(imported);
  view.device_id = static_cast<int>(msg.device_id);
  view.byte_size = msg.byte_size;
  view.slot_id = msg.slot_id;
  view.generation = msg.generation;
  view.shm_name = msg.shm_name;
  view.publisher_instance_id = instance_id;
  view.lease = std::move(lease_ptr);
  view.set_ipc_handles(
      transport::backend_from_byte(static_cast<uint8_t>(msg.backend)),
      msg.mem_handle.data(), msg.mem_handle.size(), event_payload);
  return view;
}

BufferView map_buffer_view(const ros2_cuda_ipc_msgs::msg::BufferCore& msg) {
  return default_buffer_view_mapper().map(msg);
}

}  // namespace ros2_cuda_ipc_core::subscriber
