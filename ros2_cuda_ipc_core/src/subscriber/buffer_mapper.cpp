// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/subscriber/buffer_mapper.hpp"

#include <rcutils/logging_macros.h>

#include <cstring>
#include <utility>

#include "ros2_cuda_ipc_core/backend/memory_importer.hpp"
#include "ros2_cuda_ipc_core/buffer_metadata/buffer_ref.hpp"
#include "ros2_cuda_ipc_core/detail/buffer_metadata_cache.hpp"
#include "ros2_cuda_ipc_core/detail/read_handle_factory.hpp"
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

std::unique_ptr<detail::MappedPublication> map_descriptor(
    const std::shared_ptr<detail::BufferMetadataCache>& mapping_cache,
    const ros2_cuda_ipc_msgs::msg::BufferCore& msg) {
  const PublisherInstanceId instance_id = msg.publisher_instance_id;
  if (is_nil(instance_id)) {
    RCUTILS_LOG_WARN_NAMED("ros2_cuda_ipc_core.subscriber.buffer_mapper",
                           "BufferCore publisher_instance_id is nil");
    return nullptr;
  }

  auto mapping = mapping_cache->get_or_attach(msg.shm_name, instance_id);
  auto buffer_ref =
      buffer_metadata::BufferRef::acquire(mapping, msg.slot_id, msg.generation);
  if (!buffer_ref.valid()) {
    RCUTILS_LOG_WARN_NAMED(
        "ros2_cuda_ipc_core.subscriber.buffer_mapper",
        "Failed to acquire buffer reference shm=%s slot=%u gen=%u",
        msg.shm_name.c_str(), msg.slot_id, msg.generation);
    return nullptr;
  }
  auto buffer_ref_ptr =
      std::make_unique<buffer_metadata::BufferRef>(std::move(buffer_ref));

  const CUipcEventHandle event_handle = to_cuda_event_handle(msg);
  IpcHandleKey key{};
  key.publisher_instance_id = instance_id;
  key.device_id = msg.device_id;
  key.vmm_socket_path = msg.vmm_socket_path;
  std::memcpy(key.event.data(), msg.event_handle.data(), key.event.size());

  auto imported = IpcHandleCache::instance().find(key);
  if (!imported) {
    static const backend::VmmFdMemoryImporter importer;
    auto opened = importer.import(msg, event_handle);
    if (!opened.has_value()) {
      RCUTILS_LOG_WARN_NAMED(
          "ros2_cuda_ipc_core.subscriber.buffer_mapper",
          "Failed to import GPU resource shm=%s slot=%u gen=%u",
          msg.shm_name.c_str(), msg.slot_id, msg.generation);
      return nullptr;
    }
    imported = IpcHandleCache::instance().insert_or_discard_duplicate(
        key, std::move(*opened));
  }

  auto publication = detail::ReadHandleFactory::make_publication(
      std::move(imported), std::move(buffer_ref_ptr),
      static_cast<std::size_t>(msg.byte_size), static_cast<int>(msg.device_id));
  if (!publication) {
    RCUTILS_LOG_WARN_NAMED(
        "ros2_cuda_ipc_core.subscriber.buffer_mapper",
        "Failed to create mapped publication for shm=%s slot=%u gen=%u",
        msg.shm_name.c_str(), msg.slot_id, msg.generation);
  }
  return publication;
}

}  // namespace

class BufferMapper::Impl {
 public:
  std::shared_ptr<detail::BufferMetadataCache> mapping_cache =
      std::make_shared<detail::BufferMetadataCache>();
};

BufferMapper::BufferMapper() : impl_(std::make_unique<Impl>()) {}
BufferMapper::~BufferMapper() = default;
BufferMapper::BufferMapper(BufferMapper&&) noexcept = default;
BufferMapper& BufferMapper::operator=(BufferMapper&&) noexcept = default;

std::optional<ReadHandle> BufferMapper::map(
    const ros2_cuda_ipc_msgs::msg::BufferCore& msg,
    CUstream consumer_stream) const {
  auto publication = map_publication(msg);
  if (!publication) {
    return std::nullopt;
  }

  auto read =
      detail::ReadHandleFactory::make_bound(*publication, consumer_stream);
  if (!read) {
    RCUTILS_LOG_WARN_NAMED(
        "ros2_cuda_ipc_core.subscriber.buffer_mapper",
        "Failed to bind mapped publication for shm=%s slot=%u gen=%u",
        msg.shm_name.c_str(), msg.slot_id, msg.generation);
  }
  return read;
}

std::unique_ptr<detail::MappedPublication> BufferMapper::map_publication(
    const ros2_cuda_ipc_msgs::msg::BufferCore& msg) const {
  return map_descriptor(impl_->mapping_cache, msg);
}

}  // namespace ros2_cuda_ipc_core::subscriber
