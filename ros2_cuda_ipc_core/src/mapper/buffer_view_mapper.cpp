// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/mapper/buffer_view_mapper.hpp"

#include <cstring>
#include <memory>

#include "ros2_cuda_ipc_core/backend/backend_importer.hpp"
#include "ros2_cuda_ipc_core/ipc_handle_cache.hpp"
#include "ros2_cuda_ipc_core/lease_handle.hpp"

namespace ros2_cuda_ipc_core::mapper {

namespace {

cudaIpcEventHandle_t to_cuda_event_handle(
    const ros2_cuda_ipc_msgs::msg::BufferCore& msg) {
  cudaIpcEventHandle_t handle{};
  std::memcpy(&handle, msg.event_handle.data(), sizeof(handle));
  return handle;
}

bool is_supported_backend(uint8_t backend) noexcept {
  return backend == to_backend_byte(MemoryBackendKind::CUDA_IPC) ||
         backend == to_backend_byte(MemoryBackendKind::VMM_FD);
}

BufferViewMapper& default_buffer_view_mapper() {
  static BufferViewMapper mapper;
  return mapper;
}

}  // namespace

BufferViewMapper::BufferViewMapper(BufferViewMapperOptions options)
    : options_(std::move(options)) {}

view::BufferView BufferViewMapper::map(
    const ros2_cuda_ipc_msgs::msg::BufferCore& msg) const {
  if (!is_supported_backend(static_cast<uint8_t>(msg.backend))) {
    RCLCPP_WARN(options_.logger, "Unsupported BufferCore.backend=%u",
                static_cast<unsigned>(msg.backend));
    return {};
  }

  auto lease = LeaseHandle::acquire(msg.shm_name, msg.slot_id, msg.generation);
  if (!lease.valid()) {
    RCLCPP_WARN(options_.logger,
                "Failed to acquire lease shm=%s slot=%u gen=%u",
                msg.shm_name.c_str(), msg.slot_id, msg.generation);
    return {};
  }

  auto lease_ptr = std::make_shared<LeaseHandle>(std::move(lease));
  const cudaIpcEventHandle_t event_handle = to_cuda_event_handle(msg);

  IpcHandleKey key{};
  key.backend = static_cast<uint8_t>(msg.backend);
  key.mem = msg.mem_handle;
  std::memcpy(key.event.data(), &event_handle, sizeof(event_handle));

  backend::ImportedBuffer imported;
  auto cached = IpcHandleCache::instance().find(key);
  if (cached.has_value()) {
    imported = *cached;
  } else {
    const auto& importer =
        backend::get_backend_importer(static_cast<uint8_t>(msg.backend));
    auto opened = importer.import(msg, event_handle, options_.logger);
    if (!opened.has_value()) {
      return {};
    }
    imported =
        IpcHandleCache::instance().insert_or_discard_duplicate(key, *opened);
  }

  view::BufferView view;
  view.dev_ptr = imported.dev_ptr;
  view.ready_evt = imported.event;
  view.device_id = static_cast<int>(msg.device_id);
  view.byte_size = msg.byte_size;
  view.slot_id = msg.slot_id;
  view.generation = msg.generation;
  view.shm_name = msg.shm_name;
  view.lease = std::move(lease_ptr);
  view.set_ipc_handles(backend_from_byte(static_cast<uint8_t>(msg.backend)),
                       msg.mem_handle.data(), msg.mem_handle.size(),
                       event_handle);
  return view;
}

view::BufferView map_buffer_view(
    const ros2_cuda_ipc_msgs::msg::BufferCore& msg) {
  return default_buffer_view_mapper().map(msg);
}

void fill_buffer_core_message(const view::BufferView& view,
                              ros2_cuda_ipc_msgs::msg::BufferCore& msg) {
  msg.shm_name = view.shm_name;
  msg.device_id = static_cast<uint32_t>(view.device_id);
  msg.slot_id = view.slot_id;
  msg.generation = view.generation;
  msg.byte_size = view.byte_size;
  msg.backend = to_backend_byte(view.backend());
  if (view.handles_ready()) {
    const auto& payload = view.mem_payload();
    std::memcpy(msg.mem_handle.data(), payload.data(), payload.size());
    std::memcpy(msg.event_handle.data(), &view.event_handle(),
                sizeof(cudaIpcEventHandle_t));
  } else {
    std::memset(msg.mem_handle.data(), 0, msg.mem_handle.size());
    std::memset(msg.event_handle.data(), 0, msg.event_handle.size());
  }
}

}  // namespace ros2_cuda_ipc_core::mapper
