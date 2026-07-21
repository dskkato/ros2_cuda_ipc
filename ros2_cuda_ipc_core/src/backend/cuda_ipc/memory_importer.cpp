// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/backend/cuda_ipc/memory_importer.hpp"

#include <cstring>

#include "rclcpp/logging.hpp"
#include "ros2_cuda_ipc_core/detail/cuda_util.hpp"
#include "ros2_cuda_ipc_core/transport/memory_types.hpp"

namespace ros2_cuda_ipc_core::backend::cuda_ipc {

namespace {

using BufferCoreMessage = ros2_cuda_ipc_msgs::msg::BufferCore;

static_assert(sizeof(BufferCoreMessage::_mem_handle_type) ==
                  sizeof(CUipcMemHandle),
              "BufferCore.mem_handle must match CUipcMemHandle");
static_assert(sizeof(BufferCoreMessage::_event_handle_type) ==
                  sizeof(CUipcEventHandle),
              "BufferCore.event_handle must match CUipcEventHandle");

CUipcMemHandle to_ipc_mem_handle(
    const ros2_cuda_ipc_msgs::msg::BufferCore& msg) {
  CUipcMemHandle handle{};
  std::memcpy(&handle, msg.mem_handle.data(), sizeof(handle));
  return handle;
}

CUipcEventHandle to_ipc_event_handle(
    const ros2_cuda_ipc_msgs::msg::BufferCore& msg) {
  CUipcEventHandle handle{};
  std::memcpy(&handle, msg.event_handle.data(), sizeof(handle));
  return handle;
}

}  // namespace

std::optional<ImportedMemory> MemoryImporter::import(
    const ros2_cuda_ipc_msgs::msg::BufferCore& msg,
    const rclcpp::Logger& logger) const {
  ImportedMemory imported;

  const CUresult event_result =
      cuIpcOpenEventHandle(&imported.event, to_ipc_event_handle(msg));
  if (event_result != CUDA_SUCCESS) {
    RCLCPP_WARN(
        logger, "cuIpcOpenEventHandle failed: %s",
        ros2_cuda_ipc_core::detail::cu_result_to_string(event_result).c_str());
    return std::nullopt;
  }

  const CUresult memory_result =
      cuIpcOpenMemHandle(&imported.dev_ptr, to_ipc_mem_handle(msg),
                         CU_IPC_MEM_LAZY_ENABLE_PEER_ACCESS);
  if (memory_result != CUDA_SUCCESS) {
    RCLCPP_WARN(
        logger, "cuIpcOpenMemHandle failed: %s",
        ros2_cuda_ipc_core::detail::cu_result_to_string(memory_result).c_str());
    (void)cuEventDestroy(imported.event);
    return std::nullopt;
  }

  return imported;
}

}  // namespace ros2_cuda_ipc_core::backend::cuda_ipc
