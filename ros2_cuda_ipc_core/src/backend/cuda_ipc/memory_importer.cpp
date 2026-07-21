// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/backend/cuda_ipc/memory_importer.hpp"

#include <cstring>

#include "rclcpp/logging.hpp"
#include "ros2_cuda_ipc_core/detail/cuda_util.hpp"
#include "ros2_cuda_ipc_core/detail/primary_context_guard.hpp"
#include "ros2_cuda_ipc_core/transport/memory_types.hpp"

namespace ros2_cuda_ipc_core::backend::cuda_ipc {

namespace {

CUipcMemHandle to_cuda_mem_handle(
    const ros2_cuda_ipc_msgs::msg::BufferCore& msg) {
  CUipcMemHandle handle{};
  std::memcpy(&handle, msg.mem_handle.data(), sizeof(handle));
  return handle;
}

}  // namespace

std::optional<ImportedMemory> MemoryImporter::import(
    const ros2_cuda_ipc_msgs::msg::BufferCore& msg,
    const CUipcEventHandle& event_handle, const rclcpp::Logger& logger) const {
  detail::PrimaryContextGuard context(static_cast<int>(msg.device_id));
  if (!context.ok()) {
    RCLCPP_WARN(logger, "Unable to make primary context current: %s",
                detail::cu_result_to_string(context.result()).c_str());
    return std::nullopt;
  }
  ImportedMemory imported;
  imported.device_id = static_cast<int>(msg.device_id);

  auto err = cuIpcOpenEventHandle(&imported.event, event_handle);
  if (err != CUDA_SUCCESS) {
    RCLCPP_WARN(logger, "cuIpcOpenEventHandle failed: %s",
                detail::cu_result_to_string(err).c_str());
    return std::nullopt;
  }

  const CUipcMemHandle mem_handle = to_cuda_mem_handle(msg);
  CUdeviceptr ptr = 0;
  err =
      cuIpcOpenMemHandle(&ptr, mem_handle, CU_IPC_MEM_LAZY_ENABLE_PEER_ACCESS);
  if (err != CUDA_SUCCESS) {
    RCLCPP_WARN(logger, "cuIpcOpenMemHandle failed: %s",
                detail::cu_result_to_string(err).c_str());
    cuEventDestroy(imported.event);
    return std::nullopt;
  }
  imported.dev_ptr = reinterpret_cast<void*>(ptr);

  return imported;
}

}  // namespace ros2_cuda_ipc_core::backend::cuda_ipc
