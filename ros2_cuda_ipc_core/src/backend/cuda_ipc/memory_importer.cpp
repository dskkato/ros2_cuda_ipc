// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/backend/cuda_ipc/memory_importer.hpp"

#include <cstring>

#include "rclcpp/logging.hpp"
#include "ros2_cuda_ipc_core/detail/cuda_util.hpp"
#include "ros2_cuda_ipc_core/transport/memory_types.hpp"

namespace ros2_cuda_ipc_core::backend::cuda_ipc {

namespace {

static_assert(sizeof(CUipcMemHandle) ==
                  sizeof(ros2_cuda_ipc_msgs::msg::BufferCore::_mem_handle_type),
              "BufferCore.mem_handle must match CUipcMemHandle");

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
  ImportedMemory imported;

  auto err = cuIpcOpenEventHandle(&imported.event, event_handle);
  if (err != CUDA_SUCCESS) {
    RCLCPP_WARN(logger, "cuIpcOpenEventHandle failed: %s",
                ros2_cuda_ipc_core::detail::cu_result_to_string(err).c_str());
    return std::nullopt;
  }

  const CUipcMemHandle mem_handle = to_cuda_mem_handle(msg);
  CUdeviceptr device_ptr = 0;
  err = cuIpcOpenMemHandle(&device_ptr, mem_handle,
                           CU_IPC_MEM_LAZY_ENABLE_PEER_ACCESS);
  imported.dev_ptr = reinterpret_cast<void*>(device_ptr);
  if (err != CUDA_SUCCESS) {
    RCLCPP_WARN(logger, "cuIpcOpenMemHandle failed: %s",
                ros2_cuda_ipc_core::detail::cu_result_to_string(err).c_str());
    cuEventDestroy(imported.event);
    return std::nullopt;
  }

  return imported;
}

}  // namespace ros2_cuda_ipc_core::backend::cuda_ipc
