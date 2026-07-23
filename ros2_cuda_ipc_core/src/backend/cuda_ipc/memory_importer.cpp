// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/backend/cuda_ipc/memory_importer.hpp"

#include <cstring>

#include "rclcpp/logging.hpp"
#include "ros2_cuda_ipc_core/detail/cuda_driver_context.hpp"
#include "ros2_cuda_ipc_core/detail/cuda_util.hpp"
#include "ros2_cuda_ipc_core/transport/memory_types.hpp"

namespace ros2_cuda_ipc_core::backend::cuda_ipc {

namespace {

CUipcMemHandle to_driver_mem_handle(
    const ros2_cuda_ipc_msgs::msg::BufferCore& msg) {
  CUipcMemHandle handle{};
  std::memcpy(&handle, msg.mem_handle.data(), sizeof(handle));
  return handle;
}

CUipcEventHandle to_driver_event_handle(const cudaIpcEventHandle_t& event) {
  CUipcEventHandle handle{};
  std::memcpy(&handle, &event, sizeof(handle));
  return handle;
}

}  // namespace

std::optional<ImportedMemory> MemoryImporter::import(
    const ros2_cuda_ipc_msgs::msg::BufferCore& msg,
    const cudaIpcEventHandle_t& event_handle,
    const rclcpp::Logger& logger) const {
  ImportedMemory imported;
  imported.device_id = static_cast<int>(msg.device_id);
  detail::ScopedPrimaryContext context(imported.device_id);
  if (!context.ok()) {
    RCLCPP_WARN(logger, "CUDA Driver context setup failed: %s",
                detail::cu_result_to_string(context.status()).c_str());
    return std::nullopt;
  }

  CUevent event = nullptr;
  CUresult result =
      cuIpcOpenEventHandle(&event, to_driver_event_handle(event_handle));
  if (result != CUDA_SUCCESS) {
    RCLCPP_WARN(logger, "cuIpcOpenEventHandle failed: %s",
                detail::cu_result_to_string(result).c_str());
    return std::nullopt;
  }
  imported.event = reinterpret_cast<cudaEvent_t>(event);

  CUdeviceptr device_ptr = 0;
  result = cuIpcOpenMemHandle(&device_ptr, to_driver_mem_handle(msg),
                              CU_IPC_MEM_LAZY_ENABLE_PEER_ACCESS);
  if (result != CUDA_SUCCESS) {
    RCLCPP_WARN(logger, "cuIpcOpenMemHandle failed: %s",
                detail::cu_result_to_string(result).c_str());
    (void)cuEventDestroy(event);
    return std::nullopt;
  }
  imported.dev_ptr =
      reinterpret_cast<void*>(static_cast<uintptr_t>(device_ptr));
  imported.driver_owned = true;

  return imported;
}

}  // namespace ros2_cuda_ipc_core::backend::cuda_ipc
