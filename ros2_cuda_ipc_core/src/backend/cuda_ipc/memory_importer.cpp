// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/backend/cuda_ipc/memory_importer.hpp"

#include <cstring>

#include "rclcpp/logging.hpp"
#include "ros2_cuda_ipc_core/detail/cuda_driver_context.hpp"
#include "ros2_cuda_ipc_core/transport/memory_types.hpp"

namespace ros2_cuda_ipc_core::backend::cuda_ipc {

namespace {

cudaIpcMemHandle_t to_cuda_mem_handle(
    const ros2_cuda_ipc_msgs::msg::BufferCore& msg) {
  cudaIpcMemHandle_t handle{};
  std::memcpy(&handle, msg.mem_handle.data(), sizeof(handle));
  return handle;
}

}  // namespace

std::optional<ImportedMemory> MemoryImporter::import(
    const ros2_cuda_ipc_msgs::msg::BufferCore& msg,
    const CUipcEventHandle& event_handle, const rclcpp::Logger& logger) const {
  auto context_result = detail::CudaDeviceContext::retain_primary(
      static_cast<int>(msg.device_id));
  if (!context_result) {
    RCLCPP_WARN(logger, "Failed to retain CUDA primary context: %s",
                context_result.error().to_string().c_str());
    return std::nullopt;
  }
  auto context = std::move(context_result).value();
  auto guard_result = context->push_current();
  if (!guard_result) {
    RCLCPP_WARN(logger, "Failed to activate CUDA context: %s",
                guard_result.error().to_string().c_str());
    return std::nullopt;
  }
  auto guard = std::move(guard_result).value();

  ImportedMemory imported;
  imported.context = context;

  CUresult result = cuIpcOpenEventHandle(&imported.event, event_handle);
  if (result != CUDA_SUCCESS) {
    RCLCPP_WARN(logger, "cuIpcOpenEventHandle failed: %s",
                detail::CudaDriverError(result).to_string().c_str());
    return std::nullopt;
  }

  const cudaIpcMemHandle_t mem_handle = to_cuda_mem_handle(msg);
  const cudaError_t err = cudaIpcOpenMemHandle(&imported.dev_ptr, mem_handle,
                                               cudaIpcMemLazyEnablePeerAccess);
  if (err != cudaSuccess) {
    RCLCPP_WARN(logger, "cudaIpcOpenMemHandle failed: %s",
                cudaGetErrorString(err));
    (void)cuEventDestroy(imported.event);
    imported.event = nullptr;
    return std::nullopt;
  }

  return imported;
}

}  // namespace ros2_cuda_ipc_core::backend::cuda_ipc
