// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/backend/cuda_ipc/memory_importer.hpp"

#include <cstring>

#include "rclcpp/logging.hpp"
#include "ros2_cuda_ipc_core/detail/cuda_context.hpp"
#include "ros2_cuda_ipc_core/detail/cuda_util.hpp"
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
    const cudaIpcEventHandle_t& event_handle,
    const rclcpp::Logger& logger) const {
  ImportedMemory imported;
  CUresult context_result = CUDA_SUCCESS;
  auto context = detail::CudaContextGuard::for_device(
      static_cast<int>(msg.device_id), &context_result);
  if (!context) {
    RCLCPP_WARN(logger, "Unable to make device %u context current: %s",
                msg.device_id,
                detail::cu_result_to_string(context_result).c_str());
    return std::nullopt;
  }
  imported.device = static_cast<CUdevice>(msg.device_id);
  imported.context = context.context();

  auto err = cudaIpcOpenEventHandle(&imported.event, event_handle);
  if (err != cudaSuccess) {
    RCLCPP_WARN(logger, "cudaIpcOpenEventHandle failed: %s",
                cudaGetErrorString(err));
    return std::nullopt;
  }

  const cudaIpcMemHandle_t mem_handle = to_cuda_mem_handle(msg);
  err = cudaIpcOpenMemHandle(&imported.dev_ptr, mem_handle,
                             cudaIpcMemLazyEnablePeerAccess);
  if (err != cudaSuccess) {
    RCLCPP_WARN(logger, "cudaIpcOpenMemHandle failed: %s",
                cudaGetErrorString(err));
    cudaEventDestroy(imported.event);
    return std::nullopt;
  }

  return imported;
}

}  // namespace ros2_cuda_ipc_core::backend::cuda_ipc
