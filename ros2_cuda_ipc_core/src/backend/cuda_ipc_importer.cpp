// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/backend/cuda_ipc_importer.hpp"

#include <cstring>

#include "rclcpp/logging.hpp"
#include "ros2_cuda_ipc_core/memory_types.hpp"

namespace ros2_cuda_ipc_core::backend {

namespace {

cudaIpcMemHandle_t to_cuda_mem_handle(
    const ros2_cuda_ipc_msgs::msg::BufferCore& msg) {
  cudaIpcMemHandle_t handle{};
  std::memcpy(&handle, msg.mem_handle.data(), sizeof(handle));
  return handle;
}

}  // namespace

std::optional<ImportedBuffer> CudaIpcImporter::import(
    const ros2_cuda_ipc_msgs::msg::BufferCore& msg,
    const cudaIpcEventHandle_t& event_handle,
    const rclcpp::Logger& logger) const {
  ImportedBuffer imported;

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

}  // namespace ros2_cuda_ipc_core::backend
