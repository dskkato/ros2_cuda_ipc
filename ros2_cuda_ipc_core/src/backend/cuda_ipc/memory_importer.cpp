// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/backend/cuda_ipc/memory_importer.hpp"

#include <rcutils/logging_macros.h>

#include <cstdint>
#include <cstring>

#include "ros2_cuda_ipc_core/detail/cuda_driver_context.hpp"
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

std::optional<ImportedResources> MemoryImporter::import(
    const ros2_cuda_ipc_msgs::msg::BufferCore& msg,
    const CUipcEventHandle& event_handle) const {
  auto context_result = detail::CudaDeviceContext::retain_primary(
      static_cast<int>(msg.device_id));
  if (!context_result) {
    RCUTILS_LOG_WARN_NAMED("ros2_cuda_ipc_core.backend.cuda_ipc",
                           "Failed to retain CUDA primary context: %s",
                           context_result.error().to_string().c_str());
    return std::nullopt;
  }
  auto context = std::move(context_result).value();
  auto guard_result = context->push_current();
  if (!guard_result) {
    RCUTILS_LOG_WARN_NAMED("ros2_cuda_ipc_core.backend.cuda_ipc",
                           "Failed to activate CUDA context: %s",
                           guard_result.error().to_string().c_str());
    return std::nullopt;
  }
  auto guard = std::move(guard_result).value();

  ImportedResources imported;
  imported.context = context;

  CUresult result = cuIpcOpenEventHandle(&imported.event, event_handle);
  if (result != CUDA_SUCCESS) {
    RCUTILS_LOG_WARN_NAMED("ros2_cuda_ipc_core.backend.cuda_ipc",
                           "cuIpcOpenEventHandle failed: %s",
                           detail::CudaDriverError(result).to_string().c_str());
    return std::nullopt;
  }

  const CUipcMemHandle mem_handle = to_cuda_mem_handle(msg);
  CUdeviceptr device_ptr = 0;
  result = cuIpcOpenMemHandle(&device_ptr, mem_handle,
                              CU_IPC_MEM_LAZY_ENABLE_PEER_ACCESS);
  if (result != CUDA_SUCCESS) {
    RCUTILS_LOG_WARN_NAMED("ros2_cuda_ipc_core.backend.cuda_ipc",
                           "cuIpcOpenMemHandle failed: %s",
                           detail::CudaDriverError(result).to_string().c_str());
    (void)cuEventDestroy(imported.event);
    imported.event = nullptr;
    return std::nullopt;
  }
  imported.dev_ptr =
      reinterpret_cast<void*>(static_cast<uintptr_t>(device_ptr));

  return imported;
}

}  // namespace ros2_cuda_ipc_core::backend::cuda_ipc
