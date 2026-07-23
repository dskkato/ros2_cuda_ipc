// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/backend/memory_importer.hpp"

#include <cstdint>

#include "rclcpp/logging.hpp"
#include "ros2_cuda_ipc_core/backend/cuda_ipc/memory_importer.hpp"
#include "ros2_cuda_ipc_core/backend/vmm_fd/memory_importer.hpp"
#include "ros2_cuda_ipc_core/transport/memory_types.hpp"

namespace ros2_cuda_ipc_core::backend {

bool release_imported_resources(const ImportedResources& imported) noexcept {
  // Imported resources created by the Driver-backed importers always carry
  // the context that owns them.  A missing context denotes a non-owning test
  // or inspection value; do not issue CUDA cleanup calls for such a value.
  if (!imported.context) {
    return true;
  }
  std::optional<detail::CudaContextGuard> guard;
  auto guard_result = imported.context->push_current();
  if (!guard_result) {
    RCLCPP_ERROR(
        rclcpp::get_logger("ros2_cuda_ipc_core.memory_importer"),
        "Failed to activate CUDA context for imported resource cleanup: %s",
        guard_result.error().to_string().c_str());
    return false;
  }
  guard.emplace(std::move(guard_result).value());
  bool success = true;
  const auto logger = rclcpp::get_logger("ros2_cuda_ipc_core.memory_importer");
  const auto report_cleanup_failure = [&success, &logger](const char* operation,
                                                          CUresult result) {
    if (result == CUDA_SUCCESS) {
      return;
    }
    success = false;
    RCLCPP_ERROR(logger, "%s failed during imported resource cleanup: %s",
                 operation,
                 detail::CudaDriverError(result).to_string().c_str());
  };
  if (imported.vmm_address != 0 && imported.allocation_size != 0) {
    report_cleanup_failure("cuMemUnmap", cuMemUnmap(imported.vmm_address,
                                                    imported.allocation_size));
    report_cleanup_failure(
        "cuMemAddressFree",
        cuMemAddressFree(imported.vmm_address, imported.allocation_size));
  }
  if (imported.vmm_allocation != 0) {
    report_cleanup_failure("cuMemRelease",
                           cuMemRelease(imported.vmm_allocation));
  }
  if (imported.vmm_address == 0 && imported.dev_ptr != nullptr) {
    const CUdeviceptr device_ptr =
        static_cast<CUdeviceptr>(reinterpret_cast<uintptr_t>(imported.dev_ptr));
    const CUresult result = cuIpcCloseMemHandle(device_ptr);
    if (result != CUDA_SUCCESS) {
      success = false;
      RCLCPP_ERROR(logger,
                   "cuIpcCloseMemHandle failed during imported resource "
                   "cleanup: %s",
                   detail::CudaDriverError(result).to_string().c_str());
    }
  }
  if (imported.event != nullptr) {
    const CUresult result = cuEventDestroy(imported.event);
    if (result != CUDA_SUCCESS) {
      success = false;
      RCLCPP_ERROR(logger,
                   "cuEventDestroy failed during imported resource cleanup: %s",
                   detail::CudaDriverError(result).to_string().c_str());
    }
  }
  return success;
}

const MemoryImporter& get_memory_importer(uint8_t backend) {
  static cuda_ipc::MemoryImporter cuda_ipc_importer;
  static vmm_fd::MemoryImporter vmm_fd_importer;

  switch (transport::backend_from_byte(backend)) {
    case transport::MemoryBackendKind::CUDA_IPC:
      return cuda_ipc_importer;
    case transport::MemoryBackendKind::VMM_FD:
      return vmm_fd_importer;
  }

  return cuda_ipc_importer;
}

}  // namespace ros2_cuda_ipc_core::backend
