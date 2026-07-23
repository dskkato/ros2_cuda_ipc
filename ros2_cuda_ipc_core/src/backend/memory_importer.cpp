// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/backend/memory_importer.hpp"

#include <cstdint>

#include "rclcpp/logging.hpp"
#include "ros2_cuda_ipc_core/backend/cuda_ipc/memory_importer.hpp"
#include "ros2_cuda_ipc_core/backend/vmm_fd/memory_importer.hpp"
#include "ros2_cuda_ipc_core/transport/memory_types.hpp"

namespace ros2_cuda_ipc_core::backend {

void release_imported_memory(const ImportedMemory& imported) noexcept {
  // Imported resources created by the Driver-backed importers always carry
  // the context that owns them.  A missing context denotes a non-owning test
  // or inspection value; do not issue CUDA cleanup calls for such a value.
  if (!imported.context) {
    return;
  }
  std::optional<detail::CudaContextGuard> guard;
  auto guard_result = imported.context->push_current();
  if (!guard_result) {
    RCLCPP_ERROR(
        rclcpp::get_logger("ros2_cuda_ipc_core.memory_importer"),
        "Failed to activate CUDA context for imported resource cleanup: %s",
        guard_result.error().to_string().c_str());
    return;
  }
  guard.emplace(std::move(guard_result).value());
  if (imported.vmm_address != 0 && imported.allocation_size != 0) {
    cuMemUnmap(imported.vmm_address, imported.allocation_size);
    cuMemAddressFree(imported.vmm_address, imported.allocation_size);
  }
  if (imported.vmm_allocation != 0) {
    cuMemRelease(imported.vmm_allocation);
  }
  if (imported.vmm_address == 0 && imported.dev_ptr != nullptr) {
    const CUdeviceptr device_ptr =
        static_cast<CUdeviceptr>(reinterpret_cast<uintptr_t>(imported.dev_ptr));
    const CUresult result = cuIpcCloseMemHandle(device_ptr);
    if (result != CUDA_SUCCESS) {
      RCLCPP_ERROR(rclcpp::get_logger("ros2_cuda_ipc_core.memory_importer"),
                   "cuIpcCloseMemHandle failed during imported resource "
                   "cleanup: %s",
                   detail::CudaDriverError(result).to_string().c_str());
    }
  }
  if (imported.event != nullptr) {
    const CUresult result = cuEventDestroy(imported.event);
    if (result != CUDA_SUCCESS) {
      RCLCPP_ERROR(rclcpp::get_logger("ros2_cuda_ipc_core.memory_importer"),
                   "cuEventDestroy failed during imported resource cleanup: %s",
                   detail::CudaDriverError(result).to_string().c_str());
    }
  }
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
