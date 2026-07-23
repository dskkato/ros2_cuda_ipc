// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/backend/memory_importer.hpp"

#include <cstdint>

#include "ros2_cuda_ipc_core/backend/cuda_ipc/memory_importer.hpp"
#include "ros2_cuda_ipc_core/backend/vmm_fd/memory_importer.hpp"
#include "ros2_cuda_ipc_core/detail/cuda_driver_context.hpp"
#include "ros2_cuda_ipc_core/detail/cuda_util.hpp"
#include "ros2_cuda_ipc_core/transport/memory_types.hpp"

namespace ros2_cuda_ipc_core::backend {

void release_imported_memory(const ImportedMemory& imported) noexcept {
  if (!imported.driver_owned) {
    return;
  }
  detail::ScopedPrimaryContext context(imported.device_id);
  if (!context.ok()) {
    return;
  }
  if (imported.vmm_address != 0 && imported.allocation_size != 0) {
    (void)cuMemUnmap(imported.vmm_address, imported.allocation_size);
    (void)cuMemAddressFree(imported.vmm_address, imported.allocation_size);
  }
  if (imported.vmm_allocation != 0) {
    (void)cuMemRelease(imported.vmm_allocation);
  }
  if (imported.vmm_address == 0 && imported.dev_ptr != nullptr) {
    (void)cuIpcCloseMemHandle(static_cast<CUdeviceptr>(
        reinterpret_cast<uintptr_t>(imported.dev_ptr)));
  }
  if (imported.event != nullptr) {
    (void)cuEventDestroy(reinterpret_cast<CUevent>(imported.event));
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
