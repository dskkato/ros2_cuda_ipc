// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/backend/memory_importer.hpp"

#include "ros2_cuda_ipc_core/backend/cuda_ipc/memory_importer.hpp"
#include "ros2_cuda_ipc_core/backend/vmm_fd/memory_importer.hpp"
#include "ros2_cuda_ipc_core/transport/memory_types.hpp"

namespace ros2_cuda_ipc_core::backend {

void release_imported_memory(const ImportedMemory& imported) noexcept {
  // VMM imports must be dismantled in mapping dependency order.
  if (imported.vmm_address != 0 && imported.allocation_size != 0) {
    (void)cuMemUnmap(imported.vmm_address, imported.allocation_size);
    (void)cuMemAddressFree(imported.vmm_address, imported.allocation_size);
  }
  if (imported.vmm_allocation != 0) {
    (void)cuMemRelease(imported.vmm_allocation);
  }
  // CUDA IPC imports own an event and memory mapping independently.
  if (imported.event != nullptr) {
    (void)cuEventDestroy(imported.event);
  }
  if (imported.vmm_address == 0 && imported.dev_ptr != 0) {
    (void)cuIpcCloseMemHandle(imported.dev_ptr);
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
