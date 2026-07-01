// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/cuda/memory_importer.hpp"

#include "ros2_cuda_ipc_core/cuda/cuda_ipc/memory_importer.hpp"
#include "ros2_cuda_ipc_core/cuda/vmm_fd/memory_importer.hpp"
#include "ros2_cuda_ipc_core/memory_types.hpp"

namespace ros2_cuda_ipc_core::cuda {

void release_imported_memory(const ImportedMemory& imported) noexcept {
  if (imported.vmm_address != 0 && imported.allocation_size != 0) {
    cuMemUnmap(imported.vmm_address, imported.allocation_size);
    cuMemAddressFree(imported.vmm_address, imported.allocation_size);
  }
  if (imported.vmm_allocation != 0) {
    cuMemRelease(imported.vmm_allocation);
  }
  if (imported.vmm_address == 0 && imported.dev_ptr != nullptr) {
    cudaIpcCloseMemHandle(imported.dev_ptr);
  }
  if (imported.event != nullptr) {
    cudaEventDestroy(imported.event);
  }
}

const MemoryImporter& get_memory_importer(uint8_t backend) {
  static cuda_ipc::MemoryImporter cuda_ipc_importer;
  static vmm_fd::MemoryImporter vmm_fd_importer;

  switch (backend_from_byte(backend)) {
    case MemoryBackendKind::CUDA_IPC:
      return cuda_ipc_importer;
    case MemoryBackendKind::VMM_FD:
      return vmm_fd_importer;
  }

  return cuda_ipc_importer;
}

}  // namespace ros2_cuda_ipc_core::cuda
