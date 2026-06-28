#include "ros2_cuda_ipc_core/backend/backend_importer.hpp"

#include "ros2_cuda_ipc_core/backend/cuda_ipc_importer.hpp"
#include "ros2_cuda_ipc_core/backend/vmm_fd_importer.hpp"
#include "ros2_cuda_ipc_core/memory_types.hpp"

namespace ros2_cuda_ipc_core::backend {

void release_imported_buffer(const ImportedBuffer& imported) noexcept {
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

const BackendImporter& get_backend_importer(uint8_t backend) {
  static CudaIpcImporter cuda_ipc_importer;
  static VmmFdImporter vmm_fd_importer;

  switch (backend_from_byte(backend)) {
    case MemoryBackendKind::CUDA_IPC:
      return cuda_ipc_importer;
    case MemoryBackendKind::VMM_FD:
      return vmm_fd_importer;
  }

  return cuda_ipc_importer;
}

}  // namespace ros2_cuda_ipc_core::backend
