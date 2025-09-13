// Fallback (non-CUDA) implementations
#include "ros2_cuda_ipc_core/cuda_support.hpp"

namespace ros2_cuda_ipc_core {

bool cuda_is_available() { return false; }

void* cuda_allocate(std::size_t /*bytes*/) { return nullptr; }

bool cuda_free(void* /*ptr*/) { return false; }

bool cuda_ipc_get_mem_handle(void* /*device_ptr*/,
                             CudaIpcMemHandle* /*out_handle*/) {
  return false;
}

void* cuda_ipc_open_mem_handle(const CudaIpcMemHandle& /*handle*/) {
  return nullptr;
}

bool cuda_ipc_close_mem_handle(void* /*device_ptr*/) { return false; }

}  // namespace ros2_cuda_ipc_core
