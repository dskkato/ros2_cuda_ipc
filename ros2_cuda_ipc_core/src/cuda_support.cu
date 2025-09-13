#include <cuda_runtime.h>

#include <cstring>

#include "ros2_cuda_ipc_core/cuda_support.hpp"

namespace ros2_cuda_ipc_core {

bool cuda_is_available() {
  int count = 0;
  auto err = cudaGetDeviceCount(&count);

  if (err == cudaSuccess) {
    return (count > 0);
  } else {
    return false;
  }
}

void* cuda_allocate(std::size_t bytes) {
  void* ptr = nullptr;
  auto err = cudaMalloc(&ptr, bytes);
  if (err != cudaSuccess) {
    return nullptr;
  }
  return ptr;
}

bool cuda_free(void* ptr) {
  if (!ptr) return false;
  auto err = cudaFree(ptr);
  return err == cudaSuccess;
}

bool cuda_ipc_get_mem_handle(void* device_ptr, CudaIpcMemHandle* out_handle) {
  if (!device_ptr || !out_handle) return false;
  cudaIpcMemHandle_t h{};
  auto err = cudaIpcGetMemHandle(&h, device_ptr);
  if (err != cudaSuccess) return false;
  static_assert(sizeof(CudaIpcMemHandle) == sizeof(cudaIpcMemHandle_t),
                "IPC handle size mismatch");
  std::memcpy(out_handle, &h, sizeof(h));
  return true;
}

void* cuda_ipc_open_mem_handle(const CudaIpcMemHandle& handle) {
  cudaIpcMemHandle_t h{};
  static_assert(sizeof(CudaIpcMemHandle) == sizeof(cudaIpcMemHandle_t),
                "IPC handle size mismatch");
  std::memcpy(&h, &handle, sizeof(h));
  void* ptr = nullptr;
  auto err = cudaIpcOpenMemHandle(&ptr, h, cudaIpcMemLazyEnablePeerAccess);
  if (err != cudaSuccess) return nullptr;
  return ptr;
}

bool cuda_ipc_close_mem_handle(void* device_ptr) {
  if (!device_ptr) return false;
  auto err = cudaIpcCloseMemHandle(device_ptr);
  return err == cudaSuccess;
}

}  // namespace ros2_cuda_ipc_core
