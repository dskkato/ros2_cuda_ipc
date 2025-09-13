#include <cuda_runtime.h>

#include "ros2_cuda_ipc_core/cuda_support.hpp"

namespace ros2_cuda_ipc_core {

bool cuda_is_available() {
  int count = 0;
  auto err = cudaGetDeviceCount(&count);
  return (err == cudaSuccess) && (count > 0);
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

}  // namespace ros2_cuda_ipc_core
