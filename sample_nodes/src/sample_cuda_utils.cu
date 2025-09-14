#include <cuda_runtime.h>

#include "ros2_cuda_ipc_core/cuda_support.hpp"

namespace sample_nodes {

bool cuda_fill_u8(void* device_ptr, unsigned char value, std::size_t size_bytes,
                  ros2_cuda_ipc_core::cudaStream_t stream) {
  if (!device_ptr || size_bytes == 0) return false;
  ::cudaStream_t s = stream ? reinterpret_cast<::cudaStream_t>(stream)
                            : static_cast<::cudaStream_t>(0);
  auto err =
      cudaMemsetAsync(device_ptr, static_cast<int>(value), size_bytes, s);
  if (err != cudaSuccess) return false;
  return true;
}

}  // namespace sample_nodes
