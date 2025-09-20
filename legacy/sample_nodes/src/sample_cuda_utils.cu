#include "sample_nodes/sample_cuda_utils.hpp"

namespace sample_nodes {

bool cuda_fill_u8(void* device_ptr, unsigned char value, std::size_t size_bytes,
                  cudaStream_t stream) {
  if (!device_ptr || size_bytes == 0) return false;
  auto err =
      cudaMemsetAsync(device_ptr, static_cast<int>(value), size_bytes, stream);
  if (err != cudaSuccess) return false;
  return true;
}

}  // namespace sample_nodes
