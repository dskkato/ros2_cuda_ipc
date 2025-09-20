// Simple CUDA demo helpers for sample_nodes (optional CUDA)
#ifndef SAMPLE_NODES_SAMPLE_CUDA_UTILS_HPP_
#define SAMPLE_NODES_SAMPLE_CUDA_UTILS_HPP_

#include <cuda_runtime_api.h>

#include <cstddef>

namespace sample_nodes {

// Fill a device buffer with a byte pattern asynchronously on a stream.
// - device_ptr: CUDA device pointer to buffer
// - value: byte pattern to fill (e.g., 0xAB)
// - size_bytes: number of bytes to fill
// - stream: CUDA stream handle (nullptr for default stream)
// Returns true on success.
bool cuda_fill_u8(void* device_ptr, unsigned char value, std::size_t size_bytes,
                  cudaStream_t stream);

}  // namespace sample_nodes

#endif  // SAMPLE_NODES_SAMPLE_CUDA_UTILS_HPP_
