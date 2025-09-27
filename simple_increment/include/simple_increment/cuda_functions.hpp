#pragma once

#include <cuda_runtime_api.h>

#include <cstdint>

namespace simple_increment {

/// Launch a kernel that reads from `source`, increments each byte, and writes
/// to `destination`.
cudaError_t compute_increment(int size_in_bytes, const uint8_t *source,
                              uint8_t *destination,
                              cudaStream_t stream) noexcept;

/// Launch an in-place increment kernel operating on `image`.
cudaError_t compute_increment_inplace(int size_in_bytes, uint8_t *image,
                                      cudaStream_t stream) noexcept;

}  // namespace simple_increment
