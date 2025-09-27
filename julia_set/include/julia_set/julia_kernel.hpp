#pragma once

#include <cuda_runtime_api.h>

#include <cstdint>

namespace julia_set {

cudaError_t launch_julia_kernel(uint8_t *data, uint32_t width, uint32_t height,
                                uint32_t channels, float zoom, float offset_x,
                                float offset_y, float c_real, float c_imag,
                                uint32_t max_iterations, cudaStream_t stream);

}  // namespace julia_set
