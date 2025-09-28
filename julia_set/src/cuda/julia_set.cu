#include <cmath>

#include "julia_set/cuda/julia_kernel.hpp"

namespace julia_set {
namespace {

__global__ void julia_kernel(uint8_t *data, uint32_t width, uint32_t height,
                             float zoom, float offset_x, float offset_y,
                             float c_real, float c_imag,
                             uint32_t max_iterations) {
  const uint32_t x = blockIdx.x * blockDim.x + threadIdx.x;
  const uint32_t y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= width || y >= height) {
    return;
  }

  const float jx =
      zoom * (static_cast<float>(x) / static_cast<float>(width) - 0.5f) +
      offset_x;
  const float jy =
      zoom * (static_cast<float>(y) / static_cast<float>(height) - 0.5f) +
      offset_y;

  float zx = jx;
  float zy = jy;
  uint32_t iteration = 0;
  while (zx * zx + zy * zy < 4.0f && iteration < max_iterations) {
    const float temp = zx * zx - zy * zy + c_real;
    zy = 2.0f * zx * zy + c_imag;
    zx = temp;
    ++iteration;
  }

  const float t =
      fminf(1.0f, fmaxf(0.0f, static_cast<float>(iteration) /
                                  static_cast<float>(max_iterations)));
  const uint32_t idx = y * width + x;
  data[idx] = static_cast<uint8_t>(t * 255.0f + 0.5f);
}

__global__ void colorize_kernel(const uint8_t *mono, uint8_t *rgb,
                                uint32_t width, uint32_t height,
                                uint32_t channels) {
  const uint32_t x = blockIdx.x * blockDim.x + threadIdx.x;
  const uint32_t y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= width || y >= height) {
    return;
  }

  const uint32_t mono_idx = y * width + x;
  const float t = static_cast<float>(mono[mono_idx]) / 255.0f;
  const float one_minus_t = 1.0f - t;

  const uint8_t r =
      static_cast<uint8_t>(255.0f * 9.0f * one_minus_t * t * t * t + 0.5f);
  const uint8_t g = static_cast<uint8_t>(
      255.0f * 15.0f * one_minus_t * one_minus_t * t * t + 0.5f);
  const uint8_t b = static_cast<uint8_t>(
      255.0f * 8.5f * one_minus_t * one_minus_t * one_minus_t * t + 0.5f);

  const uint32_t rgb_idx = (y * width + x) * channels;
  if (channels >= 3) {
    rgb[rgb_idx + 0] = r;
    rgb[rgb_idx + 1] = g;
    rgb[rgb_idx + 2] = b;
    if (channels > 3) {
      rgb[rgb_idx + 3] = 255;
    }
  }
}

}  // namespace

cudaError_t launch_julia_kernel(uint8_t *data, uint32_t width, uint32_t height,
                                float zoom, float offset_x, float offset_y,
                                float c_real, float c_imag,
                                uint32_t max_iterations, cudaStream_t stream) {
  const dim3 block_dim(16, 16);
  const dim3 grid_dim((width + block_dim.x - 1) / block_dim.x,
                      (height + block_dim.y - 1) / block_dim.y);
  julia_kernel<<<grid_dim, block_dim, 0, stream>>>(data, width, height, zoom,
                                                   offset_x, offset_y, c_real,
                                                   c_imag, max_iterations);
  return cudaGetLastError();
}

cudaError_t launch_colorize_kernel(const uint8_t *mono, uint8_t *rgb,
                                   uint32_t width, uint32_t height,
                                   uint32_t channels, cudaStream_t stream) {
  const dim3 block_dim(16, 16);
  const dim3 grid_dim((width + block_dim.x - 1) / block_dim.x,
                      (height + block_dim.y - 1) / block_dim.y);
  colorize_kernel<<<grid_dim, block_dim, 0, stream>>>(mono, rgb, width, height,
                                                      channels);
  return cudaGetLastError();
}

}  // namespace julia_set
