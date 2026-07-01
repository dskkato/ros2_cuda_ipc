// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <cuda_runtime_api.h>

#include "multi_process_image_fanout/kernels.hpp"

namespace multi_process_image_fanout {
namespace {

__global__ void generate_rgba_pattern_kernel(uint8_t* output, int width,
                                             int height, uint64_t stride_bytes,
                                             uint64_t frame_index) {
  const int x = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  const int y = static_cast<int>(blockIdx.y * blockDim.y + threadIdx.y);
  if (x >= width || y >= height) {
    return;
  }

  uint8_t* pixel = output + static_cast<uint64_t>(y) * stride_bytes +
                   static_cast<uint64_t>(x) * kDefaultChannels;
  const int block_x = x / 32;
  const int block_y = y / 32;

  pixel[0] = static_cast<uint8_t>((x + frame_index) & 0xffU);
  pixel[1] = static_cast<uint8_t>((y + 2 * frame_index) & 0xffU);
  pixel[2] = static_cast<uint8_t>(
      (((block_x ^ block_y) * 37) + 3 * frame_index) & 0xffU);
  pixel[3] = 255;
}

}  // namespace

cudaError_t launch_generate_rgba_pattern_kernel(uint8_t* output, int width,
                                                int height,
                                                uint64_t stride_bytes,
                                                uint64_t frame_index,
                                                cudaStream_t stream) {
  if (output == nullptr || width <= 0 || height <= 0 || stride_bytes == 0) {
    return cudaErrorInvalidValue;
  }

  constexpr dim3 block(16, 16);
  const dim3 grid((static_cast<unsigned int>(width) + block.x - 1) / block.x,
                  (static_cast<unsigned int>(height) + block.y - 1) / block.y);
  generate_rgba_pattern_kernel<<<grid, block, 0, stream>>>(
      output, width, height, stride_bytes, frame_index);
  return cudaGetLastError();
}

}  // namespace multi_process_image_fanout
