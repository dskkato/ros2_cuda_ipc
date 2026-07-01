// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <cuda_runtime_api.h>

#include <cstdint>
#include <limits>

#include "multi_process_image_fanout/kernels.hpp"

namespace multi_process_image_fanout {
namespace {

constexpr int kPatternBlockSize = 32;
constexpr unsigned int kPixelBlockX = 16;
constexpr unsigned int kPixelBlockY = 16;
constexpr unsigned int kGrayBlockX = 16;
constexpr unsigned int kGrayBlockY = 16;
constexpr unsigned int kChecksumBlockSize = 256;
constexpr unsigned int kStatsBlockSize = 256;

__device__ __forceinline__ uint8_t rgba_to_luma_u8(const uint8_t* pixel) {
  const uint32_t r = pixel[0];
  const uint32_t g = pixel[1];
  const uint32_t b = pixel[2];
  return static_cast<uint8_t>((77U * r + 150U * g + 29U * b) >> 8);
}

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
  const int block_x = x / kPatternBlockSize;
  const int block_y = y / kPatternBlockSize;

  pixel[0] = static_cast<uint8_t>((x + frame_index) & 0xffU);
  pixel[1] = static_cast<uint8_t>((y + 2 * frame_index) & 0xffU);
  pixel[2] = static_cast<uint8_t>(
      (((block_x ^ block_y) * 37) + 3 * frame_index) & 0xffU);
  pixel[3] = 255;
}

__global__ void rgba_to_luma_downscale2_kernel(const uint8_t* input_rgba,
                                               uint8_t* output_luma,
                                               int input_width,
                                               int input_height,
                                               uint64_t input_stride_bytes) {
  const int output_x = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  const int output_y = static_cast<int>(blockIdx.y * blockDim.y + threadIdx.y);
  const int output_width = input_width / 2;
  const int output_height = input_height / 2;
  if (output_x >= output_width || output_y >= output_height) {
    return;
  }

  const int input_x = output_x * 2;
  const int input_y = output_y * 2;
  if (input_x + 1 >= input_width || input_y + 1 >= input_height) {
    return;
  }

  const uint8_t* row0 =
      input_rgba + static_cast<uint64_t>(input_y) * input_stride_bytes;
  const uint8_t* row1 =
      input_rgba + static_cast<uint64_t>(input_y + 1) * input_stride_bytes;

  const uint8_t* p00 = row0 + static_cast<uint64_t>(input_x) * kDefaultChannels;
  const uint8_t* p01 =
      row0 + static_cast<uint64_t>(input_x + 1) * kDefaultChannels;
  const uint8_t* p10 = row1 + static_cast<uint64_t>(input_x) * kDefaultChannels;
  const uint8_t* p11 =
      row1 + static_cast<uint64_t>(input_x + 1) * kDefaultChannels;

  const uint32_t y00 = rgba_to_luma_u8(p00);
  const uint32_t y01 = rgba_to_luma_u8(p01);
  const uint32_t y10 = rgba_to_luma_u8(p10);
  const uint32_t y11 = rgba_to_luma_u8(p11);

  output_luma[static_cast<uint64_t>(output_y) * output_width + output_x] =
      static_cast<uint8_t>((y00 + y01 + y10 + y11) / 4U);
}

__global__ void checksum_u8_kernel(const uint8_t* input, size_t element_count,
                                   uint64_t* device_checksum) {
  const size_t index = static_cast<size_t>(blockIdx.x) * blockDim.x +
                       static_cast<size_t>(threadIdx.x);
  if (index >= element_count) {
    return;
  }

  const uint64_t weight = static_cast<uint64_t>((index % 251U) + 1U);
  atomicAdd(reinterpret_cast<unsigned long long*>(device_checksum),
            static_cast<unsigned long long>(input[index]) * weight);
}

__global__ void rgba_to_normalized_gray_kernel(const uint8_t* input_rgba,
                                               float* output_gray, int width,
                                               int height,
                                               uint64_t input_stride_bytes) {
  const int x = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  const int y = static_cast<int>(blockIdx.y * blockDim.y + threadIdx.y);
  if (x >= width || y >= height) {
    return;
  }

  const uint8_t* pixel = input_rgba +
                         static_cast<uint64_t>(y) * input_stride_bytes +
                         static_cast<uint64_t>(x) * kDefaultChannels;
  const uint8_t luma = rgba_to_luma_u8(pixel);
  output_gray[static_cast<uint64_t>(y) * width + x] =
      static_cast<float>(luma) / 255.0f;
}

struct DeviceInferenceStats {
  float mean;
  float min;
  float max;
  uint64_t checksum;
};

__global__ void inference_stats_accumulate_kernel(const float* input,
                                                  size_t element_count,
                                                  DeviceInferenceStats* stats) {
  const size_t index = static_cast<size_t>(blockIdx.x) * blockDim.x +
                       static_cast<size_t>(threadIdx.x);
  if (index >= element_count) {
    return;
  }

  const float value = input[index];
  atomicAdd(&stats->mean, value);
  atomicMin(reinterpret_cast<unsigned int*>(&stats->min),
            __float_as_uint(value));
  atomicMax(reinterpret_cast<unsigned int*>(&stats->max),
            __float_as_uint(value));
  const uint64_t quantized =
      static_cast<uint64_t>(static_cast<int>(__float2int_rn(value * 65535.0f)));
  const uint64_t weighted =
      quantized * static_cast<uint64_t>((index % 251U) + 1U);
  atomicAdd(reinterpret_cast<unsigned long long*>(&stats->checksum),
            static_cast<unsigned long long>(weighted));
}

__global__ void inference_stats_finalize_kernel(size_t element_count,
                                                DeviceInferenceStats* stats) {
  if (blockIdx.x != 0 || threadIdx.x != 0) {
    return;
  }

  if (element_count == 0) {
    stats->mean = 0.0f;
    stats->min = 0.0f;
    stats->max = 0.0f;
    stats->checksum = 0;
    return;
  }

  stats->mean /= static_cast<float>(element_count);
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

  const dim3 block(kPixelBlockX, kPixelBlockY);
  const dim3 grid((static_cast<unsigned int>(width) + block.x - 1) / block.x,
                  (static_cast<unsigned int>(height) + block.y - 1) / block.y);
  generate_rgba_pattern_kernel<<<grid, block, 0, stream>>>(
      output, width, height, stride_bytes, frame_index);
  return cudaGetLastError();
}

cudaError_t launch_rgba_to_luma_downscale2_kernel(
    const uint8_t* input_rgba, uint8_t* output_luma, int input_width,
    int input_height, uint64_t input_stride_bytes, cudaStream_t stream) {
  if (input_rgba == nullptr || output_luma == nullptr || input_width <= 0 ||
      input_height <= 0 || input_stride_bytes == 0) {
    return cudaErrorInvalidValue;
  }

  const int output_width = input_width / 2;
  const int output_height = input_height / 2;
  if (output_width <= 0 || output_height <= 0) {
    return cudaErrorInvalidValue;
  }

  const dim3 block(kGrayBlockX, kGrayBlockY);
  const dim3 grid(
      (static_cast<unsigned int>(output_width) + block.x - 1) / block.x,
      (static_cast<unsigned int>(output_height) + block.y - 1) / block.y);
  rgba_to_luma_downscale2_kernel<<<grid, block, 0, stream>>>(
      input_rgba, output_luma, input_width, input_height, input_stride_bytes);
  return cudaGetLastError();
}

cudaError_t launch_checksum_u8_kernel(const uint8_t* input,
                                      size_t element_count,
                                      uint64_t* device_checksum,
                                      cudaStream_t stream) {
  if (input == nullptr || device_checksum == nullptr || element_count == 0) {
    return cudaErrorInvalidValue;
  }

  const dim3 block(kChecksumBlockSize);
  const dim3 grid((static_cast<unsigned int>(element_count) + block.x - 1) /
                  block.x);
  checksum_u8_kernel<<<grid, block, 0, stream>>>(input, element_count,
                                                 device_checksum);
  return cudaGetLastError();
}

cudaError_t launch_rgba_to_normalized_gray_kernel(const uint8_t* input_rgba,
                                                  float* output_gray, int width,
                                                  int height,
                                                  uint64_t input_stride_bytes,
                                                  cudaStream_t stream) {
  if (input_rgba == nullptr || output_gray == nullptr || width <= 0 ||
      height <= 0 || input_stride_bytes == 0) {
    return cudaErrorInvalidValue;
  }

  const dim3 block(kGrayBlockX, kGrayBlockY);
  const dim3 grid((static_cast<unsigned int>(width) + block.x - 1) / block.x,
                  (static_cast<unsigned int>(height) + block.y - 1) / block.y);
  rgba_to_normalized_gray_kernel<<<grid, block, 0, stream>>>(
      input_rgba, output_gray, width, height, input_stride_bytes);
  return cudaGetLastError();
}

cudaError_t launch_inference_stats_kernel(const float* input,
                                          size_t element_count,
                                          InferenceStats* device_stats,
                                          cudaStream_t stream) {
  if (input == nullptr || device_stats == nullptr || element_count == 0) {
    return cudaErrorInvalidValue;
  }

  DeviceInferenceStats host_init{};
  host_init.mean = 0.0f;
  host_init.min = std::numeric_limits<float>::infinity();
  host_init.max = 0.0f;
  host_init.checksum = 0;

  cudaError_t err =
      cudaMemcpyAsync(device_stats, &host_init, sizeof(DeviceInferenceStats),
                      cudaMemcpyHostToDevice, stream);
  if (err != cudaSuccess) {
    return err;
  }

  const dim3 grid(
      (static_cast<unsigned int>(element_count) + kStatsBlockSize - 1) /
      kStatsBlockSize);
  inference_stats_accumulate_kernel<<<grid, kStatsBlockSize, 0, stream>>>(
      input, element_count,
      reinterpret_cast<DeviceInferenceStats*>(device_stats));
  err = cudaGetLastError();
  if (err != cudaSuccess) {
    return err;
  }

  inference_stats_finalize_kernel<<<1, 1, 0, stream>>>(
      element_count, reinterpret_cast<DeviceInferenceStats*>(device_stats));
  return cudaGetLastError();
}

}  // namespace multi_process_image_fanout
