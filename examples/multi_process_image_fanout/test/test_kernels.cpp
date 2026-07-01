// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "multi_process_image_fanout/kernels.hpp"

namespace multi_process_image_fanout {
namespace {

constexpr int kWidth = 64;
constexpr int kHeight = 48;
constexpr uint64_t kStrideBytes = static_cast<uint64_t>(kWidth) * 4;
constexpr uint64_t kFrameIndex = 5;

uint8_t rgba_luma(const std::array<uint8_t, 4>& pixel) {
  return static_cast<uint8_t>(
      (77U * pixel[0] + 150U * pixel[1] + 29U * pixel[2]) >> 8);
}

std::array<uint8_t, 4> cpu_pattern_pixel(int x, int y, uint64_t frame_index) {
  const int block_x = x / 32;
  const int block_y = y / 32;
  return {static_cast<uint8_t>((x + frame_index) & 0xffU),
          static_cast<uint8_t>((y + 2 * frame_index) & 0xffU),
          static_cast<uint8_t>(
              ((((block_x ^ block_y) * 37) + 3 * frame_index) & 0xffU)),
          255};
}

uint8_t cpu_downscaled_luma(int x, int y, const std::vector<uint8_t>& rgba) {
  const int input_x = x * 2;
  const int input_y = y * 2;
  const auto pixel = [&](int px, int py) {
    const std::size_t offset = static_cast<std::size_t>(py) * kStrideBytes +
                               static_cast<std::size_t>(px) * 4;
    return std::array<uint8_t, 4>{rgba[offset + 0], rgba[offset + 1],
                                  rgba[offset + 2], rgba[offset + 3]};
  };

  const uint32_t y00 = rgba_luma(pixel(input_x, input_y));
  const uint32_t y01 = rgba_luma(pixel(input_x + 1, input_y));
  const uint32_t y10 = rgba_luma(pixel(input_x, input_y + 1));
  const uint32_t y11 = rgba_luma(pixel(input_x + 1, input_y + 1));
  return static_cast<uint8_t>((y00 + y01 + y10 + y11) / 4U);
}

uint64_t cpu_checksum_u8(const std::vector<uint8_t>& input) {
  uint64_t checksum = 0;
  for (std::size_t i = 0; i < input.size(); ++i) {
    checksum += static_cast<uint64_t>(input[i]) *
                static_cast<uint64_t>((i % 251U) + 1U);
  }
  return checksum;
}

InferenceStats cpu_inference_stats(const std::vector<float>& input) {
  InferenceStats stats;
  if (input.empty()) {
    return stats;
  }

  double sum = 0.0;
  float min_value = std::numeric_limits<float>::infinity();
  float max_value = 0.0f;
  uint64_t checksum = 0;
  for (std::size_t i = 0; i < input.size(); ++i) {
    const float value = input[i];
    sum += value;
    min_value = std::min(min_value, value);
    max_value = std::max(max_value, value);
    const int quantized =
        std::clamp(static_cast<int>(std::lround(value * 65535.0f)), 0, 65535);
    checksum += static_cast<uint64_t>(quantized) *
                static_cast<uint64_t>((i % 251U) + 1U);
  }

  stats.mean = static_cast<float>(sum / static_cast<double>(input.size()));
  stats.min = min_value;
  stats.max = max_value;
  stats.checksum = checksum;
  return stats;
}

TEST(KernelSmokeTest, ProducesExpectedOutputs) {
  int device_count = 0;
  const cudaError_t count_err = cudaGetDeviceCount(&device_count);
  if (count_err != cudaSuccess || device_count == 0) {
    GTEST_SKIP() << "CUDA device is not available in this environment";
  }

  const int output_width = kWidth / 2;
  const int output_height = kHeight / 2;
  const std::size_t rgba_bytes =
      static_cast<std::size_t>(kStrideBytes) * kHeight;
  const std::size_t luma_bytes =
      static_cast<std::size_t>(output_width) * output_height;
  const std::size_t gray_count = static_cast<std::size_t>(kWidth) * kHeight;

  uint8_t* device_rgba = nullptr;
  uint8_t* device_luma = nullptr;
  float* device_gray = nullptr;
  uint64_t* device_checksum = nullptr;
  InferenceStats* device_stats = nullptr;
  cudaStream_t stream = nullptr;

  ASSERT_EQ(cudaSuccess,
            cudaMalloc(reinterpret_cast<void**>(&device_rgba), rgba_bytes));
  ASSERT_EQ(cudaSuccess,
            cudaMalloc(reinterpret_cast<void**>(&device_luma), luma_bytes));
  ASSERT_EQ(cudaSuccess, cudaMalloc(reinterpret_cast<void**>(&device_gray),
                                    gray_count * sizeof(float)));
  ASSERT_EQ(cudaSuccess, cudaMalloc(reinterpret_cast<void**>(&device_checksum),
                                    sizeof(uint64_t)));
  ASSERT_EQ(cudaSuccess, cudaMalloc(reinterpret_cast<void**>(&device_stats),
                                    sizeof(InferenceStats)));
  ASSERT_EQ(cudaSuccess,
            cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking));

  ASSERT_EQ(cudaSuccess, launch_generate_rgba_pattern_kernel(
                             device_rgba, kWidth, kHeight, kStrideBytes,
                             kFrameIndex, stream));
  ASSERT_EQ(cudaSuccess, launch_rgba_to_luma_downscale2_kernel(
                             device_rgba, device_luma, kWidth, kHeight,
                             kStrideBytes, stream));
  ASSERT_EQ(cudaSuccess,
            cudaMemsetAsync(device_checksum, 0, sizeof(uint64_t), stream));
  ASSERT_EQ(cudaSuccess, launch_checksum_u8_kernel(device_luma, luma_bytes,
                                                   device_checksum, stream));
  ASSERT_EQ(cudaSuccess, launch_rgba_to_normalized_gray_kernel(
                             device_rgba, device_gray, kWidth, kHeight,
                             kStrideBytes, stream));
  ASSERT_EQ(cudaSuccess, launch_inference_stats_kernel(device_gray, gray_count,
                                                       device_stats, stream));
  ASSERT_EQ(cudaSuccess, cudaStreamSynchronize(stream));

  std::vector<uint8_t> host_rgba(rgba_bytes);
  std::vector<uint8_t> host_luma(luma_bytes);
  std::vector<float> host_gray(gray_count);
  uint64_t host_checksum = 0;
  InferenceStats host_stats;

  ASSERT_EQ(cudaSuccess, cudaMemcpy(host_rgba.data(), device_rgba, rgba_bytes,
                                    cudaMemcpyDeviceToHost));
  ASSERT_EQ(cudaSuccess, cudaMemcpy(host_luma.data(), device_luma, luma_bytes,
                                    cudaMemcpyDeviceToHost));
  ASSERT_EQ(cudaSuccess,
            cudaMemcpy(host_gray.data(), device_gray,
                       gray_count * sizeof(float), cudaMemcpyDeviceToHost));
  ASSERT_EQ(cudaSuccess, cudaMemcpy(&host_checksum, device_checksum,
                                    sizeof(uint64_t), cudaMemcpyDeviceToHost));
  ASSERT_EQ(cudaSuccess,
            cudaMemcpy(&host_stats, device_stats, sizeof(InferenceStats),
                       cudaMemcpyDeviceToHost));

  const auto pixel = [&](int x, int y) {
    const std::size_t offset = static_cast<std::size_t>(y) * kStrideBytes +
                               static_cast<std::size_t>(x) * 4;
    return std::array<uint8_t, 4>{host_rgba[offset + 0], host_rgba[offset + 1],
                                  host_rgba[offset + 2], host_rgba[offset + 3]};
  };

  EXPECT_EQ(pixel(0, 0), cpu_pattern_pixel(0, 0, kFrameIndex));
  EXPECT_EQ(pixel(33, 0), cpu_pattern_pixel(33, 0, kFrameIndex));
  EXPECT_EQ(pixel(0, 31), cpu_pattern_pixel(0, 31, kFrameIndex));

  for (int y = 0; y < output_height; ++y) {
    for (int x = 0; x < output_width; ++x) {
      const std::size_t offset = static_cast<std::size_t>(y) * output_width + x;
      EXPECT_EQ(host_luma[offset], cpu_downscaled_luma(x, y, host_rgba));
    }
  }

  EXPECT_EQ(host_checksum, cpu_checksum_u8(host_luma));

  std::vector<float> cpu_gray(gray_count);
  for (int y = 0; y < kHeight; ++y) {
    for (int x = 0; x < kWidth; ++x) {
      const std::size_t offset = static_cast<std::size_t>(y) * kWidth + x;
      cpu_gray[offset] = static_cast<float>(rgba_luma(pixel(x, y))) / 255.0f;
      EXPECT_NEAR(host_gray[offset], cpu_gray[offset], 1e-6f);
    }
  }

  const InferenceStats cpu_stats = cpu_inference_stats(cpu_gray);
  EXPECT_NEAR(host_stats.mean, cpu_stats.mean, 1e-6f);
  EXPECT_NEAR(host_stats.min, cpu_stats.min, 1e-6f);
  EXPECT_NEAR(host_stats.max, cpu_stats.max, 1e-6f);
  EXPECT_EQ(host_stats.checksum, cpu_stats.checksum);

  ASSERT_EQ(cudaSuccess, cudaStreamDestroy(stream));
  ASSERT_EQ(cudaSuccess, cudaFree(device_rgba));
  ASSERT_EQ(cudaSuccess, cudaFree(device_luma));
  ASSERT_EQ(cudaSuccess, cudaFree(device_gray));
  ASSERT_EQ(cudaSuccess, cudaFree(device_checksum));
  ASSERT_EQ(cudaSuccess, cudaFree(device_stats));
}

}  // namespace
}  // namespace multi_process_image_fanout
