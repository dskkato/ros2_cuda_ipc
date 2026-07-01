// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <vector>

#include "multi_process_image_fanout/kernels.hpp"

namespace multi_process_image_fanout {
namespace {

TEST(GenerateRgbaPatternKernelTest, ProducesExpectedBytes) {
  constexpr int kWidth = 64;
  constexpr int kHeight = 32;
  constexpr uint64_t kStrideBytes = static_cast<uint64_t>(kWidth) * 4;
  constexpr uint64_t kFrameIndex = 5;

  int device_count = 0;
  const cudaError_t count_err = cudaGetDeviceCount(&device_count);
  if (count_err != cudaSuccess || device_count == 0) {
    GTEST_SKIP() << "CUDA device is not available in this environment";
  }

  uint8_t* device_buffer = nullptr;
  ASSERT_EQ(cudaSuccess, cudaMalloc(reinterpret_cast<void**>(&device_buffer),
                                    kStrideBytes * kHeight));

  cudaStream_t stream = nullptr;
  ASSERT_EQ(cudaSuccess,
            cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking));

  ASSERT_EQ(cudaSuccess, launch_generate_rgba_pattern_kernel(
                             device_buffer, kWidth, kHeight, kStrideBytes,
                             kFrameIndex, stream));
  ASSERT_EQ(cudaSuccess, cudaStreamSynchronize(stream));

  std::vector<uint8_t> host_buffer(kStrideBytes * kHeight);
  ASSERT_EQ(cudaSuccess,
            cudaMemcpy(host_buffer.data(), device_buffer, host_buffer.size(),
                       cudaMemcpyDeviceToHost));

  const auto pixel = [&](int x, int y) {
    const std::size_t offset = static_cast<std::size_t>(y) * kStrideBytes +
                               static_cast<std::size_t>(x) * 4;
    return std::array<uint8_t, 4>{
        host_buffer[offset + 0], host_buffer[offset + 1],
        host_buffer[offset + 2], host_buffer[offset + 3]};
  };

  EXPECT_EQ(pixel(0, 0), (std::array<uint8_t, 4>{5, 10, 15, 255}));
  EXPECT_EQ(pixel(33, 0), (std::array<uint8_t, 4>{38, 10, 52, 255}));
  EXPECT_EQ(pixel(0, 31), (std::array<uint8_t, 4>{5, 41, 15, 255}));

  EXPECT_EQ(cudaSuccess, cudaStreamDestroy(stream));
  EXPECT_EQ(cudaSuccess, cudaFree(device_buffer));
}

}  // namespace
}  // namespace multi_process_image_fanout
