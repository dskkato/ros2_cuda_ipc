// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <cuda.h>
#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include "ros2_cuda_ipc_core/subscriber/buffer_view.hpp"

namespace {

TEST(ReadyEventDriverTest, DriverWaitAcceptsRuntimeCreatedStream) {
  int device_count = 0;
  if (cudaGetDeviceCount(&device_count) != cudaSuccess || device_count == 0) {
    GTEST_SKIP() << "CUDA device not available";
  }
  ASSERT_EQ(cudaSetDevice(0), cudaSuccess);

  cudaStream_t producer = nullptr;
  cudaStream_t consumer = nullptr;
  cudaEvent_t runtime_event = nullptr;
  ASSERT_EQ(cudaStreamCreate(&producer), cudaSuccess);
  ASSERT_EQ(cudaStreamCreate(&consumer), cudaSuccess);
  ASSERT_EQ(cudaEventCreateWithFlags(&runtime_event, cudaEventDisableTiming),
            cudaSuccess);
  ASSERT_EQ(cudaEventRecord(runtime_event, producer), cudaSuccess);

  ros2_cuda_ipc_core::subscriber::BufferView view;
  view.ready_evt = reinterpret_cast<CUevent>(runtime_event);
  const auto result = view.enqueue_ready_event(consumer);
  ASSERT_TRUE(result) << result.error().to_string();
  ASSERT_EQ(cudaStreamSynchronize(consumer), cudaSuccess);

  EXPECT_EQ(cudaEventDestroy(runtime_event), cudaSuccess);
  EXPECT_EQ(cudaStreamDestroy(consumer), cudaSuccess);
  EXPECT_EQ(cudaStreamDestroy(producer), cudaSuccess);
}

}  // namespace
