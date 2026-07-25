// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <cuda.h>
#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <memory>

#include "ros2_cuda_ipc_core/backend/memory_importer.hpp"
#include "ros2_cuda_ipc_core/detail/cuda_driver_context.hpp"
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

  auto imported =
      std::make_shared<ros2_cuda_ipc_core::backend::ImportedResources>();
  imported->event = reinterpret_cast<CUevent>(runtime_event);
  ros2_cuda_ipc_core::subscriber::BufferView view;
  view.set_imported_resource(std::move(imported));
  const auto result = view.enqueue_ready_event(consumer);
  ASSERT_TRUE(result) << result.error().to_string();
  ASSERT_EQ(cudaStreamSynchronize(consumer), cudaSuccess);

  EXPECT_EQ(cudaEventDestroy(runtime_event), cudaSuccess);
  EXPECT_EQ(cudaStreamDestroy(consumer), cudaSuccess);
  EXPECT_EQ(cudaStreamDestroy(producer), cudaSuccess);
}

TEST(ReadyEventDriverTest, RuntimeEventCanBeWaitedOnDriverCreatedStream) {
  int device_count = 0;
  if (cudaGetDeviceCount(&device_count) != cudaSuccess || device_count == 0) {
    GTEST_SKIP() << "CUDA device not available";
  }
  ASSERT_EQ(cudaSetDevice(0), cudaSuccess);

  cudaStream_t producer = nullptr;
  cudaEvent_t runtime_event = nullptr;
  ASSERT_EQ(cudaStreamCreate(&producer), cudaSuccess);
  ASSERT_EQ(cudaEventCreateWithFlags(&runtime_event, cudaEventDisableTiming),
            cudaSuccess);
  ASSERT_EQ(cudaEventRecord(runtime_event, producer), cudaSuccess);

  CUstream consumer = nullptr;
  ASSERT_EQ(cuStreamCreate(&consumer, CU_STREAM_DEFAULT), CUDA_SUCCESS);

  auto imported =
      std::make_shared<ros2_cuda_ipc_core::backend::ImportedResources>();
  imported->event = reinterpret_cast<CUevent>(runtime_event);
  ros2_cuda_ipc_core::subscriber::BufferView view;
  view.set_imported_resource(std::move(imported));
  const auto result = view.enqueue_ready_event(consumer);
  ASSERT_TRUE(result) << result.error().to_string();
  ASSERT_EQ(cuStreamSynchronize(consumer), CUDA_SUCCESS);

  EXPECT_EQ(cuStreamDestroy(consumer), CUDA_SUCCESS);
  EXPECT_EQ(cudaEventDestroy(runtime_event), cudaSuccess);
  EXPECT_EQ(cudaStreamDestroy(producer), cudaSuccess);
}

TEST(ReadyEventDriverTest, RejectsStreamFromAnotherCudaDevice) {
  int device_count = 0;
  if (cudaGetDeviceCount(&device_count) != cudaSuccess || device_count < 2) {
    GTEST_SKIP() << "At least two CUDA devices are required";
  }

  auto context_result =
      ros2_cuda_ipc_core::detail::CudaDeviceContext::retain_primary(0);
  if (!context_result) {
    GTEST_SKIP() << "CUDA primary context is unavailable: "
                 << context_result.error().to_string();
  }

  auto imported =
      std::make_shared<ros2_cuda_ipc_core::backend::ImportedResources>();
  imported->context = context_result.value();
  ros2_cuda_ipc_core::subscriber::BufferView view;
  view.set_imported_resource(std::move(imported));

  ASSERT_EQ(cudaSetDevice(1), cudaSuccess);
  cudaStream_t other_device_stream = nullptr;
  ASSERT_EQ(cudaStreamCreate(&other_device_stream), cudaSuccess);
  const auto result =
      view.validate_stream(reinterpret_cast<CUstream>(other_device_stream));
  EXPECT_FALSE(result);
  if (!result) {
    EXPECT_EQ(result.error().code(), CUDA_ERROR_INVALID_DEVICE);
  }
  EXPECT_EQ(cudaStreamDestroy(other_device_stream), cudaSuccess);
}

}  // namespace
