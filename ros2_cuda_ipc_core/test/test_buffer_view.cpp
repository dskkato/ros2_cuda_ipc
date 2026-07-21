// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <cuda.h>
#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include "ros2_cuda_ipc_core/image/image_view.hpp"
#include "ros2_cuda_ipc_core/pointcloud2/pointcloud2_view.hpp"
#include "ros2_cuda_ipc_core/subscriber/buffer_view.hpp"

namespace {

class BufferViewTest : public ::testing::Test {
 protected:
  void SetUp() override {
    int count = 0;
    if (cudaGetDeviceCount(&count) != cudaSuccess || count == 0 ||
        cudaSetDevice(0) != cudaSuccess) {
      GTEST_SKIP() << "CUDA device not available";
    }
  }
};

TEST_F(BufferViewTest, DriverWaitAcceptsRuntimeCreatedStream) {
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
  EXPECT_EQ(view.enqueue_ready_event(consumer), CUDA_SUCCESS);
  EXPECT_EQ(cudaStreamSynchronize(consumer), cudaSuccess);

  EXPECT_EQ(cudaEventDestroy(runtime_event), cudaSuccess);
  EXPECT_EQ(cudaStreamDestroy(consumer), cudaSuccess);
  EXPECT_EQ(cudaStreamDestroy(producer), cudaSuccess);
}

TEST_F(BufferViewTest, ForwardingViewsReturnDriverResults) {
  cudaStream_t stream = nullptr;
  ASSERT_EQ(cudaStreamCreate(&stream), cudaSuccess);

  ros2_cuda_ipc_core::image::ImageView image;
  ros2_cuda_ipc_core::pointcloud2::PointCloud2View pointcloud;
  EXPECT_EQ(image.enqueue_ready_event(stream), CUDA_SUCCESS);
  EXPECT_EQ(pointcloud.enqueue_ready_event(stream), CUDA_SUCCESS);

  EXPECT_EQ(cudaStreamDestroy(stream), cudaSuccess);
}

}  // namespace
