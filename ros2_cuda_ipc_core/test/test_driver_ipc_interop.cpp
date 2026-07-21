// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <cuda.h>
#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <cstring>

#include "ros2_cuda_ipc_core/detail/primary_context_guard.hpp"
#include "ros2_cuda_ipc_core/subscriber/buffer_view.hpp"

namespace ros2_cuda_ipc_core {
namespace {

class DriverIpcInteropTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (cuInit(0) != CUDA_SUCCESS) GTEST_SKIP() << "CUDA driver is unavailable";
    int count = 0;
    if (cuDeviceGetCount(&count) != CUDA_SUCCESS || count == 0)
      GTEST_SKIP() << "No CUDA device is available";
    if (cudaSetDevice(0) != cudaSuccess)
      GTEST_SKIP() << "CUDA Runtime cannot select device 0";
  }
};

TEST_F(DriverIpcInteropTest, PrimaryContextGuardRestoresCurrentContext) {
  CUcontext before = nullptr;
  ASSERT_EQ(cuCtxGetCurrent(&before), CUDA_SUCCESS);
  {
    detail::PrimaryContextGuard guard(0);
    ASSERT_TRUE(guard.ok());
  }
  CUcontext after = nullptr;
  ASSERT_EQ(cuCtxGetCurrent(&after), CUDA_SUCCESS);
  EXPECT_EQ(after, before);
}

TEST_F(DriverIpcInteropTest, RuntimeAndDriverShareThePrimaryContext) {
  // cudaFree(0) creates/attaches the Runtime API context without allocating.
  ASSERT_EQ(cudaFree(nullptr), cudaSuccess);
  CUcontext runtime_context = nullptr;
  ASSERT_EQ(cuCtxGetCurrent(&runtime_context), CUDA_SUCCESS);
  CUdevice device = 0;
  ASSERT_EQ(cuDeviceGet(&device, 0), CUDA_SUCCESS);
  CUcontext primary = nullptr;
  ASSERT_EQ(cuDevicePrimaryCtxRetain(&primary, device), CUDA_SUCCESS);
  EXPECT_EQ(runtime_context, primary);
  EXPECT_EQ(cuDevicePrimaryCtxRelease(device), CUDA_SUCCESS);
}

TEST_F(DriverIpcInteropTest, DriverImportsAndClosesIpcHandles) {
  void* allocation = nullptr;
  ASSERT_EQ(cudaMalloc(&allocation, 64), cudaSuccess);
  CUipcMemHandle memory_handle{};
  cudaIpcMemHandle_t runtime_memory_handle{};
  ASSERT_EQ(cudaIpcGetMemHandle(&runtime_memory_handle, allocation),
            cudaSuccess);
  std::memcpy(&memory_handle, &runtime_memory_handle, sizeof(memory_handle));
  cudaEvent_t runtime_event = nullptr;
  ASSERT_EQ(cudaEventCreateWithFlags(&runtime_event, cudaEventInterprocess),
            cudaSuccess);
  cudaIpcEventHandle_t runtime_event_handle{};
  ASSERT_EQ(cudaIpcGetEventHandle(&runtime_event_handle, runtime_event),
            cudaSuccess);
  CUipcEventHandle event_handle{};
  std::memcpy(&event_handle, &runtime_event_handle, sizeof(event_handle));

  detail::PrimaryContextGuard context(0);
  ASSERT_TRUE(context.ok());
  CUevent imported_event = nullptr;
  ASSERT_EQ(cuIpcOpenEventHandle(&imported_event, event_handle), CUDA_SUCCESS);
  CUdeviceptr imported_memory = 0;
  ASSERT_EQ(cuIpcOpenMemHandle(&imported_memory, memory_handle,
                               CU_IPC_MEM_LAZY_ENABLE_PEER_ACCESS),
            CUDA_SUCCESS);
  EXPECT_EQ(cuIpcCloseMemHandle(imported_memory), CUDA_SUCCESS);
  EXPECT_EQ(cuEventDestroy(imported_event), CUDA_SUCCESS);
  ASSERT_EQ(cudaEventDestroy(runtime_event), cudaSuccess);
  ASSERT_EQ(cudaFree(allocation), cudaSuccess);
}

TEST_F(DriverIpcInteropTest,
       RuntimeStreamWaitsForDriverImportedPublisherEvent) {
  cudaEvent_t publisher_event = nullptr;
  ASSERT_EQ(cudaEventCreateWithFlags(&publisher_event, cudaEventInterprocess),
            cudaSuccess);
  ASSERT_EQ(cudaEventRecord(publisher_event, nullptr), cudaSuccess);
  cudaIpcEventHandle_t runtime_handle{};
  ASSERT_EQ(cudaIpcGetEventHandle(&runtime_handle, publisher_event),
            cudaSuccess);
  CUipcEventHandle handle{};
  std::memcpy(&handle, &runtime_handle, sizeof(handle));
  detail::PrimaryContextGuard context(0);
  ASSERT_TRUE(context.ok());
  CUevent imported = nullptr;
  ASSERT_EQ(cuIpcOpenEventHandle(&imported, handle), CUDA_SUCCESS);
  subscriber::BufferView view;
  view.ready_evt = imported;
  cudaStream_t stream = nullptr;
  ASSERT_EQ(cudaStreamCreate(&stream), cudaSuccess);
  EXPECT_EQ(view.enqueue_ready_event(stream), cudaSuccess);
  EXPECT_EQ(cudaStreamSynchronize(stream), cudaSuccess);
  EXPECT_EQ(cuEventDestroy(imported), CUDA_SUCCESS);
  EXPECT_EQ(cudaStreamDestroy(stream), cudaSuccess);
  EXPECT_EQ(cudaEventDestroy(publisher_event), cudaSuccess);
}

}  // namespace
}  // namespace ros2_cuda_ipc_core
