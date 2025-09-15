// More comprehensive tests for cuda_support

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include "ros2_cuda_ipc_core/cuda_support.hpp"

using namespace ros2_cuda_ipc_core;

TEST(CudaSupport, Availability) {
  if (!cuda_is_available()) {
    GTEST_SKIP() << "CUDA device not available";
  }
  EXPECT_TRUE(cuda_is_available());
}

TEST(CudaSupport, NullArgumentValidation) {
  // These must be safe no-ops regardless of device availability
  EXPECT_FALSE(cuda_free(nullptr));
  EXPECT_FALSE(cuda_ipc_get_mem_handle(nullptr, nullptr));

  CudaIpcMemHandle mem{};
  EXPECT_FALSE(cuda_ipc_get_mem_handle(nullptr, &mem));

  EXPECT_FALSE(cuda_ipc_close_mem_handle(nullptr));

  EXPECT_FALSE(cuda_event_destroy(nullptr));
  EXPECT_FALSE(cuda_event_record(nullptr));

  CudaIpcEventHandle evt{};
  EXPECT_FALSE(cuda_event_get_ipc_handle(nullptr, &evt));
  EXPECT_FALSE(cuda_event_get_ipc_handle(cudaEvent_t{}, nullptr));
  EXPECT_FALSE(cuda_event_query(nullptr));
  EXPECT_FALSE(cuda_event_record_on_stream(nullptr, nullptr));

  EXPECT_FALSE(cuda_stream_destroy(nullptr));
  EXPECT_FALSE(cuda_stream_wait_event(nullptr, nullptr));
  EXPECT_FALSE(cuda_stream_synchronize(nullptr));

  // Handle struct sizes are ABI-stable wrappers (64 bytes)
  EXPECT_EQ(sizeof(CudaIpcMemHandle), static_cast<size_t>(64));
  EXPECT_EQ(sizeof(CudaIpcEventHandle), static_cast<size_t>(64));
}

TEST(CudaSupport, ErrorMessageIncludesCudaString) {
  if (!cuda_is_available()) {
    GTEST_SKIP() << "CUDA device not available";
  }

  const cudaEvent_t bad_evt = reinterpret_cast<cudaEvent_t>(1);
  auto err = cudaEventDestroy(bad_evt);
  const char* err_str = cudaGetErrorString(err);
  EXPECT_NE(err, cudaSuccess);

  try {
    cuda_event_destroy(bad_evt);
    FAIL() << "Expected std::runtime_error";
  } catch (const std::runtime_error& e) {
    EXPECT_NE(std::string::npos, std::string(e.what()).find(err_str));
  }
}

TEST(CudaSupport, AllocateAndFree) {
  if (!cuda_is_available()) {
    GTEST_SKIP() << "CUDA device not available";
  }
  void* p = nullptr;
  ASSERT_NO_THROW({ p = cuda_allocate(1024); });
  ASSERT_NE(p, nullptr);
  EXPECT_TRUE(cuda_free(p));
}

TEST(CudaSupport, EventAndStreamBasic) {
  if (!cuda_is_available()) {
    GTEST_SKIP() << "CUDA device not available";
  }

  cudaEvent_t evt = nullptr;
  ASSERT_NO_THROW({ evt = cuda_event_create(); });
  ASSERT_NE(evt, nullptr);

  cudaStream_t stream = nullptr;
  ASSERT_NO_THROW({ stream = cuda_stream_create(); });
  ASSERT_NE(stream, nullptr);

  // Record on a dedicated stream and then synchronize to ensure completion
  EXPECT_TRUE(cuda_event_record_on_stream(evt, stream));
  EXPECT_TRUE(cuda_stream_synchronize(stream));
  EXPECT_TRUE(cuda_event_query(evt));

  EXPECT_TRUE(cuda_stream_destroy(stream));
  EXPECT_TRUE(cuda_event_destroy(evt));
}

TEST(CudaSupport, DeviceIdString) {
  if (!cuda_is_available()) {
    GTEST_SKIP() << "CUDA device not available";
  }
  std::string id;
  ASSERT_NO_THROW({ id = cuda_get_device_id_string(); });
  EXPECT_FALSE(id.empty());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
