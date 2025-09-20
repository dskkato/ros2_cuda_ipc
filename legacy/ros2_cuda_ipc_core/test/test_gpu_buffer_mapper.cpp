#include <gtest/gtest.h>

#include "ros2_cuda_ipc_core/cuda_support.hpp"
#include "ros2_cuda_ipc_core/gpu_buffer_mapper.hpp"

using namespace ros2_cuda_ipc_core;

TEST(GpuBufferMapperTest, EventOpenCacheAndWait) {
  if (!cuda_is_available()) {
    GTEST_SKIP() << "CUDA device not available";
  }

  // Create an interprocess-capable event in this process (exporter side)
  cudaEvent_t exporter_evt = cuda_event_create();
  ASSERT_NE(exporter_evt, nullptr);

  CudaIpcEventHandle evt_handle{};
  ASSERT_TRUE(cuda_event_get_ipc_handle(exporter_evt, &evt_handle));

  GpuBufferMapper mapper;
  const uint32_t slot_id = 3;

  // Open the event via IPC (consumer side) and check caching behavior
  cudaEvent_t opened_evt_1 = mapper.open_event(slot_id, evt_handle);
  if (opened_evt_1 == nullptr) {
    // Some driver/runtime combinations may not permit opening our own IPC
    // event handle within the same process. Treat as an environment skip.
    GTEST_SKIP() << "cudaIpcOpenEventHandle returned nullptr in-process";
  }

  cudaEvent_t opened_evt_2 = mapper.open_event(slot_id, evt_handle);
  EXPECT_EQ(opened_evt_1, opened_evt_2) << "Mapper should cache event per slot";

  // Create a stream and wait on the opened event. First record the exporter
  // event so the wait can complete successfully.
  cudaStream_t stream = cuda_stream_create();
  ASSERT_NE(stream, nullptr);

  ASSERT_TRUE(cuda_event_record(exporter_evt));
  EXPECT_TRUE(mapper.wait_ready(slot_id, stream));
  EXPECT_TRUE(cuda_stream_synchronize(stream));

  // Cleanup
  EXPECT_TRUE(cuda_event_destroy(exporter_evt));
  EXPECT_TRUE(cuda_stream_destroy(stream));
  mapper.close_slot(slot_id);
}
