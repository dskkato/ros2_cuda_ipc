// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <cuda.h>
#include <cuda_runtime_api.h>
#include <gtest/gtest.h>
#include <sys/mman.h>

#include <memory>
#include <string>

#include "ros2_cuda_ipc_core/backend/vmm_fd/memory_importer.hpp"
#include "ros2_cuda_ipc_core/buffer_metadata/buffer_metadata.hpp"
#include "ros2_cuda_ipc_core/detail/cuda_driver_context.hpp"
#include "ros2_cuda_ipc_core/detail/read_handle_factory.hpp"
#include "test_instance_id.hpp"

namespace {

class ShmUnlinkGuard {
 public:
  explicit ShmUnlinkGuard(std::string shm_name)
      : shm_name_(std::move(shm_name)) {}

  ~ShmUnlinkGuard() { ::shm_unlink(shm_name_.c_str()); }

  ShmUnlinkGuard(const ShmUnlinkGuard&) = delete;
  ShmUnlinkGuard& operator=(const ShmUnlinkGuard&) = delete;

 private:
  std::string shm_name_;
};

std::optional<ros2_cuda_ipc_core::subscriber::ReadHandle> make_read_handle(
    CUevent producer_event, CUstream consumer_stream,
    const std::shared_ptr<ros2_cuda_ipc_core::detail::CudaDeviceContext>&
        context,
    const std::string& shm_name) {
  const auto instance_id =
      ros2_cuda_ipc_core::test::publisher_instance_id(shm_name);
  auto mapping = ros2_cuda_ipc_core::buffer_metadata::BufferMetadata::create(
      shm_name, instance_id, 1);
  if (!mapping) {
    return std::nullopt;
  }
  auto reservation =
      ros2_cuda_ipc_core::buffer_metadata::BufferRef::reserve_for_publish(
          mapping);
  if (!reservation) {
    return std::nullopt;
  }
  if (!ros2_cuda_ipc_core::buffer_metadata::BufferRef::commit_publish(
          mapping, reservation->slot_id, reservation->generation)) {
    (void)ros2_cuda_ipc_core::buffer_metadata::BufferRef::cancel_publish(
        mapping, reservation->slot_id, reservation->generation);
    return std::nullopt;
  }
  auto buffer_ref = ros2_cuda_ipc_core::buffer_metadata::BufferRef::acquire(
      mapping, reservation->slot_id, reservation->generation);
  if (!buffer_ref.valid()) {
    return std::nullopt;
  }

  auto resource =
      std::make_shared<ros2_cuda_ipc_core::backend::ImportedResources>();
  resource->dev_ptr = reinterpret_cast<void*>(0x1000);
  resource->event = producer_event;
  resource->context = context;
  auto publication = ros2_cuda_ipc_core::subscriber::detail::ReadHandleFactory::
      make_publication(
          std::move(resource),
          std::make_unique<ros2_cuda_ipc_core::buffer_metadata::BufferRef>(
              std::move(buffer_ref)),
          64, 0);
  if (!publication) {
    return std::nullopt;
  }
  return ros2_cuda_ipc_core::subscriber::detail::ReadHandleFactory::make_bound(
      *publication, consumer_stream);
}

TEST(ReadHandleTest, DriverWaitAcceptsRuntimeCreatedStream) {
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

  {
    const std::string shm_name = "/read_handle_runtime_stream";
    ShmUnlinkGuard guard(shm_name);
    auto read = make_read_handle(reinterpret_cast<CUevent>(runtime_event),
                                 reinterpret_cast<CUstream>(consumer), nullptr,
                                 shm_name);
    ASSERT_TRUE(read);
    EXPECT_TRUE(read->valid());
    EXPECT_EQ(read->data(), reinterpret_cast<void*>(0x1000));
    EXPECT_EQ(read->byte_size(), 64u);
  }
  ASSERT_EQ(cudaStreamSynchronize(consumer), cudaSuccess);

  EXPECT_EQ(cudaEventDestroy(runtime_event), cudaSuccess);
  EXPECT_EQ(cudaStreamDestroy(consumer), cudaSuccess);
  EXPECT_EQ(cudaStreamDestroy(producer), cudaSuccess);
}

TEST(ReadHandleTest, RuntimeEventCanBeWaitedOnDriverCreatedStream) {
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
  {
    const std::string shm_name = "/read_handle_driver_stream";
    ShmUnlinkGuard guard(shm_name);
    auto read = make_read_handle(runtime_event, consumer, nullptr, shm_name);
    ASSERT_TRUE(read);
  }
  ASSERT_EQ(cuStreamSynchronize(consumer), CUDA_SUCCESS);

  EXPECT_EQ(cuStreamDestroy(consumer), CUDA_SUCCESS);
  EXPECT_EQ(cudaEventDestroy(runtime_event), cudaSuccess);
  EXPECT_EQ(cudaStreamDestroy(producer), cudaSuccess);
}

TEST(ReadHandleTest, RejectsStreamFromAnotherCudaDevice) {
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

  ASSERT_EQ(cudaSetDevice(1), cudaSuccess);
  cudaStream_t other_device_stream = nullptr;
  ASSERT_EQ(cudaStreamCreate(&other_device_stream), cudaSuccess);
  {
    const std::string shm_name = "/read_handle_other_device";
    ShmUnlinkGuard guard(shm_name);
    auto read = make_read_handle(
        nullptr, reinterpret_cast<CUstream>(other_device_stream),
        context_result.value(), shm_name);
    EXPECT_FALSE(read);
  }
  EXPECT_EQ(cudaStreamDestroy(other_device_stream), cudaSuccess);
}

TEST(ReadHandleTest, AcceptsSpecialStreamsWhenCallerContextIsOnAnotherDevice) {
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

  ASSERT_EQ(cudaSetDevice(0), cudaSuccess);
  cudaStream_t producer = nullptr;
  cudaEvent_t runtime_event = nullptr;
  ASSERT_EQ(cudaStreamCreate(&producer), cudaSuccess);
  ASSERT_EQ(cudaEventCreateWithFlags(&runtime_event, cudaEventDisableTiming),
            cudaSuccess);
  ASSERT_EQ(cudaEventRecord(runtime_event, producer), cudaSuccess);

  ASSERT_EQ(cudaSetDevice(1), cudaSuccess);
  for (const CUstream stream : {CU_STREAM_LEGACY, CU_STREAM_PER_THREAD}) {
    const std::string shm_name = "/read_handle_special_stream";
    ShmUnlinkGuard guard(shm_name);
    auto read = make_read_handle(runtime_event, stream, context_result.value(),
                                 shm_name);
    ASSERT_TRUE(read);
  }

  ASSERT_EQ(cudaSetDevice(0), cudaSuccess);
  ASSERT_EQ(cudaDeviceSynchronize(), cudaSuccess);

  EXPECT_EQ(cudaEventDestroy(runtime_event), cudaSuccess);
  EXPECT_EQ(cudaStreamDestroy(producer), cudaSuccess);
}

}  // namespace
