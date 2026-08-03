// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <cuda_runtime_api.h>
#include <fcntl.h>
#include <gtest/gtest.h>
#include <sys/mman.h>
#include <unistd.h>

#include <set>
#include <string>
#include <vector>

#include "ros2_cuda_ipc_core/buffer_metadata/buffer_metadata.hpp"
#include "ros2_cuda_ipc_core/publisher/buffer_metadata_manager.hpp"
#include "ros2_cuda_ipc_core/publisher/gpu_buffer_manager.hpp"

namespace ros2_cuda_ipc_core::publisher {
namespace {

class GpuBufferManagerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    int count = 0;
    if (cudaGetDeviceCount(&count) != cudaSuccess || count == 0 ||
        cudaSetDevice(0) != cudaSuccess) {
      GTEST_SKIP() << "CUDA device not available";
    }
  }
};

}  // namespace

TEST_F(GpuBufferManagerTest, DescriptorDirectlyIdentifiesBlock) {
  GpuBufferManager manager({2, 1024, 0});
  ASSERT_TRUE(manager.initialise());
  std::set<uint32_t> ids;
  for (int i = 0; i < 2; ++i) {
    auto block = manager.acquire_for_publish();
    ASSERT_TRUE(block);
    const auto descriptor = block->prepare_publish(nullptr);
    ASSERT_TRUE(descriptor) << descriptor.error().to_string();
    EXPECT_EQ(descriptor.value().publisher_pid,
              static_cast<uint32_t>(::getpid()));
    EXPECT_TRUE(ids.insert(descriptor.value().block_id).second);
    EXPECT_NE(descriptor.value().uid, 0u);
    const auto name = BufferMetadataManager::shm_name_for_block(
        descriptor.value().publisher_pid, descriptor.value().block_id);
    EXPECT_TRUE(buffer_metadata::BufferMetadata::attach(name));
  }
}

TEST_F(GpuBufferManagerTest, ResetUnlinksAllBlockMetadataObjects) {
  GpuBufferManager manager({2, 1024, 0});
  ASSERT_TRUE(manager.initialise());
  std::vector<std::string> names;
  for (int i = 0; i < 2; ++i) {
    auto block = manager.acquire_for_publish();
    ASSERT_TRUE(block);
    const auto descriptor = block->prepare_publish(nullptr);
    ASSERT_TRUE(descriptor);
    names.push_back(BufferMetadataManager::shm_name_for_block(
        descriptor.value().publisher_pid, descriptor.value().block_id));
  }
  manager.reset();
  for (const auto& name : names) {
    const int fd = ::shm_open(name.c_str(), O_RDONLY, 0);
    EXPECT_EQ(fd, -1);
    if (fd != -1) ::close(fd);
  }
}

TEST_F(GpuBufferManagerTest, DifferentPoolsUseDifferentBlockIds) {
  GpuBufferManager first({1, 1024, 0});
  GpuBufferManager second({1, 1024, 0});
  ASSERT_TRUE(first.initialise());
  ASSERT_TRUE(second.initialise());
  auto first_block = first.acquire_for_publish();
  auto second_block = second.acquire_for_publish();
  ASSERT_TRUE(first_block);
  ASSERT_TRUE(second_block);
  const auto first_descriptor = first_block->prepare_publish(nullptr);
  const auto second_descriptor = second_block->prepare_publish(nullptr);
  ASSERT_TRUE(first_descriptor);
  ASSERT_TRUE(second_descriptor);
  EXPECT_NE(first_descriptor.value().block_id,
            second_descriptor.value().block_id);
}

}  // namespace ros2_cuda_ipc_core::publisher
