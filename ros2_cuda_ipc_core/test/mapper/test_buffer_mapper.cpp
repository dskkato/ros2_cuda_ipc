// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include <sys/mman.h>

#include "ros2_cuda_ipc_core/subscriber/buffer_mapper.hpp"
#include "test_mapper_utils.hpp"

namespace ros2_cuda_ipc_core {

class BufferMapperTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { test::RclcppScope::SetUp(); }
  static void TearDownTestSuite() { test::RclcppScope::TearDown(); }
};

TEST_F(BufferMapperTest, MissingBlockMetadataReturnsEmptyOptional) {
  auto msg = test::make_cached_buffer_core_message(
      test::test_publisher_pid(), test::next_test_block_id(), 99, 1);
  subscriber::BufferMapper mapper;
  EXPECT_FALSE(mapper.map(msg, CU_STREAM_LEGACY));
}

TEST_F(BufferMapperTest, StaleUidIsRejectedBeforeGpuImport) {
  auto msg = test::make_seeded_buffer_core_message(2);
  ASSERT_NE(msg.publisher_pid, 0u);
  const auto name = test::metadata_shm_name(msg);
  ++msg.uid;
  subscriber::BufferMapper mapper;
  EXPECT_FALSE(mapper.map(msg, CU_STREAM_LEGACY));
  ::shm_unlink(name.c_str());
}

TEST_F(BufferMapperTest, ImportFailureReleasesAcquiredReference) {
  const uint32_t pid = test::test_publisher_pid();
  const uint32_t block_id = test::next_test_block_id();
  const auto name =
      publisher::BufferMetadataManager::shm_name_for_block(pid, block_id);
  auto mapping = buffer_metadata::BufferMetadata::create(name);
  ASSERT_TRUE(mapping);
  const auto reservation =
      buffer_metadata::BufferRef::reserve_for_publish(mapping);
  ASSERT_TRUE(reservation);
  auto msg =
      test::make_cached_buffer_core_message(pid, block_id, reservation->uid, 3);
  subscriber::BufferMapper mapper;
  EXPECT_FALSE(mapper.map(msg, CU_STREAM_LEGACY));
  EXPECT_EQ(buffer_metadata::BufferRef::current_refcount(mapping), 1u);
  ASSERT_TRUE(
      buffer_metadata::BufferRef::cancel_publish(mapping, reservation->uid));
  ::shm_unlink(name.c_str());
}

}  // namespace ros2_cuda_ipc_core
