// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include <sys/mman.h>
#include <unistd.h>

#include <atomic>

#include "ros2_cuda_ipc_core/buffer_metadata/buffer_ref.hpp"
#include "ros2_cuda_ipc_core/detail/buffer_metadata_cache.hpp"
#include "ros2_cuda_ipc_core/publisher/buffer_metadata_manager.hpp"

namespace ros2_cuda_ipc_core {
namespace {

uint32_t next_block_id() {
  static std::atomic<uint32_t> next{2000000};
  return next.fetch_add(1);
}

std::string name_for(uint32_t block_id) {
  return publisher::BufferMetadataManager::shm_name_for_block(
      static_cast<uint32_t>(::getpid()), block_id);
}

}  // namespace

TEST(BufferMetadataCacheTest, KeyIsPublisherPidAndBlockId) {
  const uint32_t pid = static_cast<uint32_t>(::getpid());
  const uint32_t first_id = next_block_id();
  const uint32_t second_id = next_block_id();
  const auto first_name = name_for(first_id);
  const auto second_name = name_for(second_id);
  auto first_owner = buffer_metadata::BufferMetadata::create(first_name);
  auto second_owner = buffer_metadata::BufferMetadata::create(second_name);
  ASSERT_TRUE(first_owner);
  ASSERT_TRUE(second_owner);
  const auto first_res =
      buffer_metadata::BufferRef::reserve_for_publish(first_owner);
  const auto second_res =
      buffer_metadata::BufferRef::reserve_for_publish(second_owner);
  ASSERT_TRUE(first_res);
  ASSERT_TRUE(second_res);

  subscriber::detail::BufferMetadataCache cache;
  auto first = cache.get_or_attach(pid, first_id, first_res->uid);
  auto second = cache.get_or_attach(pid, second_id, second_res->uid);
  ASSERT_TRUE(first);
  ASSERT_TRUE(second);
  EXPECT_NE(first.get(), second.get());
  EXPECT_EQ(cache.size(), 2u);

  ASSERT_TRUE(
      buffer_metadata::BufferRef::cancel_publish(first_owner, first_res->uid));
  ASSERT_TRUE(buffer_metadata::BufferRef::cancel_publish(second_owner,
                                                         second_res->uid));
  ::shm_unlink(first_name.c_str());
  ::shm_unlink(second_name.c_str());
}

TEST(BufferMetadataCacheTest,
     PublisherRestartWithReusedPidInvalidatesAndReattachesMapping) {
  const uint32_t pid = static_cast<uint32_t>(::getpid());
  const uint32_t block_id = next_block_id();
  const auto name = name_for(block_id);
  auto old_owner = buffer_metadata::BufferMetadata::create(name);
  ASSERT_TRUE(old_owner);
  const auto old_res =
      buffer_metadata::BufferRef::reserve_for_publish(old_owner);
  ASSERT_TRUE(old_res);
  ASSERT_TRUE(
      buffer_metadata::BufferRef::commit_publish(old_owner, old_res->uid));

  subscriber::detail::BufferMetadataCache cache;
  auto cached_old = cache.get_or_attach(pid, block_id, old_res->uid);
  ASSERT_TRUE(cached_old);

  ASSERT_EQ(::shm_unlink(name.c_str()), 0);
  auto new_owner = buffer_metadata::BufferMetadata::create(name);
  ASSERT_TRUE(new_owner);
  const auto new_res =
      buffer_metadata::BufferRef::reserve_for_publish(new_owner);
  ASSERT_TRUE(new_res);
  ASSERT_NE(new_res->uid, old_res->uid);

  auto cached_new = cache.get_or_attach(pid, block_id, new_res->uid);
  ASSERT_TRUE(cached_new);
  EXPECT_NE(cached_new.get(), cached_old.get());
  EXPECT_EQ(buffer_metadata::BufferRef::current_uid(cached_new), new_res->uid);
  EXPECT_FALSE(
      buffer_metadata::BufferRef::acquire(cached_new, old_res->uid).valid());

  ASSERT_TRUE(
      buffer_metadata::BufferRef::cancel_publish(new_owner, new_res->uid));
  ::shm_unlink(name.c_str());
}

TEST(BufferMetadataCacheTest, FailedAttachDoesNotPopulateCache) {
  subscriber::detail::BufferMetadataCache cache;
  EXPECT_FALSE(cache.get_or_attach(static_cast<uint32_t>(::getpid()),
                                   next_block_id(), 1));
  EXPECT_EQ(cache.size(), 0u);
}

}  // namespace ros2_cuda_ipc_core
