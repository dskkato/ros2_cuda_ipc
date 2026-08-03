// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include <sys/mman.h>
#include <unistd.h>

#include <atomic>
#include <sstream>
#include <string>
#include <utility>

#include "ros2_cuda_ipc_core/buffer_metadata/buffer_metadata.hpp"
#include "ros2_cuda_ipc_core/buffer_metadata/buffer_ref.hpp"
#include "ros2_cuda_ipc_core/detail/buffer_metadata_cache.hpp"

namespace {

std::string name(uint32_t block_id) {
  return ros2_cuda_ipc_core::buffer_metadata::block_metadata_shm_name(
      static_cast<uint32_t>(::getpid()), block_id);
}

class ShmGuard {
 public:
  explicit ShmGuard(std::string name) : name_(std::move(name)) {}
  ~ShmGuard() { ::shm_unlink(name_.c_str()); }
  ShmGuard(const ShmGuard&) = delete;
  ShmGuard& operator=(const ShmGuard&) = delete;

 private:
  std::string name_;
};

}  // namespace

using ros2_cuda_ipc_core::buffer_metadata::BufferMetadata;
using ros2_cuda_ipc_core::buffer_metadata::BufferRef;

TEST(BlockMetadataTest, OneMappingContainsOneBlockAndUsesDerivedName) {
  const std::string shm_name = name(900001);
  ShmGuard guard(shm_name);
  auto owner = BufferMetadata::create(shm_name, 0x123456789abcdef0ULL);
  ASSERT_TRUE(owner);
  EXPECT_EQ(owner->block()->uid.load(), 0x123456789abcdef0ULL);

  auto attached = BufferMetadata::attach(shm_name);
  ASSERT_TRUE(attached);
  EXPECT_EQ(attached->block()->uid.load(), owner->block()->uid.load());
}

TEST(BlockMetadataTest, UidRejectsStaleDescriptorAndReuseGetsNewIdentity) {
  const std::string shm_name = name(900002);
  ShmGuard guard(shm_name);
  auto mapping = BufferMetadata::create(shm_name, 11);
  ASSERT_TRUE(mapping);
  auto first = BufferRef::reserve_for_publish(mapping);
  ASSERT_TRUE(first);
  ASSERT_TRUE(BufferRef::commit_publish(mapping, first->uid));
  const uint64_t stale_uid = first->uid;

  // The grace period is part of the existing reuse contract.
  usleep(110000);
  auto second = BufferRef::reserve_for_publish(mapping);
  ASSERT_TRUE(second);
  EXPECT_NE(second->uid, stale_uid);
  EXPECT_FALSE(BufferRef::acquire(mapping, stale_uid).valid());
  ASSERT_TRUE(BufferRef::commit_publish(mapping, second->uid));
  EXPECT_TRUE(BufferRef::acquire(mapping, second->uid).valid());
}

TEST(BlockMetadataTest, CacheInvalidationReattachesRecreatedBlock) {
  constexpr uint32_t block_id = 900003;
  const std::string shm_name = name(block_id);
  ShmGuard guard(shm_name);
  auto old_owner = BufferMetadata::create(shm_name, 101);
  ASSERT_TRUE(old_owner);
  ros2_cuda_ipc_core::subscriber::detail::BufferMetadataCache cache;
  auto old_mapping =
      cache.get_or_attach(static_cast<uint32_t>(::getpid()), block_id);
  ASSERT_TRUE(old_mapping);
  old_owner.reset();
  ::shm_unlink(shm_name.c_str());

  auto new_owner = BufferMetadata::create(shm_name, 202);
  ASSERT_TRUE(new_owner);
  EXPECT_EQ(cache.get_or_attach(static_cast<uint32_t>(::getpid()), block_id)
                ->block()
                ->uid.load(),
            101u);
  cache.invalidate(static_cast<uint32_t>(::getpid()), block_id);
  auto refreshed =
      cache.get_or_attach(static_cast<uint32_t>(::getpid()), block_id);
  ASSERT_TRUE(refreshed);
  EXPECT_EQ(refreshed->block()->uid.load(), 202u);
}
