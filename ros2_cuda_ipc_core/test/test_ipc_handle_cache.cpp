// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <atomic>

#include "ros2_cuda_ipc_core/subscriber/buffer_view.hpp"
#include "ros2_cuda_ipc_core/subscriber/ipc_handle_cache.hpp"
#include "test_instance_id.hpp"

namespace ros2_cuda_ipc_core {

TEST(IpcHandleCacheTest, KeyEqualityAndHashUseInstanceBackendPayloadAndEvent) {
  subscriber::IpcHandleKey lhs{};
  lhs.publisher_instance_id = test::publisher_instance_id("lhs");
  lhs.backend = 1;
  lhs.mem[0] = 3;
  lhs.event[0] = 5;

  subscriber::IpcHandleKey same = lhs;
  subscriber::IpcHandleKey different_backend = lhs;
  different_backend.backend = 2;
  subscriber::IpcHandleKey different_mem = lhs;
  different_mem.mem[1] = 7;
  subscriber::IpcHandleKey different_event = lhs;
  different_event.event[1] = 9;
  subscriber::IpcHandleKey different_instance = lhs;
  different_instance.publisher_instance_id =
      test::publisher_instance_id("different");

  subscriber::IpcHandleKeyHash hash;
  EXPECT_TRUE(lhs == same);
  EXPECT_EQ(hash(lhs), hash(same));
  EXPECT_FALSE(lhs == different_backend);
  EXPECT_FALSE(lhs == different_mem);
  EXPECT_FALSE(lhs == different_event);
  EXPECT_FALSE(lhs == different_instance);
  EXPECT_NE(hash(lhs), hash(different_instance));
}

TEST(IpcHandleCacheTest, DuplicateInsertReturnsExistingEntry) {
  subscriber::IpcHandleCache cache([](const backend::ImportedMemory&) {});
  subscriber::IpcHandleKey key{};
  key.backend = 1;
  key.mem[0] = 11;
  key.event[0] = 13;

  backend::ImportedMemory first;
  first.dev_ptr = reinterpret_cast<void*>(0x1010);
  first.event = reinterpret_cast<CUevent>(0x2020);

  backend::ImportedMemory duplicate;
  duplicate.dev_ptr = reinterpret_cast<void*>(0x3030);
  duplicate.event = reinterpret_cast<CUevent>(0x4040);

  auto inserted = cache.insert_or_discard_duplicate(key, std::move(first));
  auto second = cache.insert_or_discard_duplicate(key, std::move(duplicate));

  ASSERT_TRUE(inserted);
  ASSERT_TRUE(second);
  EXPECT_EQ(inserted->dev_ptr, reinterpret_cast<void*>(0x1010));
  EXPECT_EQ(second->dev_ptr, inserted->dev_ptr);
  EXPECT_EQ(second->event, inserted->event);
  EXPECT_EQ(cache.size(), 1u);
}

TEST(IpcHandleCacheTest, DuplicateInsertInvokesReleaseHook) {
  std::atomic<int> released{0};
  subscriber::IpcHandleCache cache(
      [&released](const backend::ImportedMemory&) { released.fetch_add(1); });
  subscriber::IpcHandleKey key{};
  key.backend = 1;
  key.mem[0] = 17;
  key.event[0] = 19;

  backend::ImportedMemory first;
  first.dev_ptr = reinterpret_cast<void*>(0x5050);
  first.event = reinterpret_cast<CUevent>(0x6060);

  backend::ImportedMemory duplicate;
  duplicate.dev_ptr = reinterpret_cast<void*>(0x7070);
  duplicate.event = reinterpret_cast<CUevent>(0x8080);

  cache.insert_or_discard_duplicate(key, std::move(first));
  cache.insert_or_discard_duplicate(key, std::move(duplicate));

  EXPECT_EQ(released.load(), 1);
}

TEST(IpcHandleCacheTest, ClearReleasesCacheOwnedResources) {
  std::atomic<int> released{0};
  subscriber::IpcHandleCache cache(
      [&released](const backend::ImportedMemory&) { released.fetch_add(1); });
  subscriber::IpcHandleKey key{};
  backend::ImportedMemory imported;
  imported.dev_ptr = reinterpret_cast<void*>(0x9090);
  cache.insert_or_discard_duplicate(key, std::move(imported));

  cache.clear();

  EXPECT_EQ(released.load(), 1);
  EXPECT_EQ(cache.size(), 0u);
}

TEST(IpcHandleCacheTest, ClearDefersReleaseUntilExternalOwnerIsGone) {
  std::atomic<int> released{0};
  subscriber::IpcHandleCache cache(
      [&released](const backend::ImportedMemory&) { released.fetch_add(1); });
  subscriber::IpcHandleKey key{};
  backend::ImportedMemory imported;
  imported.dev_ptr = reinterpret_cast<void*>(0xa0a0);
  auto entry = cache.insert_or_discard_duplicate(key, std::move(imported));

  cache.clear();

  EXPECT_EQ(cache.size(), 0u);
  EXPECT_EQ(released.load(), 0);
  ASSERT_TRUE(entry);
  EXPECT_EQ(entry->dev_ptr, reinterpret_cast<void*>(0xa0a0));

  entry.reset();
  EXPECT_EQ(released.load(), 1);
}

TEST(IpcHandleCacheTest, BufferViewCopiesShareImportedResourceOwnership) {
  std::atomic<int> released{0};
  auto* imported = new backend::ImportedMemory();
  imported->dev_ptr = reinterpret_cast<void*>(0xb0b0);
  std::shared_ptr<const backend::ImportedMemory> resource(
      imported, [&released](const backend::ImportedMemory* value) {
        released.fetch_add(1);
        delete value;
      });

  subscriber::BufferView first;
  first.set_imported_resource(resource);
  subscriber::BufferView second = first;
  resource.reset();
  first.reset();

  EXPECT_EQ(released.load(), 0);
  EXPECT_TRUE(second.valid());
  EXPECT_EQ(second.dev_ptr, reinterpret_cast<void*>(0xb0b0));

  second.reset();
  EXPECT_EQ(released.load(), 1);
}

}  // namespace ros2_cuda_ipc_core
