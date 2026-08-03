// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <atomic>
#include <stdexcept>
#include <thread>
#include <vector>

#include "ros2_cuda_ipc_core/subscriber/ipc_handle_cache.hpp"

namespace ros2_cuda_ipc_core {

TEST(IpcHandleCacheTest, KeyEqualityAndHashUseBlockIdentityAndPayload) {
  subscriber::IpcHandleKey lhs{};
  lhs.publisher_pid = 100;
  lhs.block_id = 200;
  lhs.device_id = 2;
  lhs.vmm_socket_path = "path-3";
  lhs.event[0] = 5;

  subscriber::IpcHandleKey same = lhs;
  subscriber::IpcHandleKey different_device = lhs;
  different_device.device_id = 3;
  subscriber::IpcHandleKey different_mem = lhs;
  different_mem.vmm_socket_path = "path-7";
  subscriber::IpcHandleKey different_event = lhs;
  different_event.event[1] = 9;
  subscriber::IpcHandleKey different_block = lhs;
  different_block.block_id = 201;

  subscriber::IpcHandleKeyHash hash;
  EXPECT_TRUE(lhs == same);
  EXPECT_EQ(hash(lhs), hash(same));
  EXPECT_FALSE(lhs == different_device);
  EXPECT_NE(hash(lhs), hash(different_device));
  EXPECT_FALSE(lhs == different_mem);
  EXPECT_FALSE(lhs == different_event);
  EXPECT_FALSE(lhs == different_block);
  EXPECT_NE(hash(lhs), hash(different_block));
}

TEST(IpcHandleCacheTest, DuplicateInsertReturnsExistingEntry) {
  subscriber::IpcHandleCache cache([](const backend::ImportedResources&) {});
  subscriber::IpcHandleKey key{};
  key.vmm_socket_path = "path-11";
  key.event[0] = 13;

  backend::ImportedResources first;
  first.dev_ptr = reinterpret_cast<void*>(0x1010);
  first.event = reinterpret_cast<CUevent>(0x2020);

  backend::ImportedResources duplicate;
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
  std::atomic<std::size_t> observed_cache_size{999};
  subscriber::IpcHandleCache* cache_ptr = nullptr;
  subscriber::IpcHandleCache cache(
      [&released, &observed_cache_size,
       &cache_ptr](const backend::ImportedResources&) {
        released.fetch_add(1);
        // A duplicate candidate must be destroyed after insert releases the
        // cache mutex; otherwise this lookup would deadlock.
        observed_cache_size.store(cache_ptr->size());
      });
  cache_ptr = &cache;
  subscriber::IpcHandleKey key{};
  key.vmm_socket_path = "path-17";
  key.event[0] = 19;

  backend::ImportedResources first;
  first.dev_ptr = reinterpret_cast<void*>(0x5050);
  first.event = reinterpret_cast<CUevent>(0x6060);

  backend::ImportedResources duplicate;
  duplicate.dev_ptr = reinterpret_cast<void*>(0x7070);
  duplicate.event = reinterpret_cast<CUevent>(0x8080);

  auto first_entry = cache.insert_or_discard_duplicate(key, std::move(first));
  auto duplicate_entry =
      cache.insert_or_discard_duplicate(key, std::move(duplicate));

  EXPECT_EQ(released.load(), 1);
  EXPECT_EQ(observed_cache_size.load(), 1u);
  first_entry.reset();
  duplicate_entry.reset();
  EXPECT_EQ(released.load(), 1);
  cache.clear();
  EXPECT_EQ(released.load(), 2);
}

TEST(IpcHandleCacheTest, ClearDoesNotReleaseActiveResources) {
  std::atomic<int> released{0};
  subscriber::IpcHandleCache cache(
      [&released](const backend::ImportedResources&) {
        released.fetch_add(1);
      });
  subscriber::IpcHandleKey key{};
  backend::ImportedResources imported;
  imported.dev_ptr = reinterpret_cast<void*>(0x9090);
  auto entry = cache.insert_or_discard_duplicate(key, std::move(imported));
  auto active_resource = entry;
  entry.reset();

  cache.clear();

  EXPECT_EQ(released.load(), 0);
  EXPECT_EQ(cache.size(), 0u);
  ASSERT_TRUE(active_resource);
  EXPECT_EQ(active_resource->dev_ptr, reinterpret_cast<void*>(0x9090));
  active_resource.reset();
  EXPECT_EQ(released.load(), 1);
}

TEST(IpcHandleCacheTest, ClearDefersReleaseUntilExternalOwnerIsGone) {
  std::atomic<int> released{0};
  subscriber::IpcHandleCache cache(
      [&released](const backend::ImportedResources&) {
        released.fetch_add(1);
      });
  subscriber::IpcHandleKey key{};
  backend::ImportedResources imported;
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

TEST(IpcHandleCacheTest, ClearWithoutActiveViewReleasesCachedResource) {
  std::atomic<int> released{0};
  subscriber::IpcHandleCache cache(
      [&released](const backend::ImportedResources&) {
        released.fetch_add(1);
      });
  subscriber::IpcHandleKey key{};
  backend::ImportedResources imported;
  imported.dev_ptr = reinterpret_cast<void*>(0xa1a1);

  auto entry = cache.insert_or_discard_duplicate(key, std::move(imported));
  entry.reset();

  EXPECT_EQ(released.load(), 0);
  EXPECT_EQ(cache.size(), 1u);
  cache.clear();
  EXPECT_EQ(released.load(), 1);
  EXPECT_EQ(cache.size(), 0u);
}

TEST(IpcHandleCacheTest, CacheHitReusesTheSameEntry) {
  std::atomic<int> released{0};
  subscriber::IpcHandleCache cache(
      [&released](const backend::ImportedResources&) {
        released.fetch_add(1);
      });
  subscriber::IpcHandleKey key{};
  key.vmm_socket_path = "path-41";
  key.event[0] = 43;

  int import_count = 0;
  backend::ImportedResources imported;
  imported.dev_ptr = reinterpret_cast<void*>(0xd0d0);
  ++import_count;
  auto first = cache.insert_or_discard_duplicate(key, std::move(imported));
  ASSERT_TRUE(first);

  int hit_count = 0;
  for (int i = 0; i < 1000; ++i) {
    auto hit = cache.find(key);
    ASSERT_TRUE(hit);
    ++hit_count;
    EXPECT_EQ(hit.get(), first.get());
  }

  EXPECT_EQ(import_count, 1);
  EXPECT_EQ(hit_count, 1000);
  EXPECT_EQ(released.load(), 0);
  first.reset();
  cache.clear();
  EXPECT_EQ(released.load(), 1);
}

TEST(IpcHandleCacheTest, EntryRemainsCachedAfterExternalOwnerIsDestroyed) {
  std::atomic<int> released{0};
  subscriber::IpcHandleCache cache(
      [&released](const backend::ImportedResources&) {
        released.fetch_add(1);
      });
  subscriber::IpcHandleKey key{};
  backend::ImportedResources imported;
  imported.dev_ptr = reinterpret_cast<void*>(0xe0e0);

  auto entry = cache.insert_or_discard_duplicate(key, std::move(imported));
  const auto* address = entry.get();
  {
    auto external_owner = entry;
    entry.reset();
    ASSERT_TRUE(external_owner);
  }

  EXPECT_EQ(released.load(), 0);
  EXPECT_EQ(cache.size(), 1u);
  auto hit = cache.find(key);
  ASSERT_TRUE(hit);
  EXPECT_EQ(hit.get(), address);
  hit.reset();
  EXPECT_EQ(released.load(), 0);

  cache.clear();
  EXPECT_EQ(released.load(), 1);
}

TEST(IpcHandleCacheTest, DuplicateInsertionsAreThreadSafeAndReleaseLosers) {
  constexpr std::size_t kThreadCount = 8;
  std::atomic<int> released{0};
  subscriber::IpcHandleCache cache(
      [&released](const backend::ImportedResources&) {
        released.fetch_add(1);
      });
  subscriber::IpcHandleKey key{};
  key.vmm_socket_path = "path-51";
  key.event[0] = 53;

  std::vector<subscriber::IpcHandleCache::Entry> entries(kThreadCount);
  std::vector<std::thread> threads;
  threads.reserve(kThreadCount);
  std::atomic<std::size_t> ready{0};
  std::atomic<bool> start{false};
  for (std::size_t i = 0; i < kThreadCount; ++i) {
    threads.emplace_back([&, i] {
      backend::ImportedResources imported;
      imported.dev_ptr = reinterpret_cast<void*>(0x10000 + i * 0x10);
      imported.event = reinterpret_cast<CUevent>(0x20000 + i * 0x10);
      ready.fetch_add(1);
      while (!start.load()) {
        std::this_thread::yield();
      }
      entries[i] = cache.insert_or_discard_duplicate(key, std::move(imported));
    });
  }
  while (ready.load() != kThreadCount) {
    std::this_thread::yield();
  }
  start.store(true);
  for (auto& thread : threads) {
    thread.join();
  }

  ASSERT_EQ(cache.size(), 1u);
  ASSERT_TRUE(entries.front());
  for (const auto& entry : entries) {
    ASSERT_TRUE(entry);
    EXPECT_EQ(entry.get(), entries.front().get());
  }
  EXPECT_EQ(released.load(), static_cast<int>(kThreadCount - 1));

  entries.clear();
  EXPECT_EQ(released.load(), static_cast<int>(kThreadCount - 1));
  cache.clear();
  EXPECT_EQ(released.load(), static_cast<int>(kThreadCount));
}

TEST(IpcHandleCacheTest, CleanupIsOutsideMutexAndDoesNotPropagateExceptions) {
  std::atomic<int> released{0};
  std::atomic<std::size_t> observed_cache_size{999};
  subscriber::IpcHandleCache* cache_ptr = nullptr;
  subscriber::IpcHandleCache cache([&](const backend::ImportedResources&) {
    released.fetch_add(1);
    // clear() must have detached the entries before invoking the
    // deleter; otherwise this call would try to reacquire the mutex.
    observed_cache_size.store(cache_ptr->size());
    throw std::runtime_error("synthetic cleanup failure");
  });
  cache_ptr = &cache;
  subscriber::IpcHandleKey key{};
  backend::ImportedResources imported;
  imported.dev_ptr = reinterpret_cast<void*>(0xf0f0);
  auto entry = cache.insert_or_discard_duplicate(key, std::move(imported));
  entry.reset();

  EXPECT_NO_THROW(cache.clear());
  EXPECT_EQ(cache.size(), 0u);
  EXPECT_EQ(observed_cache_size.load(), 0u);
  EXPECT_EQ(released.load(), 1);
}

TEST(IpcHandleCacheTest, ExternalOwnersShareImportedResourceOwnership) {
  std::atomic<int> released{0};
  auto* imported = new backend::ImportedResources();
  imported->dev_ptr = reinterpret_cast<void*>(0xb0b0);
  std::shared_ptr<const backend::ImportedResources> resource(
      imported, [&released](const backend::ImportedResources* value) {
        released.fetch_add(1);
        delete value;
      });

  auto first = resource;
  auto second = first;
  resource.reset();
  first.reset();

  EXPECT_EQ(released.load(), 0);
  EXPECT_TRUE(second);
  EXPECT_EQ(second->dev_ptr, reinterpret_cast<void*>(0xb0b0));

  second.reset();
  EXPECT_EQ(released.load(), 1);
}

TEST(IpcHandleCacheTest, ExternalOwnerSurvivesCacheClear) {
  std::atomic<int> released{0};
  subscriber::IpcHandleCache cache(
      [&released](const backend::ImportedResources&) {
        released.fetch_add(1);
      });
  subscriber::IpcHandleKey key{};
  backend::ImportedResources imported;
  imported.dev_ptr = reinterpret_cast<void*>(0xc0c0);

  auto entry = cache.insert_or_discard_duplicate(key, std::move(imported));
  auto external_owner = entry;
  entry.reset();
  cache.clear();

  EXPECT_TRUE(external_owner);
  EXPECT_EQ(external_owner->dev_ptr, reinterpret_cast<void*>(0xc0c0));
  EXPECT_EQ(released.load(), 0);

  external_owner.reset();
  EXPECT_EQ(released.load(), 1);
}

}  // namespace ros2_cuda_ipc_core
