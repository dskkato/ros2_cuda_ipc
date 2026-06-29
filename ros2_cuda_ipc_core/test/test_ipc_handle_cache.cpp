// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <atomic>

#include "ros2_cuda_ipc_core/ipc_handle_cache.hpp"

namespace {

TEST(IpcHandleCacheTest, KeyEqualityAndHashUseBackendPayloadAndEvent) {
  ros2_cuda_ipc_core::IpcHandleKey lhs{};
  lhs.backend = 1;
  lhs.mem[0] = 3;
  lhs.event[0] = 5;

  ros2_cuda_ipc_core::IpcHandleKey same = lhs;
  ros2_cuda_ipc_core::IpcHandleKey different_backend = lhs;
  different_backend.backend = 2;
  ros2_cuda_ipc_core::IpcHandleKey different_mem = lhs;
  different_mem.mem[1] = 7;
  ros2_cuda_ipc_core::IpcHandleKey different_event = lhs;
  different_event.event[1] = 9;

  ros2_cuda_ipc_core::IpcHandleKeyHash hash;
  EXPECT_TRUE(lhs == same);
  EXPECT_EQ(hash(lhs), hash(same));
  EXPECT_FALSE(lhs == different_backend);
  EXPECT_FALSE(lhs == different_mem);
  EXPECT_FALSE(lhs == different_event);
}

TEST(IpcHandleCacheTest, DuplicateInsertReturnsExistingEntry) {
  ros2_cuda_ipc_core::IpcHandleCache cache;
  ros2_cuda_ipc_core::IpcHandleKey key{};
  key.backend = 1;
  key.mem[0] = 11;
  key.event[0] = 13;

  ros2_cuda_ipc_core::backend::ImportedBuffer first;
  first.dev_ptr = reinterpret_cast<void*>(0x1010);
  first.event = reinterpret_cast<cudaEvent_t>(0x2020);

  ros2_cuda_ipc_core::backend::ImportedBuffer duplicate;
  duplicate.dev_ptr = reinterpret_cast<void*>(0x3030);
  duplicate.event = reinterpret_cast<cudaEvent_t>(0x4040);

  auto inserted = cache.insert_or_discard_duplicate(key, first);
  auto second = cache.insert_or_discard_duplicate(key, duplicate);

  EXPECT_EQ(inserted.dev_ptr, first.dev_ptr);
  EXPECT_EQ(second.dev_ptr, first.dev_ptr);
  EXPECT_EQ(second.event, first.event);
  EXPECT_EQ(cache.size(), 1u);
}

TEST(IpcHandleCacheTest, DuplicateInsertInvokesReleaseHook) {
  std::atomic<int> released{0};
  ros2_cuda_ipc_core::IpcHandleCache cache(
      [&released](const ros2_cuda_ipc_core::backend::ImportedBuffer&) {
        released.fetch_add(1);
      });
  ros2_cuda_ipc_core::IpcHandleKey key{};
  key.backend = 1;
  key.mem[0] = 17;
  key.event[0] = 19;

  ros2_cuda_ipc_core::backend::ImportedBuffer first;
  first.dev_ptr = reinterpret_cast<void*>(0x5050);
  first.event = reinterpret_cast<cudaEvent_t>(0x6060);

  ros2_cuda_ipc_core::backend::ImportedBuffer duplicate;
  duplicate.dev_ptr = reinterpret_cast<void*>(0x7070);
  duplicate.event = reinterpret_cast<cudaEvent_t>(0x8080);

  cache.insert_or_discard_duplicate(key, first);
  cache.insert_or_discard_duplicate(key, duplicate);

  EXPECT_EQ(released.load(), 1);
}

}  // namespace
