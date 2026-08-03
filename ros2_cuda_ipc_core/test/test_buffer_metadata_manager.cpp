// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <fcntl.h>
#include <gtest/gtest.h>
#include <sys/mman.h>
#include <unistd.h>

#include <atomic>
#include <set>
#include <thread>
#include <vector>

#include "ros2_cuda_ipc_core/buffer_metadata/buffer_ref.hpp"
#include "ros2_cuda_ipc_core/publisher/buffer_metadata_manager.hpp"

namespace ros2_cuda_ipc_core::publisher {

TEST(BufferMetadataManagerTest, MultiplePoolsReceiveProcessUniqueBlockIds) {
  BufferMetadataManager first(2);
  BufferMetadataManager second(3);
  ASSERT_TRUE(first.initialise());
  ASSERT_TRUE(second.initialise());

  std::set<uint32_t> ids;
  std::vector<BufferMetadataManager::Reservation> reservations;
  for (auto* manager : {&first, &second}) {
    while (auto reservation = manager->reserve_for_publish()) {
      EXPECT_EQ(reservation->publisher_pid, static_cast<uint32_t>(::getpid()));
      EXPECT_TRUE(ids.insert(reservation->block_id).second);
      EXPECT_EQ(reservation->shm_name,
                BufferMetadataManager::shm_name_for_block(
                    reservation->publisher_pid, reservation->block_id));
      reservations.push_back(std::move(*reservation));
    }
  }
  EXPECT_EQ(reservations.size(), 5u);
  for (const auto& reservation : reservations) {
    EXPECT_TRUE(buffer_metadata::BufferRef::cancel_publish(reservation.mapping,
                                                           reservation.uid));
  }
}

TEST(BufferMetadataManagerTest, ResetUnlinksEveryBlockMetadataObject) {
  BufferMetadataManager manager(3);
  ASSERT_TRUE(manager.initialise());
  std::vector<std::string> names;
  for (int i = 0; i < 3; ++i) {
    auto reservation = manager.reserve_for_publish();
    ASSERT_TRUE(reservation);
    names.push_back(reservation->shm_name);
    ASSERT_TRUE(manager.cancel(*reservation));
  }
  manager.reset();
  for (const auto& name : names) {
    const int fd = ::shm_open(name.c_str(), O_RDONLY, 0);
    EXPECT_EQ(fd, -1);
    if (fd != -1) ::close(fd);
  }
}

TEST(BufferMetadataManagerTest, ReinitialiseUsesNewBlockIdentities) {
  BufferMetadataManager manager(1);
  ASSERT_TRUE(manager.initialise());
  const auto first = manager.reserve_for_publish();
  ASSERT_TRUE(first);
  const uint32_t first_id = first->block_id;
  const std::string first_name = first->shm_name;
  ASSERT_TRUE(manager.cancel(*first));
  manager.reset();
  ASSERT_TRUE(manager.initialise());
  const auto second = manager.reserve_for_publish();
  ASSERT_TRUE(second);
  EXPECT_NE(second->block_id, first_id);
  EXPECT_NE(second->shm_name, first_name);
  ASSERT_TRUE(manager.cancel(*second));
}

TEST(BufferMetadataManagerTest, InitialisationCleansMetadataFromDeadPublisher) {
  constexpr uint32_t kDeadPid = 99999999;
  constexpr uint32_t kBlockId = 314159;
  const auto name =
      BufferMetadataManager::shm_name_for_block(kDeadPid, kBlockId);
  (void)::shm_unlink(name.c_str());
  auto orphan = buffer_metadata::BufferMetadata::create(name);
  ASSERT_TRUE(orphan);
  orphan.reset();

  BufferMetadataManager manager(1);
  ASSERT_TRUE(manager.initialise());
  const int fd = ::shm_open(name.c_str(), O_RDONLY, 0);
  EXPECT_EQ(fd, -1);
  if (fd != -1) ::close(fd);
}

TEST(BufferMetadataManagerTest, ResetRacingWithReserveLeavesNoReservation) {
  for (int iteration = 0; iteration < 100; ++iteration) {
    BufferMetadataManager manager(1);
    ASSERT_TRUE(manager.initialise());
    std::atomic<bool> start{false};
    std::optional<BufferMetadataManager::Reservation> reservation;
    std::thread reserve_thread([&] {
      while (!start.load(std::memory_order_acquire)) std::this_thread::yield();
      reservation = manager.reserve_for_publish();
    });
    std::thread reset_thread([&] {
      while (!start.load(std::memory_order_acquire)) std::this_thread::yield();
      manager.reset();
    });
    start.store(true, std::memory_order_release);
    reserve_thread.join();
    reset_thread.join();
    if (reservation) {
      EXPECT_TRUE(manager.cancel(*reservation));
      EXPECT_EQ(
          buffer_metadata::BufferRef::current_refcount(reservation->mapping),
          0u);
    }
  }
}

}  // namespace ros2_cuda_ipc_core::publisher
