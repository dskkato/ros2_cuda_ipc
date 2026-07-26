// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <fcntl.h>
#include <gtest/gtest.h>
#include <sys/mman.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <optional>
#include <sstream>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "ros2_cuda_ipc_core/lease/lease_handle.hpp"
#include "ros2_cuda_ipc_core/publisher/lease_manager.hpp"

namespace {

std::string make_unique_shm_name() {
  static std::atomic<int> counter{0};
  std::ostringstream out;
  out << "/lease_manager_" << ::getpid() << "_" << counter.fetch_add(1);
  return out.str();
}

}  // namespace

TEST(LeaseManagerTest, ResetRacingWithReserveDoesNotLeavePending) {
  for (int iteration = 0; iteration < 1000; ++iteration) {
    const std::string prefix = make_unique_shm_name();
    ros2_cuda_ipc_core::publisher::LeaseManager manager(
        prefix, 1, std::chrono::milliseconds(100));
    ASSERT_TRUE(manager.initialise());
    std::atomic<bool> start{false};
    std::optional<ros2_cuda_ipc_core::publisher::LeaseManager::Reservation>
        reservation;
    std::thread reserve_thread([&]() {
      while (!start.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
      reservation = manager.reserve_for_publish(1);
    });
    std::thread reset_thread([&]() {
      while (!start.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
      manager.reset();
    });

    start.store(true, std::memory_order_release);
    reserve_thread.join();
    reset_thread.join();

    if (reservation) {
      manager.cancel(*reservation);
      const auto pending =
          ros2_cuda_ipc_core::lease::LeaseHandle::current_pending(
              reservation->mapping, 0);
      ASSERT_TRUE(pending.has_value());
      EXPECT_EQ(*pending, 0u);
    }
  }
}

TEST(LeaseManagerTest, SamePrefixProducesDistinctInstances) {
  const std::string prefix = make_unique_shm_name();
  ros2_cuda_ipc_core::publisher::LeaseManager first(
      prefix, 1, std::chrono::milliseconds(100));
  ros2_cuda_ipc_core::publisher::LeaseManager second(
      prefix, 2, std::chrono::milliseconds(100));
  ASSERT_TRUE(first.initialise());
  ASSERT_TRUE(second.initialise());
  EXPECT_NE(first.shm_name(), second.shm_name());
  EXPECT_NE(first.publisher_instance_id(), second.publisher_instance_id());
}

TEST(LeaseManagerTest, ResetUnlinksAndReinitialiseChangesIdentity) {
  ros2_cuda_ipc_core::publisher::LeaseManager manager(
      make_unique_shm_name(), 1, std::chrono::milliseconds(100));
  ASSERT_TRUE(manager.initialise());
  const std::string old_name = manager.shm_name();
  const auto old_id = manager.publisher_instance_id();
  manager.reset();
  EXPECT_TRUE(manager.shm_name().empty());
  EXPECT_TRUE(ros2_cuda_ipc_core::is_nil(manager.publisher_instance_id()));
  const int fd = ::shm_open(old_name.c_str(), O_RDWR, 0660);
  if (fd != -1) {
    ::close(fd);
  }
  EXPECT_EQ(fd, -1);

  ASSERT_TRUE(manager.initialise());
  EXPECT_NE(manager.shm_name(), old_name);
  EXPECT_NE(manager.publisher_instance_id(), old_id);
}

TEST(LeaseManagerTest, InvalidPrefixFailsClosed) {
  ros2_cuda_ipc_core::publisher::LeaseManager manager(
      "/invalid/prefix", 1, std::chrono::milliseconds(100));
  EXPECT_FALSE(manager.initialise());
  EXPECT_TRUE(manager.shm_name().empty());
  EXPECT_TRUE(ros2_cuda_ipc_core::is_nil(manager.publisher_instance_id()));
}
