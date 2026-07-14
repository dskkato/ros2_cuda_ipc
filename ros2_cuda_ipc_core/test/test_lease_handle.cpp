// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include <sys/mman.h>
#include <unistd.h>

#include <atomic>
#include <optional>
#include <sstream>
#include <string>
#include <thread>

#include "ros2_cuda_ipc_core/lease_handle.hpp"

namespace {

std::string make_unique_shm_name(const std::string& prefix) {
  static std::atomic<int> counter{0};
  std::ostringstream oss;
  oss << "/" << prefix << "_" << ::getpid() << "_" << counter.fetch_add(1);
  return oss.str();
}

}  // namespace

TEST(LeaseHandleTest, AcquireReleaseLifecycle) {
  const std::string shm_name = make_unique_shm_name("lease_ut");
  ASSERT_TRUE(ros2_cuda_ipc_core::LeaseHandle::init(shm_name, 2));

  auto slot = ros2_cuda_ipc_core::LeaseHandle::choose_empty_slot(shm_name);
  ASSERT_TRUE(slot.has_value());
  auto gen = ros2_cuda_ipc_core::LeaseHandle::bump_generation(shm_name,
                                                              slot.value(), 0);
  ASSERT_TRUE(gen.has_value());

  {
    auto lease = ros2_cuda_ipc_core::LeaseHandle::acquire(
        shm_name, slot.value(), gen.value());
    ASSERT_TRUE(lease.valid());

    auto other_slot =
        ros2_cuda_ipc_core::LeaseHandle::choose_empty_slot(shm_name);
    ASSERT_TRUE(other_slot.has_value());
    EXPECT_NE(other_slot.value(), slot.value());

    auto ref = ros2_cuda_ipc_core::LeaseHandle::current_refcount(shm_name,
                                                                 slot.value());
    ASSERT_TRUE(ref.has_value());
    EXPECT_EQ(ref.value(), 1u);
  }

  auto ref_after =
      ros2_cuda_ipc_core::LeaseHandle::current_refcount(shm_name, slot.value());
  ASSERT_TRUE(ref_after.has_value());
  EXPECT_EQ(ref_after.value(), 0u);

  auto slot_after =
      ros2_cuda_ipc_core::LeaseHandle::choose_empty_slot(shm_name);
  ASSERT_TRUE(slot_after.has_value());
  EXPECT_EQ(slot_after.value(), slot.value());

  ::shm_unlink(shm_name.c_str());
}

TEST(LeaseHandleTest, GenerationMismatchReturnsInvalid) {
  const std::string shm_name = make_unique_shm_name("lease_mismatch");
  ASSERT_TRUE(ros2_cuda_ipc_core::LeaseHandle::init(shm_name, 1));

  auto slot = ros2_cuda_ipc_core::LeaseHandle::choose_empty_slot(shm_name);
  ASSERT_TRUE(slot.has_value());
  auto gen = ros2_cuda_ipc_core::LeaseHandle::bump_generation(shm_name,
                                                              slot.value(), 0);
  ASSERT_TRUE(gen.has_value());

  auto lease = ros2_cuda_ipc_core::LeaseHandle::acquire(shm_name, slot.value(),
                                                        gen.value() + 1);
  EXPECT_FALSE(lease.valid());

  ::shm_unlink(shm_name.c_str());
}

TEST(LeaseHandleTest, PendingPreventsSlotReuse) {
  const std::string shm_name = make_unique_shm_name("lease_pending");
  ASSERT_TRUE(ros2_cuda_ipc_core::LeaseHandle::init(shm_name, 1));

  auto slot = ros2_cuda_ipc_core::LeaseHandle::choose_empty_slot(shm_name);
  ASSERT_TRUE(slot.has_value());
  auto gen = ros2_cuda_ipc_core::LeaseHandle::bump_generation(shm_name,
                                                              slot.value(), 2);
  ASSERT_TRUE(gen.has_value());

  auto free_slot = ros2_cuda_ipc_core::LeaseHandle::choose_empty_slot(shm_name);
  EXPECT_FALSE(free_slot.has_value());

  auto next_gen = ros2_cuda_ipc_core::LeaseHandle::bump_generation(
      shm_name, slot.value(), 0);
  ASSERT_TRUE(next_gen.has_value());

  free_slot = ros2_cuda_ipc_core::LeaseHandle::choose_empty_slot(shm_name);
  ASSERT_TRUE(free_slot.has_value());
  EXPECT_EQ(free_slot.value(), slot.value());

  ::shm_unlink(shm_name.c_str());
}

TEST(LeaseHandleTest, PendingDecrementedOnAcquire) {
  const std::string shm_name = make_unique_shm_name("lease_pending_dec");
  ASSERT_TRUE(ros2_cuda_ipc_core::LeaseHandle::init(shm_name, 1));

  auto slot = ros2_cuda_ipc_core::LeaseHandle::choose_empty_slot(shm_name);
  ASSERT_TRUE(slot.has_value());
  auto gen = ros2_cuda_ipc_core::LeaseHandle::bump_generation(shm_name,
                                                              slot.value(), 2);
  ASSERT_TRUE(gen.has_value());

  {
    auto lease = ros2_cuda_ipc_core::LeaseHandle::acquire(
        shm_name, slot.value(), gen.value());
    ASSERT_TRUE(lease.valid());
    auto pending = ros2_cuda_ipc_core::LeaseHandle::current_pending(
        shm_name, slot.value());
    ASSERT_TRUE(pending.has_value());
    EXPECT_EQ(pending.value(), 1u);
  }

  {
    auto lease = ros2_cuda_ipc_core::LeaseHandle::acquire(
        shm_name, slot.value(), gen.value());
    ASSERT_TRUE(lease.valid());
    auto pending = ros2_cuda_ipc_core::LeaseHandle::current_pending(
        shm_name, slot.value());
    ASSERT_TRUE(pending.has_value());
    EXPECT_EQ(pending.value(), 0u);
  }

  auto free_slot = ros2_cuda_ipc_core::LeaseHandle::choose_empty_slot(shm_name);
  ASSERT_TRUE(free_slot.has_value());
  EXPECT_EQ(free_slot.value(), slot.value());

  {
    auto lease = ros2_cuda_ipc_core::LeaseHandle::acquire(
        shm_name, slot.value(), gen.value());
    ASSERT_TRUE(lease.valid());
    auto pending = ros2_cuda_ipc_core::LeaseHandle::current_pending(
        shm_name, slot.value());
    ASSERT_TRUE(pending.has_value());
    EXPECT_EQ(pending.value(), 0u);
  }

  ::shm_unlink(shm_name.c_str());
}

TEST(LeaseHandleTest, ForceClearPendingResetsCounterWhenIdle) {
  const std::string shm_name = make_unique_shm_name("lease_force_clear");
  ASSERT_TRUE(ros2_cuda_ipc_core::LeaseHandle::init(shm_name, 1));

  auto gen = ros2_cuda_ipc_core::LeaseHandle::bump_generation(shm_name, 0, 2);
  ASSERT_TRUE(gen.has_value());

  auto pending = ros2_cuda_ipc_core::LeaseHandle::current_pending(shm_name, 0);
  ASSERT_TRUE(pending.has_value());
  EXPECT_EQ(pending.value(), 2u);

  EXPECT_TRUE(
      ros2_cuda_ipc_core::LeaseHandle::force_clear_pending(shm_name, 0));

  pending = ros2_cuda_ipc_core::LeaseHandle::current_pending(shm_name, 0);
  ASSERT_TRUE(pending.has_value());
  EXPECT_EQ(pending.value(), 0u);

  ::shm_unlink(shm_name.c_str());
}

TEST(LeaseHandleTest, AtomicPublisherReservationExcludesSubscriberAcquire) {
  const std::string shm_name = make_unique_shm_name("lease_reserve_race");
  ASSERT_TRUE(ros2_cuda_ipc_core::LeaseHandle::init(shm_name, 1));

  for (int iteration = 0; iteration < 1000; ++iteration) {
    auto initial =
        ros2_cuda_ipc_core::LeaseHandle::reserve_for_publish(shm_name, 0);
    ASSERT_TRUE(initial.has_value());

    std::atomic<bool> start{false};
    std::optional<ros2_cuda_ipc_core::LeaseHandle::PublisherReservation>
        publisher;
    std::optional<ros2_cuda_ipc_core::LeaseHandle> subscriber;
    std::thread publisher_thread([&]() {
      while (!start.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
      publisher =
          ros2_cuda_ipc_core::LeaseHandle::reserve_for_publish(shm_name, 0);
    });
    std::thread subscriber_thread([&]() {
      while (!start.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
      subscriber.emplace(ros2_cuda_ipc_core::LeaseHandle::acquire(
          shm_name, initial->slot_id, initial->generation));
    });
    start.store(true, std::memory_order_release);
    publisher_thread.join();
    subscriber_thread.join();

    EXPECT_FALSE(publisher.has_value() && subscriber->valid());
  }
  ::shm_unlink(shm_name.c_str());
}

TEST(LeaseHandleTest, OnlyOnePublisherCanReserveSingleSlot) {
  const std::string shm_name = make_unique_shm_name("lease_pub_race");
  ASSERT_TRUE(ros2_cuda_ipc_core::LeaseHandle::init(shm_name, 1));
  std::atomic<bool> start{false};
  std::optional<ros2_cuda_ipc_core::LeaseHandle::PublisherReservation> first;
  std::optional<ros2_cuda_ipc_core::LeaseHandle::PublisherReservation> second;
  std::thread a([&]() {
    while (!start.load(std::memory_order_acquire)) {
      std::this_thread::yield();
    }
    first = ros2_cuda_ipc_core::LeaseHandle::reserve_for_publish(shm_name, 1);
  });
  std::thread b([&]() {
    while (!start.load(std::memory_order_acquire)) {
      std::this_thread::yield();
    }
    second = ros2_cuda_ipc_core::LeaseHandle::reserve_for_publish(shm_name, 1);
  });
  start.store(true, std::memory_order_release);
  a.join();
  b.join();
  EXPECT_NE(first.has_value(), second.has_value());
  ::shm_unlink(shm_name.c_str());
}

TEST(LeaseHandleTest, CancelDoesNotClearNewerGeneration) {
  const std::string shm_name = make_unique_shm_name("lease_cancel_generation");
  ASSERT_TRUE(ros2_cuda_ipc_core::LeaseHandle::init(shm_name, 1));
  auto old = ros2_cuda_ipc_core::LeaseHandle::reserve_for_publish(shm_name, 0);
  ASSERT_TRUE(old.has_value());
  auto current =
      ros2_cuda_ipc_core::LeaseHandle::reserve_for_publish(shm_name, 1);
  ASSERT_TRUE(current.has_value());

  EXPECT_FALSE(ros2_cuda_ipc_core::LeaseHandle::cancel_pending(
      shm_name, old->slot_id, old->generation));
  auto pending = ros2_cuda_ipc_core::LeaseHandle::current_pending(shm_name, 0);
  ASSERT_TRUE(pending.has_value());
  EXPECT_EQ(*pending, 1u);
  ::shm_unlink(shm_name.c_str());
}
