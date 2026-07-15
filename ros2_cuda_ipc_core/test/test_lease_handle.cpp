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

  auto reservation =
      ros2_cuda_ipc_core::LeaseHandle::reserve_for_publish(shm_name, 0);
  ASSERT_TRUE(reservation.has_value());

  {
    auto lease = ros2_cuda_ipc_core::LeaseHandle::acquire(
        shm_name, reservation->slot_id, reservation->generation);
    ASSERT_TRUE(lease.valid());

    auto other =
        ros2_cuda_ipc_core::LeaseHandle::reserve_for_publish(shm_name, 0);
    ASSERT_TRUE(other.has_value());
    EXPECT_NE(other->slot_id, reservation->slot_id);

    auto ref = ros2_cuda_ipc_core::LeaseHandle::current_refcount(
        shm_name, reservation->slot_id);
    ASSERT_TRUE(ref.has_value());
    EXPECT_EQ(ref.value(), 1u);
  }

  auto ref_after = ros2_cuda_ipc_core::LeaseHandle::current_refcount(
      shm_name, reservation->slot_id);
  ASSERT_TRUE(ref_after.has_value());
  EXPECT_EQ(ref_after.value(), 0u);

  auto reservation_after =
      ros2_cuda_ipc_core::LeaseHandle::reserve_for_publish(shm_name, 0);
  ASSERT_TRUE(reservation_after.has_value());
  EXPECT_EQ(reservation_after->slot_id, reservation->slot_id);

  ::shm_unlink(shm_name.c_str());
}

TEST(LeaseHandleTest, GenerationMismatchReturnsInvalid) {
  const std::string shm_name = make_unique_shm_name("lease_mismatch");
  ASSERT_TRUE(ros2_cuda_ipc_core::LeaseHandle::init(shm_name, 1));

  auto reservation =
      ros2_cuda_ipc_core::LeaseHandle::reserve_for_publish(shm_name, 0);
  ASSERT_TRUE(reservation.has_value());

  auto lease = ros2_cuda_ipc_core::LeaseHandle::acquire(
      shm_name, reservation->slot_id, reservation->generation + 1);
  EXPECT_FALSE(lease.valid());

  ::shm_unlink(shm_name.c_str());
}

TEST(LeaseHandleTest, PendingPreventsSlotReuse) {
  const std::string shm_name = make_unique_shm_name("lease_pending");
  ASSERT_TRUE(ros2_cuda_ipc_core::LeaseHandle::init(shm_name, 1));

  auto reservation =
      ros2_cuda_ipc_core::LeaseHandle::reserve_for_publish(shm_name, 2);
  ASSERT_TRUE(reservation.has_value());

  auto next = ros2_cuda_ipc_core::LeaseHandle::reserve_for_publish(shm_name, 0);
  EXPECT_FALSE(next.has_value());

  EXPECT_TRUE(
      ros2_cuda_ipc_core::LeaseHandle::force_clear_pending(shm_name, 0));

  next = ros2_cuda_ipc_core::LeaseHandle::reserve_for_publish(shm_name, 0);
  ASSERT_TRUE(next.has_value());
  EXPECT_EQ(next->slot_id, reservation->slot_id);

  ::shm_unlink(shm_name.c_str());
}

TEST(LeaseHandleTest, PendingDecrementedOnAcquire) {
  const std::string shm_name = make_unique_shm_name("lease_pending_dec");
  ASSERT_TRUE(ros2_cuda_ipc_core::LeaseHandle::init(shm_name, 1));

  auto reservation =
      ros2_cuda_ipc_core::LeaseHandle::reserve_for_publish(shm_name, 2);
  ASSERT_TRUE(reservation.has_value());

  {
    auto lease = ros2_cuda_ipc_core::LeaseHandle::acquire(
        shm_name, reservation->slot_id, reservation->generation);
    ASSERT_TRUE(lease.valid());
    auto pending = ros2_cuda_ipc_core::LeaseHandle::current_pending(
        shm_name, reservation->slot_id);
    ASSERT_TRUE(pending.has_value());
    EXPECT_EQ(pending.value(), 1u);
  }

  {
    auto lease = ros2_cuda_ipc_core::LeaseHandle::acquire(
        shm_name, reservation->slot_id, reservation->generation);
    ASSERT_TRUE(lease.valid());
    auto pending = ros2_cuda_ipc_core::LeaseHandle::current_pending(
        shm_name, reservation->slot_id);
    ASSERT_TRUE(pending.has_value());
    EXPECT_EQ(pending.value(), 0u);
  }

  {
    auto lease = ros2_cuda_ipc_core::LeaseHandle::acquire(
        shm_name, reservation->slot_id, reservation->generation);
    ASSERT_TRUE(lease.valid());
    auto pending = ros2_cuda_ipc_core::LeaseHandle::current_pending(
        shm_name, reservation->slot_id);
    ASSERT_TRUE(pending.has_value());
    EXPECT_EQ(pending.value(), 0u);
  }

  ::shm_unlink(shm_name.c_str());
}

TEST(LeaseHandleTest, ForceClearPendingResetsCounterWhenIdle) {
  const std::string shm_name = make_unique_shm_name("lease_force_clear");
  ASSERT_TRUE(ros2_cuda_ipc_core::LeaseHandle::init(shm_name, 1));

  auto reservation =
      ros2_cuda_ipc_core::LeaseHandle::reserve_for_publish(shm_name, 2);
  ASSERT_TRUE(reservation.has_value());

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

TEST(LeaseHandleTest, CancelRetriesTransientReservationContention) {
  const std::string shm_name = make_unique_shm_name("lease_cancel_contention");
  ASSERT_TRUE(ros2_cuda_ipc_core::LeaseHandle::init(shm_name, 1));

  for (int iteration = 0; iteration < 1000; ++iteration) {
    auto reservation =
        ros2_cuda_ipc_core::LeaseHandle::reserve_for_publish(shm_name, 1);
    ASSERT_TRUE(reservation.has_value());

    std::atomic<bool> start{false};
    bool cancelled = false;
    std::thread cancel_thread([&]() {
      while (!start.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
      cancelled = ros2_cuda_ipc_core::LeaseHandle::cancel_pending(
          shm_name, reservation->slot_id, reservation->generation);
    });
    std::thread reclaim_thread([&]() {
      while (!start.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
      ros2_cuda_ipc_core::LeaseHandle::force_clear_pending(
          shm_name, reservation->slot_id);
    });

    start.store(true, std::memory_order_release);
    cancel_thread.join();
    reclaim_thread.join();
    EXPECT_TRUE(cancelled);

    const auto pending = ros2_cuda_ipc_core::LeaseHandle::current_pending(
        shm_name, reservation->slot_id);
    ASSERT_TRUE(pending.has_value());
    EXPECT_EQ(*pending, 0u);
  }
  ::shm_unlink(shm_name.c_str());
}
