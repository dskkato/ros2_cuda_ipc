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

#include "ros2_cuda_ipc_core/lease/lease_handle.hpp"
#include "test_instance_id.hpp"

namespace {

std::string make_unique_shm_name(const std::string& prefix) {
  static std::atomic<int> counter{0};
  std::ostringstream oss;
  oss << "/" << prefix << "_" << ::getpid() << "_" << counter.fetch_add(1);
  return oss.str();
}

}  // namespace

namespace ros2_cuda_ipc_core {
TEST(LeaseHandleTest, AcquireReleaseLifecycle) {
  const std::string shm_name = make_unique_shm_name("lease_ut");
  auto mapping = lease::LeaseMapping::create(
      shm_name, test::publisher_instance_id(shm_name), 2);
  ASSERT_TRUE(mapping);

  auto reservation = lease::LeaseHandle::reserve_for_publish(mapping, 0);
  ASSERT_TRUE(reservation.has_value());

  {
    auto lease = lease::LeaseHandle::acquire(mapping, reservation->slot_id,
                                             reservation->generation);
    ASSERT_TRUE(lease.valid());

    auto other = lease::LeaseHandle::reserve_for_publish(mapping, 0);
    ASSERT_TRUE(other.has_value());
    EXPECT_NE(other->slot_id, reservation->slot_id);

    auto ref =
        lease::LeaseHandle::current_refcount(mapping, reservation->slot_id);
    ASSERT_TRUE(ref.has_value());
    EXPECT_EQ(ref.value(), 1u);
  }

  auto ref_after =
      lease::LeaseHandle::current_refcount(mapping, reservation->slot_id);
  ASSERT_TRUE(ref_after.has_value());
  EXPECT_EQ(ref_after.value(), 0u);

  auto reservation_after = lease::LeaseHandle::reserve_for_publish(mapping, 0);
  ASSERT_TRUE(reservation_after.has_value());
  EXPECT_EQ(reservation_after->slot_id, reservation->slot_id);

  ::shm_unlink(shm_name.c_str());
}

TEST(LeaseHandleTest, GenerationMismatchReturnsInvalid) {
  const std::string shm_name = make_unique_shm_name("lease_mismatch");
  auto mapping = lease::LeaseMapping::create(
      shm_name, test::publisher_instance_id(shm_name), 1);
  ASSERT_TRUE(mapping);

  auto reservation = lease::LeaseHandle::reserve_for_publish(mapping, 0);
  ASSERT_TRUE(reservation.has_value());

  auto lease = lease::LeaseHandle::acquire(mapping, reservation->slot_id,
                                           reservation->generation + 1);
  EXPECT_FALSE(lease.valid());

  ::shm_unlink(shm_name.c_str());
}

TEST(LeaseHandleTest, PublisherInstanceMismatchReturnsInvalid) {
  const std::string shm_name = make_unique_shm_name("lease_instance_mismatch");
  const auto owner_id = test::publisher_instance_id(shm_name + "_owner");
  const auto other_id = test::publisher_instance_id(shm_name + "_other");
  auto mapping = lease::LeaseMapping::create(shm_name, owner_id, 1);
  ASSERT_TRUE(mapping);
  auto reservation = lease::LeaseHandle::reserve_for_publish(mapping, 1);
  ASSERT_TRUE(reservation.has_value());

  auto wrong_mapping = lease::LeaseMapping::attach(shm_name, other_id);
  EXPECT_FALSE(wrong_mapping);
  auto pending = lease::LeaseHandle::current_pending(mapping, 0);
  ASSERT_TRUE(pending.has_value());
  EXPECT_EQ(*pending, 1u);
  ::shm_unlink(shm_name.c_str());
}

TEST(LeaseHandleTest, HeaderPublisherInstanceMismatchReturnsInvalid) {
  const std::string shm_name =
      make_unique_shm_name("lease_header_instance_mismatch");
  const auto owner_id = test::publisher_instance_id(shm_name + "_owner");
  const auto other_id = test::publisher_instance_id(shm_name + "_other");
  auto mapping = lease::LeaseMapping::create(shm_name, owner_id, 1);
  ASSERT_TRUE(mapping);

  // Generation zero is valid in a newly-created slot. No mapping has been
  // cached yet, so this exercises validation against the mapped header.
  EXPECT_FALSE(lease::LeaseMapping::attach(shm_name, other_id));
  ::shm_unlink(shm_name.c_str());
}

TEST(LeaseHandleTest, SameSlotAndGenerationAreSeparatedByInstance) {
  const std::string first_name = make_unique_shm_name("lease_instance_first");
  const std::string second_name = make_unique_shm_name("lease_instance_second");
  const auto first_id = test::publisher_instance_id(first_name);
  const auto second_id = test::publisher_instance_id(second_name);
  auto first_mapping = lease::LeaseMapping::create(first_name, first_id, 1);
  auto second_mapping = lease::LeaseMapping::create(second_name, second_id, 1);
  ASSERT_TRUE(first_mapping);
  ASSERT_TRUE(second_mapping);
  auto first = lease::LeaseHandle::reserve_for_publish(first_mapping, 1);
  auto second = lease::LeaseHandle::reserve_for_publish(second_mapping, 1);
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  ASSERT_EQ(first->slot_id, second->slot_id);
  ASSERT_EQ(first->generation, second->generation);

  EXPECT_FALSE(lease::LeaseMapping::attach(first_name, second_id));
  EXPECT_TRUE(lease::LeaseHandle::acquire(first_mapping, first->slot_id,
                                          first->generation)
                  .valid());
  ::shm_unlink(first_name.c_str());
  ::shm_unlink(second_name.c_str());
}

TEST(LeaseHandleTest, ExplicitMappingSeparatesReusedNameByInstance) {
  const std::string shm_name = make_unique_shm_name("lease_cached_instance");
  const auto old_id = test::publisher_instance_id(shm_name + "_old");
  const auto new_id = test::publisher_instance_id(shm_name + "_new");
  auto old_mapping = lease::LeaseMapping::create(shm_name, old_id, 1);
  ASSERT_TRUE(old_mapping);
  ASSERT_EQ(::shm_unlink(shm_name.c_str()), 0);
  auto new_mapping = lease::LeaseMapping::create(shm_name, new_id, 1);
  ASSERT_TRUE(new_mapping);

  EXPECT_TRUE(lease::LeaseHandle::acquire(new_mapping, 0, 0).valid());
  // The old mapping remains usable because its owner still holds it; the new
  // instance is a separate mapping even though the POSIX name was reused.
  EXPECT_TRUE(lease::LeaseHandle::acquire(old_mapping, 0, 0).valid());
  old_mapping.reset();
  new_mapping.reset();
  ::shm_unlink(shm_name.c_str());
}

TEST(LeaseHandleTest, MappingLifetimeFollowsUsersAfterUnlink) {
  const std::string shm_name = make_unique_shm_name("lease_lifetime");
  const auto instance_id = test::publisher_instance_id(shm_name);
  auto mapping = lease::LeaseMapping::create(shm_name, instance_id, 1);
  ASSERT_TRUE(mapping);
  std::weak_ptr<lease::LeaseMapping> weak_mapping = mapping;
  auto reservation = lease::LeaseHandle::reserve_for_publish(mapping, 1);
  ASSERT_TRUE(reservation);
  {
    auto lease = lease::LeaseHandle::acquire(mapping, reservation->slot_id,
                                             reservation->generation);
    ASSERT_TRUE(lease.valid());
    ASSERT_EQ(::shm_unlink(shm_name.c_str()), 0);
    mapping.reset();
    EXPECT_FALSE(weak_mapping.expired());
  }
  reservation.reset();
  EXPECT_TRUE(weak_mapping.expired());
}

TEST(LeaseHandleTest, PendingPreventsSlotReuse) {
  const std::string shm_name = make_unique_shm_name("lease_pending");
  auto mapping = lease::LeaseMapping::create(
      shm_name, test::publisher_instance_id(shm_name), 1);
  ASSERT_TRUE(mapping);

  auto reservation = lease::LeaseHandle::reserve_for_publish(mapping, 2);
  ASSERT_TRUE(reservation.has_value());

  auto next = lease::LeaseHandle::reserve_for_publish(mapping, 0);
  EXPECT_FALSE(next.has_value());

  EXPECT_TRUE(lease::LeaseHandle::force_clear_pending(mapping, 0));

  next = lease::LeaseHandle::reserve_for_publish(mapping, 0);
  ASSERT_TRUE(next.has_value());
  EXPECT_EQ(next->slot_id, reservation->slot_id);

  ::shm_unlink(shm_name.c_str());
}

TEST(LeaseHandleTest, PendingDecrementedOnAcquire) {
  const std::string shm_name = make_unique_shm_name("lease_pending_dec");
  auto mapping = lease::LeaseMapping::create(
      shm_name, test::publisher_instance_id(shm_name), 1);
  ASSERT_TRUE(mapping);

  auto reservation = lease::LeaseHandle::reserve_for_publish(mapping, 2);
  ASSERT_TRUE(reservation.has_value());

  {
    auto lease = lease::LeaseHandle::acquire(mapping, reservation->slot_id,
                                             reservation->generation);
    ASSERT_TRUE(lease.valid());
    auto pending =
        lease::LeaseHandle::current_pending(mapping, reservation->slot_id);
    ASSERT_TRUE(pending.has_value());
    EXPECT_EQ(pending.value(), 1u);
  }

  {
    auto lease = lease::LeaseHandle::acquire(mapping, reservation->slot_id,
                                             reservation->generation);
    ASSERT_TRUE(lease.valid());
    auto pending =
        lease::LeaseHandle::current_pending(mapping, reservation->slot_id);
    ASSERT_TRUE(pending.has_value());
    EXPECT_EQ(pending.value(), 0u);
  }

  {
    auto lease = lease::LeaseHandle::acquire(mapping, reservation->slot_id,
                                             reservation->generation);
    ASSERT_TRUE(lease.valid());
    auto pending =
        lease::LeaseHandle::current_pending(mapping, reservation->slot_id);
    ASSERT_TRUE(pending.has_value());
    EXPECT_EQ(pending.value(), 0u);
  }

  ::shm_unlink(shm_name.c_str());
}

TEST(LeaseHandleTest, ForceClearPendingResetsCounterWhenIdle) {
  const std::string shm_name = make_unique_shm_name("lease_force_clear");
  auto mapping = lease::LeaseMapping::create(
      shm_name, test::publisher_instance_id(shm_name), 1);
  ASSERT_TRUE(mapping);

  auto reservation = lease::LeaseHandle::reserve_for_publish(mapping, 2);
  ASSERT_TRUE(reservation.has_value());

  auto pending = lease::LeaseHandle::current_pending(mapping, 0);
  ASSERT_TRUE(pending.has_value());
  EXPECT_EQ(pending.value(), 2u);

  EXPECT_TRUE(lease::LeaseHandle::force_clear_pending(mapping, 0));

  pending = lease::LeaseHandle::current_pending(mapping, 0);
  ASSERT_TRUE(pending.has_value());
  EXPECT_EQ(pending.value(), 0u);

  ::shm_unlink(shm_name.c_str());
}

TEST(LeaseHandleTest, AtomicPublisherReservationExcludesSubscriberAcquire) {
  const std::string shm_name = make_unique_shm_name("lease_reserve_race");
  auto mapping = lease::LeaseMapping::create(
      shm_name, test::publisher_instance_id(shm_name), 1);
  ASSERT_TRUE(mapping);

  for (int iteration = 0; iteration < 1000; ++iteration) {
    auto initial = lease::LeaseHandle::reserve_for_publish(mapping, 0);
    ASSERT_TRUE(initial.has_value());

    std::atomic<bool> start{false};
    std::optional<lease::LeaseHandle::PublisherReservation> publisher;
    std::optional<lease::LeaseHandle> subscriber;
    std::thread publisher_thread([&]() {
      while (!start.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
      publisher = lease::LeaseHandle::reserve_for_publish(mapping, 0);
    });
    std::thread subscriber_thread([&]() {
      while (!start.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
      subscriber.emplace(lease::LeaseHandle::acquire(mapping, initial->slot_id,
                                                     initial->generation));
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
  auto mapping = lease::LeaseMapping::create(
      shm_name, test::publisher_instance_id(shm_name), 1);
  ASSERT_TRUE(mapping);
  std::atomic<bool> start{false};
  std::optional<lease::LeaseHandle::PublisherReservation> first;
  std::optional<lease::LeaseHandle::PublisherReservation> second;
  std::thread a([&]() {
    while (!start.load(std::memory_order_acquire)) {
      std::this_thread::yield();
    }
    first = lease::LeaseHandle::reserve_for_publish(mapping, 1);
  });
  std::thread b([&]() {
    while (!start.load(std::memory_order_acquire)) {
      std::this_thread::yield();
    }
    second = lease::LeaseHandle::reserve_for_publish(mapping, 1);
  });
  start.store(true, std::memory_order_release);
  a.join();
  b.join();
  EXPECT_NE(first.has_value(), second.has_value());
  ::shm_unlink(shm_name.c_str());
}

TEST(LeaseHandleTest, CancelDoesNotClearNewerGeneration) {
  const std::string shm_name = make_unique_shm_name("lease_cancel_generation");
  auto mapping = lease::LeaseMapping::create(
      shm_name, test::publisher_instance_id(shm_name), 1);
  ASSERT_TRUE(mapping);
  auto old = lease::LeaseHandle::reserve_for_publish(mapping, 0);
  ASSERT_TRUE(old.has_value());
  auto current = lease::LeaseHandle::reserve_for_publish(mapping, 1);
  ASSERT_TRUE(current.has_value());

  EXPECT_FALSE(lease::LeaseHandle::cancel_pending(mapping, old->slot_id,
                                                  old->generation));
  auto pending = lease::LeaseHandle::current_pending(mapping, 0);
  ASSERT_TRUE(pending.has_value());
  EXPECT_EQ(*pending, 1u);
  ::shm_unlink(shm_name.c_str());
}

TEST(LeaseHandleTest, CancelRetriesTransientReservationContention) {
  const std::string shm_name = make_unique_shm_name("lease_cancel_contention");
  auto mapping = lease::LeaseMapping::create(
      shm_name, test::publisher_instance_id(shm_name), 1);
  ASSERT_TRUE(mapping);

  for (int iteration = 0; iteration < 1000; ++iteration) {
    auto reservation = lease::LeaseHandle::reserve_for_publish(mapping, 1);
    ASSERT_TRUE(reservation.has_value());

    std::atomic<bool> start{false};
    bool cancelled = false;
    std::thread cancel_thread([&]() {
      while (!start.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
      cancelled = lease::LeaseHandle::cancel_pending(
          mapping, reservation->slot_id, reservation->generation);
    });
    std::thread reclaim_thread([&]() {
      while (!start.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
      lease::LeaseHandle::force_clear_pending(mapping, reservation->slot_id);
    });

    start.store(true, std::memory_order_release);
    cancel_thread.join();
    reclaim_thread.join();
    EXPECT_TRUE(cancelled);

    const auto pending =
        lease::LeaseHandle::current_pending(mapping, reservation->slot_id);
    ASSERT_TRUE(pending.has_value());
    EXPECT_EQ(*pending, 0u);
  }
  ::shm_unlink(shm_name.c_str());
}

}  // namespace ros2_cuda_ipc_core
