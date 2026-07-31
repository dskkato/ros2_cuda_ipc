// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include <sys/mman.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <optional>
#include <sstream>
#include <string>
#include <thread>

#include "ros2_cuda_ipc_core/buffer_metadata/buffer_ref.hpp"
#include "test_instance_id.hpp"

namespace {

std::string make_unique_shm_name(const std::string& prefix) {
  static std::atomic<int> counter{0};
  std::ostringstream oss;
  oss << "/" << prefix << "_" << ::getpid() << "_" << counter.fetch_add(1);
  return oss.str();
}

void cancel(
    const std::shared_ptr<ros2_cuda_ipc_core::buffer_metadata::BufferMetadata>&
        mapping,
    const std::optional<
        ros2_cuda_ipc_core::buffer_metadata::BufferRef::PublisherReservation>&
        reservation) {
  if (reservation) {
    EXPECT_TRUE(ros2_cuda_ipc_core::buffer_metadata::BufferRef::cancel_publish(
        mapping, reservation->slot_id, reservation->generation));
  }
}

}  // namespace

namespace ros2_cuda_ipc_core {

TEST(BufferRefTest, AcquireReleaseLifecycle) {
  const std::string shm_name = make_unique_shm_name("buffer_ref_ut");
  auto mapping = buffer_metadata::BufferMetadata::create(
      shm_name, test::publisher_instance_id(shm_name), 2);
  ASSERT_TRUE(mapping);

  auto reservation = buffer_metadata::BufferRef::reserve_for_publish(mapping);
  ASSERT_TRUE(reservation.has_value());
  EXPECT_EQ(buffer_metadata::BufferRef::current_refcount(mapping,
                                                         reservation->slot_id),
            std::optional<uint32_t>(1));

  std::optional<buffer_metadata::BufferRef::PublisherReservation> other;
  {
    auto buffer_ref = buffer_metadata::BufferRef::acquire(
        mapping, reservation->slot_id, reservation->generation);
    ASSERT_TRUE(buffer_ref.valid());

    other = buffer_metadata::BufferRef::reserve_for_publish(mapping);
    ASSERT_TRUE(other.has_value());
    EXPECT_NE(other->slot_id, reservation->slot_id);

    auto ref = buffer_metadata::BufferRef::current_refcount(
        mapping, reservation->slot_id);
    ASSERT_TRUE(ref.has_value());
    EXPECT_EQ(ref.value(), 2u);
  }

  auto ref_after = buffer_metadata::BufferRef::current_refcount(
      mapping, reservation->slot_id);
  ASSERT_TRUE(ref_after.has_value());
  EXPECT_EQ(ref_after.value(), 1u);

  cancel(mapping, reservation);
  cancel(mapping, other);
  auto reservation_after =
      buffer_metadata::BufferRef::reserve_for_publish(mapping);
  ASSERT_TRUE(reservation_after.has_value());
  EXPECT_EQ(reservation_after->slot_id, reservation->slot_id);
  cancel(mapping, reservation_after);

  ::shm_unlink(shm_name.c_str());
}

TEST(BufferRefTest, GenerationMismatchReturnsInvalid) {
  const std::string shm_name = make_unique_shm_name("buffer_ref_mismatch");
  auto mapping = buffer_metadata::BufferMetadata::create(
      shm_name, test::publisher_instance_id(shm_name), 1);
  ASSERT_TRUE(mapping);

  auto reservation = buffer_metadata::BufferRef::reserve_for_publish(mapping);
  ASSERT_TRUE(reservation.has_value());
  auto buffer_ref = buffer_metadata::BufferRef::acquire(
      mapping, reservation->slot_id, reservation->generation + 1);
  EXPECT_FALSE(buffer_ref.valid());
  cancel(mapping, reservation);

  ::shm_unlink(shm_name.c_str());
}

TEST(BufferRefTest, PublisherInstanceMismatchReturnsInvalid) {
  const std::string shm_name =
      make_unique_shm_name("buffer_ref_instance_mismatch");
  const auto owner_id = test::publisher_instance_id(shm_name + "_owner");
  const auto other_id = test::publisher_instance_id(shm_name + "_other");
  auto mapping = buffer_metadata::BufferMetadata::create(shm_name, owner_id, 1);
  ASSERT_TRUE(mapping);
  auto reservation = buffer_metadata::BufferRef::reserve_for_publish(mapping);
  ASSERT_TRUE(reservation.has_value());

  EXPECT_FALSE(buffer_metadata::BufferMetadata::attach(shm_name, other_id));
  auto ref = buffer_metadata::BufferRef::current_refcount(mapping, 0);
  ASSERT_TRUE(ref.has_value());
  EXPECT_EQ(*ref, 1u);
  cancel(mapping, reservation);
  ::shm_unlink(shm_name.c_str());
}

TEST(BufferRefTest, HeaderPublisherInstanceMismatchReturnsInvalid) {
  const std::string shm_name =
      make_unique_shm_name("buffer_ref_header_instance_mismatch");
  const auto owner_id = test::publisher_instance_id(shm_name + "_owner");
  const auto other_id = test::publisher_instance_id(shm_name + "_other");
  auto mapping = buffer_metadata::BufferMetadata::create(shm_name, owner_id, 1);
  ASSERT_TRUE(mapping);
  EXPECT_FALSE(buffer_metadata::BufferMetadata::attach(shm_name, other_id));
  ::shm_unlink(shm_name.c_str());
}

TEST(BufferRefTest, SameSlotAndGenerationAreSeparatedByInstance) {
  const std::string first_name =
      make_unique_shm_name("buffer_ref_instance_first");
  const std::string second_name =
      make_unique_shm_name("buffer_ref_instance_second");
  const auto first_id = test::publisher_instance_id(first_name);
  const auto second_id = test::publisher_instance_id(second_name);
  auto first_mapping =
      buffer_metadata::BufferMetadata::create(first_name, first_id, 1);
  auto second_mapping =
      buffer_metadata::BufferMetadata::create(second_name, second_id, 1);
  ASSERT_TRUE(first_mapping);
  ASSERT_TRUE(second_mapping);
  auto first = buffer_metadata::BufferRef::reserve_for_publish(first_mapping);
  auto second = buffer_metadata::BufferRef::reserve_for_publish(second_mapping);
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  ASSERT_EQ(first->slot_id, second->slot_id);
  ASSERT_EQ(first->generation, second->generation);

  EXPECT_FALSE(buffer_metadata::BufferMetadata::attach(first_name, second_id));
  auto buffer_ref = buffer_metadata::BufferRef::acquire(
      first_mapping, first->slot_id, first->generation);
  EXPECT_TRUE(buffer_ref.valid());
  cancel(first_mapping, first);
  cancel(second_mapping, second);
  ::shm_unlink(first_name.c_str());
  ::shm_unlink(second_name.c_str());
}

TEST(BufferRefTest, ExplicitMappingSeparatesReusedNameByInstance) {
  const std::string shm_name =
      make_unique_shm_name("buffer_ref_cached_instance");
  const auto old_id = test::publisher_instance_id(shm_name + "_old");
  const auto new_id = test::publisher_instance_id(shm_name + "_new");
  auto old_mapping =
      buffer_metadata::BufferMetadata::create(shm_name, old_id, 1);
  ASSERT_TRUE(old_mapping);
  ASSERT_EQ(::shm_unlink(shm_name.c_str()), 0);
  auto new_mapping =
      buffer_metadata::BufferMetadata::create(shm_name, new_id, 1);
  ASSERT_TRUE(new_mapping);

  EXPECT_TRUE(buffer_metadata::BufferRef::acquire(new_mapping, 0, 0).valid());
  EXPECT_TRUE(buffer_metadata::BufferRef::acquire(old_mapping, 0, 0).valid());
  old_mapping.reset();
  new_mapping.reset();
  ::shm_unlink(shm_name.c_str());
}

TEST(BufferRefTest, MappingLifetimeFollowsUsersAfterUnlink) {
  const std::string shm_name = make_unique_shm_name("buffer_ref_lifetime");
  const auto instance_id = test::publisher_instance_id(shm_name);
  auto mapping =
      buffer_metadata::BufferMetadata::create(shm_name, instance_id, 1);
  ASSERT_TRUE(mapping);
  std::weak_ptr<buffer_metadata::BufferMetadata> weak_mapping = mapping;
  auto reservation = buffer_metadata::BufferRef::reserve_for_publish(mapping);
  ASSERT_TRUE(reservation);
  {
    auto buffer_ref = buffer_metadata::BufferRef::acquire(
        mapping, reservation->slot_id, reservation->generation);
    ASSERT_TRUE(buffer_ref.valid());
    ASSERT_EQ(::shm_unlink(shm_name.c_str()), 0);
    mapping.reset();
    EXPECT_FALSE(weak_mapping.expired());
  }
  EXPECT_TRUE(buffer_metadata::BufferRef::cancel_publish(
      reservation->mapping, reservation->slot_id, reservation->generation));
  reservation.reset();
  EXPECT_TRUE(weak_mapping.expired());
}

TEST(BufferRefTest, PublisherReservationUsesReferenceCount) {
  const std::string shm_name =
      make_unique_shm_name("buffer_ref_reservation_ref");
  auto mapping = buffer_metadata::BufferMetadata::create(
      shm_name, test::publisher_instance_id(shm_name), 1);
  ASSERT_TRUE(mapping);

  auto reservation = buffer_metadata::BufferRef::reserve_for_publish(mapping);
  ASSERT_TRUE(reservation.has_value());
  EXPECT_EQ(buffer_metadata::BufferRef::current_refcount(mapping, 0),
            std::optional<uint32_t>(1));
  EXPECT_FALSE(
      buffer_metadata::BufferRef::reserve_for_publish(mapping).has_value());
  cancel(mapping, reservation);
  EXPECT_EQ(buffer_metadata::BufferRef::current_refcount(mapping, 0),
            std::optional<uint32_t>(0));
  ::shm_unlink(shm_name.c_str());
}

TEST(BufferRefTest, CommitAppliesFixedGracePeriod) {
  const std::string shm_name = make_unique_shm_name("buffer_ref_grace");
  auto mapping = buffer_metadata::BufferMetadata::create(
      shm_name, test::publisher_instance_id(shm_name), 1);
  ASSERT_TRUE(mapping);

  auto reservation = buffer_metadata::BufferRef::reserve_for_publish(mapping);
  ASSERT_TRUE(reservation.has_value());
  ASSERT_TRUE(buffer_metadata::BufferRef::commit_publish(
      mapping, reservation->slot_id, reservation->generation));
  EXPECT_EQ(buffer_metadata::BufferRef::current_refcount(mapping, 0),
            std::optional<uint32_t>(0));
  const auto timestamp =
      buffer_metadata::BufferRef::current_publish_timestamp_us(mapping, 0);
  ASSERT_TRUE(timestamp.has_value());
  EXPECT_NE(*timestamp, 0u);
  EXPECT_FALSE(
      buffer_metadata::BufferRef::reserve_for_publish(mapping).has_value());

  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  auto next = buffer_metadata::BufferRef::reserve_for_publish(mapping);
  ASSERT_TRUE(next.has_value());
  cancel(mapping, next);
  ::shm_unlink(shm_name.c_str());
}

TEST(BufferRefTest, ActiveBufferRefBlocksReuseAfterGracePeriod) {
  const std::string shm_name = make_unique_shm_name("buffer_ref_active");
  auto mapping = buffer_metadata::BufferMetadata::create(
      shm_name, test::publisher_instance_id(shm_name), 1);
  ASSERT_TRUE(mapping);

  auto reservation = buffer_metadata::BufferRef::reserve_for_publish(mapping);
  ASSERT_TRUE(reservation.has_value());
  ASSERT_TRUE(buffer_metadata::BufferRef::commit_publish(
      mapping, reservation->slot_id, reservation->generation));
  std::optional<buffer_metadata::BufferRef> buffer_ref;
  buffer_ref.emplace(buffer_metadata::BufferRef::acquire(
      mapping, reservation->slot_id, reservation->generation));
  ASSERT_TRUE(buffer_ref->valid());
  std::this_thread::sleep_for(std::chrono::milliseconds(105));
  EXPECT_FALSE(
      buffer_metadata::BufferRef::reserve_for_publish(mapping).has_value());
  buffer_ref.reset();
  auto next = buffer_metadata::BufferRef::reserve_for_publish(mapping);
  ASSERT_TRUE(next.has_value());
  cancel(mapping, next);
  ::shm_unlink(shm_name.c_str());
}

TEST(BufferRefTest, PublisherAndSubscriberRaceIsGenerationSafe) {
  const std::string shm_name = make_unique_shm_name("buffer_ref_reserve_race");
  auto mapping = buffer_metadata::BufferMetadata::create(
      shm_name, test::publisher_instance_id(shm_name), 1);
  ASSERT_TRUE(mapping);

  auto initial = buffer_metadata::BufferRef::reserve_for_publish(mapping);
  ASSERT_TRUE(initial.has_value());
  ASSERT_TRUE(buffer_metadata::BufferRef::commit_publish(
      mapping, initial->slot_id, initial->generation));
  std::this_thread::sleep_for(std::chrono::milliseconds(105));

  for (int iteration = 0; iteration < 1000; ++iteration) {
    std::atomic<bool> start{false};
    std::optional<buffer_metadata::BufferRef::PublisherReservation> publisher;
    std::optional<buffer_metadata::BufferRef> subscriber;
    std::thread publisher_thread([&]() {
      while (!start.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
      publisher = buffer_metadata::BufferRef::reserve_for_publish(mapping);
    });
    std::thread subscriber_thread([&]() {
      while (!start.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
      subscriber.emplace(buffer_metadata::BufferRef::acquire(
          mapping, initial->slot_id, initial->generation));
    });
    start.store(true, std::memory_order_release);
    publisher_thread.join();
    subscriber_thread.join();

    EXPECT_FALSE(publisher.has_value() && subscriber->valid());
    if (publisher) {
      cancel(mapping, publisher);
    }
    subscriber.reset();
  }
  ::shm_unlink(shm_name.c_str());
}

TEST(BufferRefTest, OnlyOnePublisherCanReserveSingleSlot) {
  const std::string shm_name = make_unique_shm_name("buffer_ref_pub_race");
  auto mapping = buffer_metadata::BufferMetadata::create(
      shm_name, test::publisher_instance_id(shm_name), 1);
  ASSERT_TRUE(mapping);
  std::atomic<bool> start{false};
  std::optional<buffer_metadata::BufferRef::PublisherReservation> first;
  std::optional<buffer_metadata::BufferRef::PublisherReservation> second;
  std::thread a([&]() {
    while (!start.load(std::memory_order_acquire)) {
      std::this_thread::yield();
    }
    first = buffer_metadata::BufferRef::reserve_for_publish(mapping);
  });
  std::thread b([&]() {
    while (!start.load(std::memory_order_acquire)) {
      std::this_thread::yield();
    }
    second = buffer_metadata::BufferRef::reserve_for_publish(mapping);
  });
  start.store(true, std::memory_order_release);
  a.join();
  b.join();
  EXPECT_NE(first.has_value(), second.has_value());
  cancel(mapping, first);
  cancel(mapping, second);
  ::shm_unlink(shm_name.c_str());
}

}  // namespace ros2_cuda_ipc_core
