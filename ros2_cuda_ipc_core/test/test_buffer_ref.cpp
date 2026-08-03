// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <fcntl.h>
#include <gtest/gtest.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <sstream>
#include <thread>

#include "ros2_cuda_ipc_core/buffer_metadata/buffer_ref.hpp"

namespace {

std::string make_unique_shm_name(const char* suffix) {
  static std::atomic<uint32_t> counter{0};
  std::ostringstream name;
  name << "/buffer_ref_" << ::getpid() << "_" << suffix << "_"
       << counter.fetch_add(1);
  return name.str();
}

class ShmGuard {
 public:
  explicit ShmGuard(std::string name) : name_(std::move(name)) {}
  ~ShmGuard() { (void)::shm_unlink(name_.c_str()); }

 private:
  std::string name_;
};

}  // namespace

namespace ros2_cuda_ipc_core {

TEST(BufferRefTest, MappingContainsExactlyOneBlockMetadata) {
  const auto name = make_unique_shm_name("one");
  ShmGuard guard(name);
  auto mapping = buffer_metadata::BufferMetadata::create(name);
  ASSERT_TRUE(mapping);
  EXPECT_NE(mapping->metadata(), nullptr);

  const int fd = ::shm_open(name.c_str(), O_RDONLY, 0);
  ASSERT_NE(fd, -1);
  struct stat st{};
  ASSERT_EQ(::fstat(fd, &st), 0);
  EXPECT_EQ(st.st_size,
            static_cast<off_t>(sizeof(buffer_metadata::BlockMetadata)));
  ::close(fd);
}

TEST(BufferRefTest, AcquireReleaseAndRepeatedPublication) {
  const auto name = make_unique_shm_name("lifecycle");
  ShmGuard guard(name);
  auto mapping = buffer_metadata::BufferMetadata::create(name);
  ASSERT_TRUE(mapping);

  const auto first = buffer_metadata::BufferRef::reserve_for_publish(mapping);
  ASSERT_TRUE(first);
  ASSERT_TRUE(buffer_metadata::BufferRef::commit_publish(mapping, first->uid));
  {
    auto reference = buffer_metadata::BufferRef::acquire(mapping, first->uid);
    ASSERT_TRUE(reference.valid());
    EXPECT_EQ(buffer_metadata::BufferRef::current_refcount(mapping), 1u);
  }
  EXPECT_EQ(buffer_metadata::BufferRef::current_refcount(mapping), 0u);

  std::this_thread::sleep_for(std::chrono::milliseconds(105));
  const auto second = buffer_metadata::BufferRef::reserve_for_publish(mapping);
  ASSERT_TRUE(second);
  EXPECT_NE(second->uid, first->uid);
  ASSERT_TRUE(buffer_metadata::BufferRef::cancel_publish(mapping, second->uid));
}

TEST(BufferRefTest, StaleUidIsRejected) {
  const auto name = make_unique_shm_name("stale");
  ShmGuard guard(name);
  auto mapping = buffer_metadata::BufferMetadata::create(name);
  ASSERT_TRUE(mapping);
  const auto reservation =
      buffer_metadata::BufferRef::reserve_for_publish(mapping);
  ASSERT_TRUE(reservation);
  EXPECT_FALSE(
      buffer_metadata::BufferRef::acquire(mapping, reservation->uid + 1)
          .valid());
  ASSERT_TRUE(
      buffer_metadata::BufferRef::cancel_publish(mapping, reservation->uid));
}

TEST(BufferRefTest, PublisherAndSubscriberRaceIsUidSafe) {
  const auto name = make_unique_shm_name("race");
  ShmGuard guard(name);
  auto mapping = buffer_metadata::BufferMetadata::create(name);
  ASSERT_TRUE(mapping);
  const auto initial = buffer_metadata::BufferRef::reserve_for_publish(mapping);
  ASSERT_TRUE(initial);
  ASSERT_TRUE(
      buffer_metadata::BufferRef::commit_publish(mapping, initial->uid));
  std::this_thread::sleep_for(std::chrono::milliseconds(105));

  for (int iteration = 0; iteration < 100; ++iteration) {
    std::atomic<bool> start{false};
    std::optional<buffer_metadata::BufferRef::PublisherReservation> publisher;
    std::optional<buffer_metadata::BufferRef> subscriber;
    std::thread publisher_thread([&] {
      while (!start.load(std::memory_order_acquire)) std::this_thread::yield();
      publisher = buffer_metadata::BufferRef::reserve_for_publish(mapping);
    });
    std::thread subscriber_thread([&] {
      while (!start.load(std::memory_order_acquire)) std::this_thread::yield();
      subscriber.emplace(
          buffer_metadata::BufferRef::acquire(mapping, initial->uid));
    });
    start.store(true, std::memory_order_release);
    publisher_thread.join();
    subscriber_thread.join();
    EXPECT_FALSE(publisher.has_value() && subscriber->valid());
    if (publisher) {
      ASSERT_TRUE(
          buffer_metadata::BufferRef::cancel_publish(mapping, publisher->uid));
    }
    subscriber.reset();
  }
}

}  // namespace ros2_cuda_ipc_core
