// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include <sys/mman.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "ros2_cuda_ipc_core/buffer_metadata/buffer_ref.hpp"
#include "ros2_cuda_ipc_core/detail/buffer_metadata_cache.hpp"
#include "test_instance_id.hpp"

namespace {

std::string make_unique_shm_name(const std::string& prefix) {
  static std::atomic<int> counter{0};
  std::ostringstream oss;
  oss << "/" << prefix << "_" << ::getpid() << "_" << counter.fetch_add(1);
  return oss.str();
}

class ShmUnlinkGuard {
 public:
  explicit ShmUnlinkGuard(std::string shm_name)
      : shm_name_(std::move(shm_name)) {}

  ~ShmUnlinkGuard() { ::shm_unlink(shm_name_.c_str()); }

  ShmUnlinkGuard(const ShmUnlinkGuard&) = delete;
  ShmUnlinkGuard& operator=(const ShmUnlinkGuard&) = delete;

 private:
  std::string shm_name_;
};

struct FactoryState {
  ~FactoryState() {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto& shm_name : shm_names) {
      ::shm_unlink(shm_name.c_str());
    }
  }

  std::atomic<std::size_t> attach_count{0};
  std::atomic<std::size_t> destroy_count{0};
  std::atomic<std::size_t> observed_cache_size{999};
  std::atomic<ros2_cuda_ipc_core::subscriber::detail::BufferMetadataCache*>
      cache{nullptr};
  std::mutex mutex;
  std::vector<std::string> shm_names;
};

std::shared_ptr<ros2_cuda_ipc_core::buffer_metadata::BufferMetadata>
make_counted_mapping(
    const std::shared_ptr<FactoryState>& state,
    const ros2_cuda_ipc_core::PublisherInstanceId& publisher_instance_id,
    const std::string& prefix) {
  const std::string shm_name = make_unique_shm_name(prefix);
  auto owner = ros2_cuda_ipc_core::buffer_metadata::BufferMetadata::create(
      shm_name, publisher_instance_id, 1);
  if (!owner) {
    return nullptr;
  }
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->shm_names.push_back(shm_name);
  }

  auto* mapping = owner.get();
  return std::shared_ptr<ros2_cuda_ipc_core::buffer_metadata::BufferMetadata>(
      mapping,
      [state, owner = std::move(owner)](ros2_cuda_ipc_core::buffer_metadata::
                                            BufferMetadata*) mutable noexcept {
        if (auto* cache = state->cache.load()) {
          state->observed_cache_size.store(cache->size());
        }
        state->destroy_count.fetch_add(1);
        owner.reset();
      });
}

ros2_cuda_ipc_core::subscriber::detail::BufferMetadataCache::AttachFn
make_factory(const std::shared_ptr<FactoryState>& state,
             const std::string& prefix) {
  return [state, prefix](
             const std::string&,
             const ros2_cuda_ipc_core::PublisherInstanceId& instance_id) {
    state->attach_count.fetch_add(1);
    return make_counted_mapping(state, instance_id, prefix);
  };
}

}  // namespace

namespace ros2_cuda_ipc_core {

using subscriber::detail::BufferMetadataCache;

TEST(BufferMetadataCacheTest, ReusesMappingAcrossCallbackLocalBufferRefs) {
  auto state = std::make_shared<FactoryState>();
  BufferMetadataCache cache(make_factory(state, "buffer_ref_cache_reuse"));
  state->cache.store(&cache);

  const std::string logical_shm_name = "/logical_buffer_metadata_cache_reuse";
  const auto instance_id = test::publisher_instance_id(logical_shm_name);
  auto first = cache.get_or_attach(logical_shm_name, instance_id);
  ASSERT_TRUE(first);

  auto reservation = buffer_metadata::BufferRef::reserve_for_publish(first);
  ASSERT_TRUE(reservation.has_value());
  const auto block_id = reservation->block_id;
  const auto uid = reservation->uid;
  ASSERT_TRUE(buffer_metadata::BufferRef::commit_publish(first, block_id, uid));
  reservation.reset();

  for (int i = 0; i < 1000; ++i) {
    auto mapping = cache.get_or_attach(logical_shm_name, instance_id);
    ASSERT_TRUE(mapping);
    EXPECT_EQ(mapping.get(), first.get());
    {
      auto buffer_ref =
          buffer_metadata::BufferRef::acquire(mapping, block_id, uid);
      ASSERT_TRUE(buffer_ref.valid());
    }
  }

  EXPECT_EQ(state->attach_count.load(), 1u);
  EXPECT_EQ(cache.size(), 1u);
  EXPECT_EQ(state->destroy_count.load(), 0u);

  first.reset();
  cache.clear();
  EXPECT_EQ(cache.size(), 0u);
  EXPECT_EQ(state->destroy_count.load(), 1u);
  EXPECT_EQ(state->observed_cache_size.load(), 0u);
}

TEST(BufferMetadataCacheTest, ClearReleasesMappingWithoutActiveBufferRef) {
  auto state = std::make_shared<FactoryState>();
  BufferMetadataCache cache(make_factory(state, "buffer_ref_cache_clear"));
  state->cache.store(&cache);

  const auto instance_id = test::publisher_instance_id("clear");
  auto mapping =
      cache.get_or_attach("/logical_buffer_metadata_cache_clear", instance_id);
  ASSERT_TRUE(mapping);
  std::weak_ptr<buffer_metadata::BufferMetadata> weak_mapping = mapping;
  mapping.reset();

  cache.clear();

  EXPECT_EQ(cache.size(), 0u);
  EXPECT_TRUE(weak_mapping.expired());
  EXPECT_EQ(state->destroy_count.load(), 1u);
  EXPECT_EQ(state->observed_cache_size.load(), 0u);
}

TEST(BufferMetadataCacheTest, ClearPreservesMappingForActiveBufferRef) {
  auto state = std::make_shared<FactoryState>();
  BufferMetadataCache cache(make_factory(state, "buffer_ref_cache_active"));
  state->cache.store(&cache);

  const auto instance_id = test::publisher_instance_id("active");
  auto mapping =
      cache.get_or_attach("/logical_buffer_metadata_cache_active", instance_id);
  ASSERT_TRUE(mapping);
  auto reservation = buffer_metadata::BufferRef::reserve_for_publish(mapping);
  ASSERT_TRUE(reservation.has_value());
  const auto block_id = reservation->block_id;
  const auto uid = reservation->uid;
  ASSERT_TRUE(
      buffer_metadata::BufferRef::commit_publish(mapping, block_id, uid));
  reservation.reset();

  std::weak_ptr<buffer_metadata::BufferMetadata> weak_mapping = mapping;

  {
    auto active_buffer_ref =
        buffer_metadata::BufferRef::acquire(mapping, block_id, uid);
    ASSERT_TRUE(active_buffer_ref.valid());
    mapping.reset();
    cache.clear();

    EXPECT_EQ(cache.size(), 0u);
    EXPECT_FALSE(weak_mapping.expired());
    EXPECT_TRUE(active_buffer_ref.valid());
    EXPECT_EQ(state->destroy_count.load(), 0u);
    EXPECT_EQ(state->observed_cache_size.load(), 999u);
  }

  EXPECT_TRUE(weak_mapping.expired());
  EXPECT_EQ(state->destroy_count.load(), 1u);
}

TEST(BufferMetadataCacheTest, ConcurrentDuplicateAttachKeepsOneEntry) {
  constexpr std::size_t kThreadCount = 8;
  auto state = std::make_shared<FactoryState>();
  std::atomic<std::size_t> attach_started{0};
  std::atomic<bool> start{false};
  std::atomic<bool> allow_attach{false};
  BufferMetadataCache cache(
      [state, &attach_started, &start, &allow_attach](
          const std::string&, const PublisherInstanceId& instance_id) {
        while (!start.load()) {
          std::this_thread::yield();
        }
        state->attach_count.fetch_add(1);
        attach_started.fetch_add(1);
        while (!allow_attach.load()) {
          std::this_thread::yield();
        }
        return make_counted_mapping(state, instance_id,
                                    "buffer_ref_cache_duplicate");
      });
  state->cache.store(&cache);

  const std::string logical_shm_name =
      "/logical_buffer_metadata_cache_duplicate";
  const auto instance_id = test::publisher_instance_id(logical_shm_name);
  std::vector<std::shared_ptr<buffer_metadata::BufferMetadata>> returned(
      kThreadCount);
  std::vector<std::thread> threads;
  threads.reserve(kThreadCount);
  for (std::size_t i = 0; i < kThreadCount; ++i) {
    threads.emplace_back([&, i] {
      returned[i] = cache.get_or_attach(logical_shm_name, instance_id);
    });
  }

  start.store(true);
  while (attach_started.load() != kThreadCount) {
    std::this_thread::yield();
  }
  allow_attach.store(true);

  for (auto& thread : threads) {
    thread.join();
  }

  ASSERT_EQ(cache.size(), 1u);
  ASSERT_TRUE(returned.front());
  for (const auto& mapping : returned) {
    ASSERT_TRUE(mapping);
    EXPECT_EQ(mapping.get(), returned.front().get());
  }
  EXPECT_EQ(state->attach_count.load(), kThreadCount);
  EXPECT_EQ(state->destroy_count.load(), kThreadCount - 1);
  EXPECT_EQ(state->observed_cache_size.load(), 1u);

  returned.clear();
  EXPECT_EQ(state->destroy_count.load(), kThreadCount - 1);
  cache.clear();
  EXPECT_EQ(state->destroy_count.load(), kThreadCount);
  EXPECT_EQ(state->observed_cache_size.load(), 0u);
}

TEST(BufferMetadataCacheTest, DifferentPublisherInstancesUseDifferentEntries) {
  auto state = std::make_shared<FactoryState>();
  BufferMetadataCache cache(make_factory(state, "buffer_ref_cache_instances"));
  state->cache.store(&cache);

  const std::string shm_name = "/logical_buffer_metadata_cache_instances";
  const auto first_id = test::publisher_instance_id("instance-first");
  const auto second_id = test::publisher_instance_id("instance-second");
  auto first = cache.get_or_attach(shm_name, first_id);
  auto second = cache.get_or_attach(shm_name, second_id);
  ASSERT_TRUE(first);
  ASSERT_TRUE(second);

  EXPECT_NE(first.get(), second.get());
  EXPECT_EQ(cache.size(), 2u);
  EXPECT_EQ(state->attach_count.load(), 2u);
  EXPECT_EQ(cache.get_or_attach(shm_name, first_id).get(), first.get());
}

TEST(BufferMetadataCacheTest, AttachFailureDoesNotPopulateCache) {
  std::atomic<std::size_t> attach_count{0};
  BufferMetadataCache cache(
      [&attach_count](const std::string&, const PublisherInstanceId&) {
        attach_count.fetch_add(1);
        return std::shared_ptr<buffer_metadata::BufferMetadata>{};
      });

  const auto instance_id = test::publisher_instance_id("failure");
  EXPECT_FALSE(
      cache.get_or_attach("/missing_buffer_metadata_cache", instance_id));
  EXPECT_FALSE(
      cache.get_or_attach("/missing_buffer_metadata_cache", instance_id));
  EXPECT_EQ(cache.size(), 0u);
  EXPECT_EQ(attach_count.load(), 2u);
}

TEST(BufferMetadataCacheTest, AttachExceptionLeavesCacheUnchanged) {
  BufferMetadataCache cache(
      [](const std::string&, const PublisherInstanceId&)
          -> std::shared_ptr<buffer_metadata::BufferMetadata> {
        throw std::runtime_error("synthetic attach failure");
      });

  EXPECT_THROW(cache.get_or_attach("/exception_buffer_metadata_cache",
                                   test::publisher_instance_id("exception")),
               std::runtime_error);
  EXPECT_EQ(cache.size(), 0u);
}

TEST(BufferMetadataCacheTest, ReportsMissAndHitLatency) {
  const std::string shm_name = make_unique_shm_name("buffer_ref_cache_timing");
  const ShmUnlinkGuard shm_unlink_guard(shm_name);
  const auto instance_id = test::publisher_instance_id(shm_name);
  auto publisher_mapping =
      buffer_metadata::BufferMetadata::create(shm_name, instance_id, 1);
  ASSERT_TRUE(publisher_mapping);

  BufferMetadataCache cache;
  const auto miss_start = std::chrono::steady_clock::now();
  auto first = cache.get_or_attach(shm_name, instance_id);
  const auto miss_end = std::chrono::steady_clock::now();
  ASSERT_TRUE(first);

  constexpr std::size_t kHitCount = 10000;
  std::vector<double> hit_ns;
  hit_ns.reserve(kHitCount);
  for (std::size_t i = 0; i < kHitCount; ++i) {
    const auto start = std::chrono::steady_clock::now();
    auto mapping = cache.get_or_attach(shm_name, instance_id);
    const auto end = std::chrono::steady_clock::now();
    ASSERT_TRUE(mapping);
    ASSERT_EQ(mapping.get(), first.get());
    hit_ns.push_back(
        std::chrono::duration<double, std::nano>(end - start).count());
  }

  const auto miss_ns =
      std::chrono::duration<double, std::nano>(miss_end - miss_start).count();
  std::sort(hit_ns.begin(), hit_ns.end());
  const auto percentile = [&hit_ns](double quantile) {
    const auto index = static_cast<std::size_t>(
        quantile * static_cast<double>(hit_ns.size() - 1));
    return hit_ns[index];
  };
  const double hit_average =
      std::accumulate(hit_ns.begin(), hit_ns.end(), 0.0) /
      static_cast<double>(hit_ns.size());

  std::cout << "[timing] BufferMetadataCache get_or_attach miss count=1"
            << " average=" << miss_ns << "ns"
            << " p50=" << miss_ns << "ns"
            << " p95=" << miss_ns << "ns"
            << " p99=" << miss_ns << "ns"
            << " maximum=" << miss_ns << "ns\n"
            << "[timing] BufferMetadataCache get_or_attach hit count="
            << kHitCount << " average=" << hit_average << "ns"
            << " p50=" << percentile(0.50) << "ns"
            << " p95=" << percentile(0.95) << "ns"
            << " p99=" << percentile(0.99) << "ns"
            << " maximum=" << hit_ns.back() << "ns\n";

  EXPECT_EQ(cache.size(), 1u);
  cache.clear();
  publisher_mapping.reset();
}

}  // namespace ros2_cuda_ipc_core
