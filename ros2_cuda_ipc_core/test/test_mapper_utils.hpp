// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <gtest/gtest.h>
#include <unistd.h>

#include <atomic>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros2_cuda_ipc_core/backend/memory_importer.hpp"
#include "ros2_cuda_ipc_core/buffer_metadata/buffer_ref.hpp"
#include "ros2_cuda_ipc_core/subscriber/ipc_handle_cache.hpp"
#include "ros2_cuda_ipc_msgs/msg/buffer_core.hpp"
#include "test_instance_id.hpp"

namespace ros2_cuda_ipc_core::test {

inline std::string make_unique_shm_name(const std::string& prefix) {
  static std::atomic<int> counter{0};
  std::ostringstream oss;
  oss << "/" << prefix << "_" << ::getpid() << "_" << counter.fetch_add(1);
  return oss.str();
}

class RclcppScope {
 public:
  static void SetUp() {
    if (!rclcpp::ok()) {
      int argc = 0;
      char** argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  static void TearDown() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

inline subscriber::IpcHandleKey make_key(
    const ros2_cuda_ipc_msgs::msg::BufferCore& msg) {
  subscriber::IpcHandleKey key{};
  key.publisher_instance_id = msg.publisher_instance_id;
  key.device_id = msg.device_id;
  key.vmm_socket_path = msg.vmm_socket_path;
  std::memcpy(key.event.data(), msg.event_handle.data(),
              msg.event_handle.size());
  return key;
}

inline ros2_cuda_ipc_msgs::msg::BufferCore make_cached_buffer_core_message(
    const std::string& shm_name, uint32_t block_id, uint32_t uid,
    uint8_t key_seed) {
  ros2_cuda_ipc_msgs::msg::BufferCore msg;
  msg.shm_name = shm_name;
  msg.publisher_instance_id = publisher_instance_id(shm_name);
  msg.device_id = 0;
  msg.block_id = block_id;
  msg.uid = uid;
  msg.byte_size = 64;
  std::ostringstream vmm_socket_path;
  vmm_socket_path << "/tmp/ros2_cuda_ipc_test_socket_" << std::hex
                  << std::setfill('0') << std::setw(4)
                  << static_cast<unsigned int>(key_seed) << ".sock";
  msg.vmm_socket_path = vmm_socket_path.str();
  msg.event_handle.fill(0);
  msg.event_handle[0] = static_cast<uint8_t>(key_seed + 1);
  return msg;
}

inline void seed_cache_for_message(
    const ros2_cuda_ipc_msgs::msg::BufferCore& msg, uintptr_t ptr_seed) {
  backend::ImportedResources imported;
  imported.dev_ptr = reinterpret_cast<void*>(ptr_seed);
  // Successful mapper fixtures model an already-ready publication.  Tests
  // that exercise a real producer event construct it explicitly in
  // test_read_handle.cpp.
  imported.event = nullptr;
  // The production cache strongly owns this synthetic resource until it is
  // explicitly cleared or the process exits.
  (void)subscriber::IpcHandleCache::instance().insert_or_discard_duplicate(
      make_key(msg), std::move(imported));
}

inline ros2_cuda_ipc_msgs::msg::BufferCore make_seeded_buffer_core_message(
    const std::string& prefix, uint8_t key_seed) {
  const std::string shm_name = make_unique_shm_name(prefix);
  const auto instance_id = publisher_instance_id(shm_name);
  auto mapping =
      buffer_metadata::BufferMetadata::create(shm_name, instance_id, 1);
  if (!mapping) {
    ADD_FAILURE() << "BufferMetadata::create failed for " << shm_name;
    return ros2_cuda_ipc_msgs::msg::BufferCore{};
  }
  auto reservation = buffer_metadata::BufferRef::reserve_for_publish(mapping);
  if (!reservation.has_value()) {
    ADD_FAILURE() << "BufferRef::reserve_for_publish failed for " << shm_name;
    return ros2_cuda_ipc_msgs::msg::BufferCore{};
  }
  auto msg = make_cached_buffer_core_message(shm_name, reservation->block_id,
                                             reservation->uid, key_seed);
  if (!buffer_metadata::BufferRef::commit_publish(
          mapping, reservation->block_id, reservation->uid)) {
    ADD_FAILURE() << "BufferRef::commit_publish failed for " << shm_name;
    (void)buffer_metadata::BufferRef::cancel_publish(
        mapping, reservation->block_id, reservation->uid);
    (void)::shm_unlink(shm_name.c_str());
    return ros2_cuda_ipc_msgs::msg::BufferCore{};
  }
  seed_cache_for_message(msg, static_cast<uintptr_t>(0x1000 + key_seed * 0x10));
  return msg;
}

}  // namespace ros2_cuda_ipc_core::test
