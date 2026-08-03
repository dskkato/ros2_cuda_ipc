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
#include "ros2_cuda_ipc_core/publisher/buffer_metadata_manager.hpp"
#include "ros2_cuda_ipc_core/subscriber/ipc_handle_cache.hpp"
#include "ros2_cuda_ipc_msgs/msg/buffer_core.hpp"

namespace ros2_cuda_ipc_core::test {

inline uint32_t next_test_block_id() {
  static std::atomic<uint32_t> counter{1000000};
  return counter.fetch_add(1);
}

inline uint32_t test_publisher_pid() {
  return static_cast<uint32_t>(::getpid());
}

inline std::string metadata_shm_name(
    const ros2_cuda_ipc_msgs::msg::BufferCore& msg) {
  return publisher::BufferMetadataManager::shm_name_for_block(msg.publisher_pid,
                                                              msg.block_id);
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
    if (rclcpp::ok()) rclcpp::shutdown();
  }
};

inline subscriber::IpcHandleKey make_key(
    const ros2_cuda_ipc_msgs::msg::BufferCore& msg) {
  subscriber::IpcHandleKey key{};
  key.publisher_pid = msg.publisher_pid;
  key.block_id = msg.block_id;
  key.device_id = msg.device_id;
  key.vmm_socket_path = msg.vmm_socket_path;
  std::memcpy(key.event.data(), msg.event_handle.data(),
              msg.event_handle.size());
  return key;
}

inline ros2_cuda_ipc_msgs::msg::BufferCore make_cached_buffer_core_message(
    uint32_t publisher_pid, uint32_t block_id, uint64_t uid, uint8_t key_seed) {
  ros2_cuda_ipc_msgs::msg::BufferCore msg;
  msg.publisher_pid = publisher_pid;
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
  imported.event = nullptr;
  (void)subscriber::IpcHandleCache::instance().insert_or_discard_duplicate(
      make_key(msg), std::move(imported));
}

inline ros2_cuda_ipc_msgs::msg::BufferCore make_seeded_buffer_core_message(
    uint8_t key_seed) {
  const uint32_t publisher_pid = test_publisher_pid();
  const uint32_t block_id = next_test_block_id();
  const std::string shm_name =
      publisher::BufferMetadataManager::shm_name_for_block(publisher_pid,
                                                           block_id);
  auto mapping = buffer_metadata::BufferMetadata::create(shm_name);
  if (!mapping) {
    ADD_FAILURE() << "BufferMetadata::create failed for " << shm_name;
    return ros2_cuda_ipc_msgs::msg::BufferCore{};
  }
  auto reservation = buffer_metadata::BufferRef::reserve_for_publish(mapping);
  if (!reservation) {
    ADD_FAILURE() << "BufferRef::reserve_for_publish failed for " << shm_name;
    return ros2_cuda_ipc_msgs::msg::BufferCore{};
  }
  auto msg = make_cached_buffer_core_message(publisher_pid, block_id,
                                             reservation->uid, key_seed);
  if (!buffer_metadata::BufferRef::commit_publish(mapping, reservation->uid)) {
    ADD_FAILURE() << "BufferRef::commit_publish failed for " << shm_name;
    (void)buffer_metadata::BufferRef::cancel_publish(mapping, reservation->uid);
    (void)::shm_unlink(shm_name.c_str());
    return ros2_cuda_ipc_msgs::msg::BufferCore{};
  }
  seed_cache_for_message(msg, static_cast<uintptr_t>(0x1000 + key_seed * 0x10));
  return msg;
}

}  // namespace ros2_cuda_ipc_core::test
