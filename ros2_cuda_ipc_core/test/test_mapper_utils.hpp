// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <gtest/gtest.h>
#include <unistd.h>

#include <atomic>
#include <cstring>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros2_cuda_ipc_core/ipc_handle_cache.hpp"
#include "ros2_cuda_ipc_core/lease_handle.hpp"
#include "ros2_cuda_ipc_msgs/msg/buffer_core.hpp"

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

inline IpcHandleKey make_key(const ros2_cuda_ipc_msgs::msg::BufferCore& msg) {
  IpcHandleKey key{};
  key.backend = static_cast<uint8_t>(msg.backend);
  key.mem = msg.mem_handle;
  std::memcpy(key.event.data(), msg.event_handle.data(),
              msg.event_handle.size());
  return key;
}

inline ros2_cuda_ipc_msgs::msg::BufferCore make_cached_buffer_core_message(
    const std::string& shm_name, uint32_t slot_id, uint32_t generation,
    uint8_t key_seed, MemoryBackendKind backend = MemoryBackendKind::CUDA_IPC) {
  ros2_cuda_ipc_msgs::msg::BufferCore msg;
  msg.shm_name = shm_name;
  msg.device_id = 0;
  msg.slot_id = slot_id;
  msg.generation = generation;
  msg.byte_size = 64;
  msg.backend = to_backend_byte(backend);
  msg.mem_handle.fill(0);
  msg.event_handle.fill(0);
  msg.mem_handle[0] = key_seed;
  msg.event_handle[0] = static_cast<uint8_t>(key_seed + 1);
  return msg;
}

inline void seed_cache_for_message(
    const ros2_cuda_ipc_msgs::msg::BufferCore& msg, uintptr_t ptr_seed) {
  cuda::ImportedMemory imported;
  imported.dev_ptr = reinterpret_cast<void*>(ptr_seed);
  imported.event = reinterpret_cast<cudaEvent_t>(ptr_seed + 1U);
  IpcHandleCache::instance().insert_or_discard_duplicate(make_key(msg),
                                                         imported);
}

inline ros2_cuda_ipc_msgs::msg::BufferCore make_seeded_buffer_core_message(
    const std::string& prefix, uint8_t key_seed) {
  const std::string shm_name = make_unique_shm_name(prefix);
  if (!LeaseHandle::init(shm_name, 1)) {
    ADD_FAILURE() << "LeaseHandle::init failed for " << shm_name;
    return ros2_cuda_ipc_msgs::msg::BufferCore{};
  }
  auto generation = LeaseHandle::bump_generation(shm_name, 0, 1);
  if (!generation.has_value()) {
    ADD_FAILURE() << "LeaseHandle::bump_generation failed for " << shm_name;
    return ros2_cuda_ipc_msgs::msg::BufferCore{};
  }
  auto msg = make_cached_buffer_core_message(shm_name, 0, generation.value(),
                                             key_seed);
  seed_cache_for_message(msg, static_cast<uintptr_t>(0x1000 + key_seed * 0x10));
  return msg;
}

}  // namespace ros2_cuda_ipc_core::test
