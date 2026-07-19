// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "rclcpp/logging.hpp"
#include "ros2_cuda_ipc_core/lease/lease_mapping.hpp"
#include "ros2_cuda_ipc_core/subscriber/buffer_view.hpp"
#include "ros2_cuda_ipc_msgs/msg/buffer_core.hpp"

namespace ros2_cuda_ipc_core::subscriber {

struct BufferViewMapperOptions {
  rclcpp::Logger logger =
      rclcpp::get_logger("ros2_cuda_ipc_core.BufferViewMapper");
};

/// Reuses subscriber mappings without extending their lifetime.
class LeaseMappingCache {
 public:
  std::shared_ptr<lease::LeaseMapping> get_or_attach(
      const std::string& shm_name,
      const PublisherInstanceId& publisher_instance_id) const;

 private:
  struct Key {
    std::string shm_name;
    PublisherInstanceId publisher_instance_id{};

    bool operator==(const Key& other) const noexcept {
      return shm_name == other.shm_name &&
             publisher_instance_id == other.publisher_instance_id;
    }
  };

  struct KeyHash {
    std::size_t operator()(const Key& key) const noexcept;
  };

  mutable std::mutex mutex_;
  mutable std::unordered_map<Key, std::weak_ptr<lease::LeaseMapping>, KeyHash>
      mappings_;
};

class BufferViewMapper {
 public:
  explicit BufferViewMapper(BufferViewMapperOptions options = {});

  BufferView map(const ros2_cuda_ipc_msgs::msg::BufferCore& msg) const;

 private:
  BufferViewMapperOptions options_;
  std::shared_ptr<LeaseMappingCache> mapping_cache_;
};

BufferView map_buffer_view(const ros2_cuda_ipc_msgs::msg::BufferCore& msg);

}  // namespace ros2_cuda_ipc_core::subscriber
