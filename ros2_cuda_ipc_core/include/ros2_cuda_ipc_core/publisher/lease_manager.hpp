// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/logger.hpp"
#include "ros2_cuda_ipc_core/publisher_instance_id.hpp"

namespace ros2_cuda_ipc_core::publisher {

class LeaseManager {
 public:
  struct Reservation {
    uint32_t slot_id = 0;
    uint32_t generation = 0;
    std::string shm_name;
    PublisherInstanceId publisher_instance_id{};
  };

  LeaseManager(std::string shm_name_prefix, std::size_t slot_count,
               std::chrono::milliseconds pending_ttl, rclcpp::Logger logger);

  ~LeaseManager();

  bool initialise();
  void reset() noexcept;
  bool is_initialised() const noexcept;
  std::optional<Reservation> reserve_for_publish(uint32_t pending_count);
  bool cancel(const Reservation& reservation) noexcept;
  void reclaim_stale_pending();

  std::string shm_name() const;
  PublisherInstanceId publisher_instance_id() const;
  std::chrono::steady_clock::time_point pending_deadline(
      uint32_t slot_id) const noexcept;

 private:
  std::string shm_name_prefix_;
  std::string shm_name_;
  PublisherInstanceId publisher_instance_id_{};
  std::size_t slot_count_;
  std::chrono::milliseconds pending_ttl_;
  rclcpp::Logger logger_;
  mutable std::mutex deadlines_mutex_;
  std::vector<std::chrono::steady_clock::time_point> pending_deadlines_;
  bool initialised_ = false;
};

}  // namespace ros2_cuda_ipc_core::publisher
