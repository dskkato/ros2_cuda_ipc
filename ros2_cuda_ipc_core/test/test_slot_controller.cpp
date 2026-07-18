// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include <sys/mman.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "ros2_cuda_ipc_core/lease/lease_handle.hpp"
#include "ros2_cuda_ipc_core/publisher/slot_controller.hpp"

namespace {

std::string make_unique_shm_name() {
  static std::atomic<int> counter{0};
  std::ostringstream out;
  out << "/slot_controller_" << ::getpid() << "_" << counter.fetch_add(1);
  return out.str();
}

class ShmUnlinkGuard {
 public:
  explicit ShmUnlinkGuard(std::string name) : name_(std::move(name)) {}
  ~ShmUnlinkGuard() { ::shm_unlink(name_.c_str()); }

  ShmUnlinkGuard(const ShmUnlinkGuard&) = delete;
  ShmUnlinkGuard& operator=(const ShmUnlinkGuard&) = delete;

 private:
  std::string name_;
};

}  // namespace

TEST(SlotControllerTest, ResetRacingWithReserveDoesNotLeavePending) {
  const auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  for (int iteration = 0; iteration < 1000; ++iteration) {
    const std::string shm_name = make_unique_shm_name();
    const ShmUnlinkGuard shm_guard(shm_name);
    ros2_cuda_ipc_core::publisher::SlotController controller(
        shm_name, 1, std::chrono::milliseconds(100),
        rclcpp::get_logger("SlotControllerTest"), clock);
    ASSERT_TRUE(controller.initialise());

    std::atomic<bool> start{false};
    std::optional<ros2_cuda_ipc_core::publisher::SlotController::Reservation>
        reservation;
    std::thread reserve_thread([&]() {
      while (!start.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
      reservation = controller.reserve_for_publish(1);
    });
    std::thread reset_thread([&]() {
      while (!start.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
      controller.reset();
    });

    start.store(true, std::memory_order_release);
    reserve_thread.join();
    reset_thread.join();

    if (reservation) {
      controller.cancel(*reservation);
    }
    const auto pending =
        ros2_cuda_ipc_core::lease::LeaseHandle::current_pending(shm_name, 0);
    ASSERT_TRUE(pending.has_value());
    EXPECT_EQ(*pending, 0u);
  }
}

TEST(SlotControllerTest, CapacityMismatchRollsBackOutOfRangeReservation) {
  const std::string shm_name = make_unique_shm_name();
  const ShmUnlinkGuard shm_guard(shm_name);
  const auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  ros2_cuda_ipc_core::publisher::SlotController controller(
      shm_name, 1, std::chrono::milliseconds(100),
      rclcpp::get_logger("SlotControllerTest"), clock);
  ASSERT_TRUE(controller.initialise());

  // Simulate an unsupported second Publisher reinitialising the same name
  // with a different capacity, then occupy slot 0 so the local controller
  // observes an out-of-range reservation for slot 1.
  ASSERT_TRUE(ros2_cuda_ipc_core::lease::LeaseHandle::init(shm_name, 2));
  const auto occupied =
      ros2_cuda_ipc_core::lease::LeaseHandle::reserve_for_publish(shm_name, 1);
  ASSERT_TRUE(occupied.has_value());
  ASSERT_EQ(occupied->slot_id, 0u);

  EXPECT_FALSE(controller.reserve_for_publish(1).has_value());
  const auto out_of_range_pending =
      ros2_cuda_ipc_core::lease::LeaseHandle::current_pending(shm_name, 1);
  ASSERT_TRUE(out_of_range_pending.has_value());
  EXPECT_EQ(*out_of_range_pending, 0u);

  EXPECT_TRUE(ros2_cuda_ipc_core::lease::LeaseHandle::cancel_pending(
      shm_name, occupied->slot_id, occupied->generation));
}

TEST(SlotControllerTest, ReclaimsPendingUsingInjectedRosClock) {
  const std::string shm_name = make_unique_shm_name();
  const ShmUnlinkGuard shm_guard(shm_name);
  const auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  ASSERT_EQ(rcl_enable_ros_time_override(clock->get_clock_handle()),
            RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), 0),
            RCL_RET_OK);

  {
    ros2_cuda_ipc_core::publisher::SlotController controller(
        shm_name, 1, std::chrono::milliseconds(100),
        rclcpp::get_logger("SlotControllerTest"), clock);
    ASSERT_TRUE(controller.initialise());
    auto reservation = controller.reserve_for_publish(1);
    ASSERT_TRUE(reservation.has_value());

    controller.reclaim_stale_pending();
    EXPECT_FALSE(controller.reserve_for_publish(0).has_value());

    ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), 100000000),
              RCL_RET_OK);
    controller.reclaim_stale_pending();
    EXPECT_TRUE(controller.reserve_for_publish(0).has_value());
  }

  EXPECT_EQ(rcl_disable_ros_time_override(clock->get_clock_handle()),
            RCL_RET_OK);
}

TEST(SlotControllerTest, RejectsNullClock) {
  EXPECT_THROW(ros2_cuda_ipc_core::publisher::SlotController(
                   make_unique_shm_name(), 1, std::chrono::milliseconds(100),
                   rclcpp::get_logger("SlotControllerTest"), nullptr),
               std::invalid_argument);
}
