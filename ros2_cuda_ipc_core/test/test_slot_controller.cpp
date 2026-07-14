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

#include "rclcpp/rclcpp.hpp"
#include "ros2_cuda_ipc_core/lease_handle.hpp"
#include "ros2_cuda_ipc_core/slot_controller.hpp"

namespace {

std::string make_unique_shm_name() {
  static std::atomic<int> counter{0};
  std::ostringstream out;
  out << "/slot_controller_" << ::getpid() << "_" << counter.fetch_add(1);
  return out.str();
}

}  // namespace

TEST(SlotControllerTest, ResetRacingWithReserveDoesNotLeavePending) {
  for (int iteration = 0; iteration < 1000; ++iteration) {
    const std::string shm_name = make_unique_shm_name();
    ros2_cuda_ipc_core::SlotController controller(
        shm_name, 1, std::chrono::milliseconds(100),
        rclcpp::get_logger("SlotControllerTest"));
    ASSERT_TRUE(controller.initialise());

    std::atomic<bool> start{false};
    std::optional<ros2_cuda_ipc_core::SlotController::Reservation> reservation;
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
        ros2_cuda_ipc_core::LeaseHandle::current_pending(shm_name, 0);
    ASSERT_TRUE(pending.has_value());
    EXPECT_EQ(*pending, 0u);
    ::shm_unlink(shm_name.c_str());
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
