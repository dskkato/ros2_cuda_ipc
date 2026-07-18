// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include <sys/mman.h>

#include <algorithm>
#include <cstring>

#include "../test_mapper_utils.hpp"
#include "ros2_cuda_ipc_core/backend/vmm_fd/payload.hpp"
#include "ros2_cuda_ipc_core/lease/lease_handle.hpp"
#include "ros2_cuda_ipc_core/subscriber/buffer_view_mapper.hpp"
#include "ros2_cuda_ipc_core/transport/memory_types.hpp"

namespace ros2_cuda_ipc_core {

using subscriber::BufferViewMapper;

class BufferViewMapperTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { test::RclcppScope::SetUp(); }
  static void TearDownTestSuite() { test::RclcppScope::TearDown(); }
};

TEST_F(BufferViewMapperTest, LeaseFailureReturnsInvalid) {
  const std::string shm_name = test::make_unique_shm_name("buffer_mapper_fail");
  ASSERT_TRUE(lease::LeaseHandle::init(shm_name, 1));

  ros2_cuda_ipc_msgs::msg::BufferCore msg;
  msg.shm_name = shm_name;
  msg.slot_id = 0;
  msg.device_id = 0;
  msg.generation = 99;
  msg.byte_size = 64;
  msg.backend = ros2_cuda_ipc_msgs::msg::BufferCore::CUDA_IPC;

  BufferViewMapper mapper;
  auto view = mapper.map(msg);
  EXPECT_FALSE(view.valid());

  ::shm_unlink(shm_name.c_str());
}

TEST_F(BufferViewMapperTest, UnsupportedBackendReturnsInvalid) {
  const std::string shm_name =
      test::make_unique_shm_name("buffer_mapper_backend");
  ASSERT_TRUE(lease::LeaseHandle::init(shm_name, 1));
  auto reservation = lease::LeaseHandle::reserve_for_publish(shm_name, 0);
  ASSERT_TRUE(reservation.has_value());

  ros2_cuda_ipc_msgs::msg::BufferCore msg;
  msg.shm_name = shm_name;
  msg.slot_id = 0;
  msg.device_id = 0;
  msg.generation = reservation->generation;
  msg.byte_size = 64;
  msg.backend = 255;

  BufferViewMapper mapper;
  auto view = mapper.map(msg);
  EXPECT_FALSE(view.valid());

  auto refcnt = lease::LeaseHandle::current_refcount(shm_name, 0);
  ASSERT_TRUE(refcnt.has_value());
  EXPECT_EQ(refcnt.value(), 0u);

  ::shm_unlink(shm_name.c_str());
}

TEST_F(BufferViewMapperTest, InvalidVmmPayloadReturnsInvalid) {
  const std::string shm_name =
      test::make_unique_shm_name("buffer_mapper_vmm_payload");
  ASSERT_TRUE(lease::LeaseHandle::init(shm_name, 1));
  auto reservation = lease::LeaseHandle::reserve_for_publish(shm_name, 1);
  ASSERT_TRUE(reservation.has_value());

  ros2_cuda_ipc_msgs::msg::BufferCore msg;
  msg.shm_name = shm_name;
  msg.slot_id = 0;
  msg.device_id = 0;
  msg.generation = reservation->generation;
  msg.byte_size = 64;
  msg.backend = ros2_cuda_ipc_msgs::msg::BufferCore::VMM_FD;
  msg.mem_handle.fill(0);
  msg.event_handle.fill(0);

  BufferViewMapper mapper;
  auto view = mapper.map(msg);
  EXPECT_FALSE(view.valid());

  auto refcnt = lease::LeaseHandle::current_refcount(shm_name, 0);
  ASSERT_TRUE(refcnt.has_value());
  EXPECT_EQ(refcnt.value(), 0u);

  ::shm_unlink(shm_name.c_str());
}

TEST_F(BufferViewMapperTest, MissingVmmSocketReturnsInvalidAndReleasesLease) {
  const std::string shm_name =
      test::make_unique_shm_name("buffer_mapper_vmm_sock");
  ASSERT_TRUE(lease::LeaseHandle::init(shm_name, 1));
  auto reservation = lease::LeaseHandle::reserve_for_publish(shm_name, 1);
  ASSERT_TRUE(reservation.has_value());

  ros2_cuda_ipc_msgs::msg::BufferCore msg;
  msg.shm_name = shm_name;
  msg.slot_id = 0;
  msg.device_id = 0;
  msg.generation = reservation->generation;
  msg.byte_size = 64;
  msg.backend = ros2_cuda_ipc_msgs::msg::BufferCore::VMM_FD;
  msg.event_handle.fill(0);
  ASSERT_TRUE(backend::vmm_fd::encode_uuid_payload(
      "12345678-1234-5678-1234-567812345678", msg.mem_handle));

  BufferViewMapper mapper;
  auto view = mapper.map(msg);
  EXPECT_FALSE(view.valid());

  auto refcnt = lease::LeaseHandle::current_refcount(shm_name, 0);
  ASSERT_TRUE(refcnt.has_value());
  EXPECT_EQ(refcnt.value(), 0u);

  ::shm_unlink(shm_name.c_str());
}

}  // namespace ros2_cuda_ipc_core
