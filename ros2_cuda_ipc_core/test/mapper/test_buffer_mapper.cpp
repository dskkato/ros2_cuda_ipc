// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include <sys/mman.h>

#include <cstring>

#include "ros2_cuda_ipc_core/backend/vmm_fd/payload.hpp"
#include "ros2_cuda_ipc_core/subscriber/buffer_mapper.hpp"
#include "test_mapper_utils.hpp"

namespace ros2_cuda_ipc_core {

class BufferMapperTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { test::RclcppScope::SetUp(); }
  static void TearDownTestSuite() { test::RclcppScope::TearDown(); }
};

TEST_F(BufferMapperTest, LeaseFailureReturnsEmptyOptional) {
  const std::string shm_name = test::make_unique_shm_name("buffer_mapper_fail");
  auto mapping = lease::LeaseMapping::create(
      shm_name, test::publisher_instance_id(shm_name), 1);
  ASSERT_TRUE(mapping);

  ros2_cuda_ipc_msgs::msg::BufferCore msg;
  msg.shm_name = shm_name;
  msg.publisher_instance_id = test::publisher_instance_id(shm_name);
  msg.slot_id = 0;
  msg.device_id = 0;
  msg.generation = 99;
  msg.byte_size = 64;
  msg.backend = ros2_cuda_ipc_msgs::msg::BufferCore::CUDA_IPC;

  subscriber::BufferMapper mapper;
  EXPECT_FALSE(mapper.map(msg, CU_STREAM_LEGACY));

  ::shm_unlink(shm_name.c_str());
}

TEST_F(BufferMapperTest, PublisherInstanceMismatchRejectsMessage) {
  const std::string shm_name =
      test::make_unique_shm_name("buffer_mapper_instance");
  const auto owner_id = test::publisher_instance_id(shm_name);
  auto mapping = lease::LeaseMapping::create(shm_name, owner_id, 1);
  ASSERT_TRUE(mapping);
  auto reservation = lease::LeaseHandle::reserve_for_publish(mapping);
  ASSERT_TRUE(reservation.has_value());

  auto msg = test::make_cached_buffer_core_message(
      shm_name, reservation->slot_id, reservation->generation, 91);
  msg.publisher_instance_id =
      test::publisher_instance_id(shm_name + "_different");
  subscriber::BufferMapper mapper;
  EXPECT_FALSE(mapper.map(msg, CU_STREAM_LEGACY));
  auto refcnt = lease::LeaseHandle::current_refcount(mapping, 0);
  ASSERT_TRUE(refcnt.has_value());
  EXPECT_EQ(*refcnt, 1u);
  ASSERT_TRUE(lease::LeaseHandle::cancel_publish(mapping, reservation->slot_id,
                                                 reservation->generation));
  ::shm_unlink(shm_name.c_str());
}

TEST_F(BufferMapperTest, UnsupportedBackendReturnsEmptyOptional) {
  const std::string shm_name =
      test::make_unique_shm_name("buffer_mapper_backend");
  auto mapping = lease::LeaseMapping::create(
      shm_name, test::publisher_instance_id(shm_name), 1);
  ASSERT_TRUE(mapping);
  auto reservation = lease::LeaseHandle::reserve_for_publish(mapping);
  ASSERT_TRUE(reservation.has_value());

  ros2_cuda_ipc_msgs::msg::BufferCore msg;
  msg.shm_name = shm_name;
  msg.publisher_instance_id = test::publisher_instance_id(shm_name);
  msg.slot_id = 0;
  msg.device_id = 0;
  msg.generation = reservation->generation;
  msg.byte_size = 64;
  msg.backend = 255;

  subscriber::BufferMapper mapper;
  EXPECT_FALSE(mapper.map(msg, CU_STREAM_LEGACY));

  auto refcnt = lease::LeaseHandle::current_refcount(mapping, 0);
  ASSERT_TRUE(refcnt.has_value());
  EXPECT_EQ(refcnt.value(), 1u);
  ASSERT_TRUE(lease::LeaseHandle::cancel_publish(mapping, reservation->slot_id,
                                                 reservation->generation));

  ::shm_unlink(shm_name.c_str());
}

TEST_F(BufferMapperTest, InvalidVmmPayloadReturnsEmptyOptional) {
  const std::string shm_name =
      test::make_unique_shm_name("buffer_mapper_vmm_payload");
  auto mapping = lease::LeaseMapping::create(
      shm_name, test::publisher_instance_id(shm_name), 1);
  ASSERT_TRUE(mapping);
  auto reservation = lease::LeaseHandle::reserve_for_publish(mapping);
  ASSERT_TRUE(reservation.has_value());

  ros2_cuda_ipc_msgs::msg::BufferCore msg;
  msg.shm_name = shm_name;
  msg.publisher_instance_id = test::publisher_instance_id(shm_name);
  msg.slot_id = 0;
  msg.device_id = 0;
  msg.generation = reservation->generation;
  msg.byte_size = 64;
  msg.backend = ros2_cuda_ipc_msgs::msg::BufferCore::VMM_FD;
  msg.mem_handle.fill(0);
  msg.event_handle.fill(0);

  subscriber::BufferMapper mapper;
  EXPECT_FALSE(mapper.map(msg, CU_STREAM_LEGACY));

  auto refcnt = lease::LeaseHandle::current_refcount(mapping, 0);
  ASSERT_TRUE(refcnt.has_value());
  EXPECT_EQ(refcnt.value(), 1u);
  ASSERT_TRUE(lease::LeaseHandle::cancel_publish(mapping, reservation->slot_id,
                                                 reservation->generation));

  ::shm_unlink(shm_name.c_str());
}

TEST_F(BufferMapperTest, MissingVmmSocketReturnsEmptyAndReleasesLease) {
  const std::string shm_name =
      test::make_unique_shm_name("buffer_mapper_vmm_sock");
  auto mapping = lease::LeaseMapping::create(
      shm_name, test::publisher_instance_id(shm_name), 1);
  ASSERT_TRUE(mapping);
  auto reservation = lease::LeaseHandle::reserve_for_publish(mapping);
  ASSERT_TRUE(reservation.has_value());

  ros2_cuda_ipc_msgs::msg::BufferCore msg;
  msg.shm_name = shm_name;
  msg.publisher_instance_id = test::publisher_instance_id(shm_name);
  msg.slot_id = 0;
  msg.device_id = 0;
  msg.generation = reservation->generation;
  msg.byte_size = 64;
  msg.backend = ros2_cuda_ipc_msgs::msg::BufferCore::VMM_FD;
  msg.event_handle.fill(0);
  ASSERT_TRUE(backend::vmm_fd::encode_uuid_payload(
      "12345678-1234-5678-1234-567812345678", msg.mem_handle));

  subscriber::BufferMapper mapper;
  EXPECT_FALSE(mapper.map(msg, CU_STREAM_LEGACY));

  auto refcnt = lease::LeaseHandle::current_refcount(mapping, 0);
  ASSERT_TRUE(refcnt.has_value());
  EXPECT_EQ(refcnt.value(), 1u);
  ASSERT_TRUE(lease::LeaseHandle::cancel_publish(mapping, reservation->slot_id,
                                                 reservation->generation));

  ::shm_unlink(shm_name.c_str());
}

}  // namespace ros2_cuda_ipc_core
