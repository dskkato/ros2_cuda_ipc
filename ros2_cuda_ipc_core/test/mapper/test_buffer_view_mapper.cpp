#include <gtest/gtest.h>
#include <sys/mman.h>

#include <algorithm>
#include <cstring>

#include "../test_mapper_utils.hpp"
#include "ros2_cuda_ipc_core/lease_handle.hpp"
#include "ros2_cuda_ipc_core/mapper/buffer_view_mapper.hpp"
#include "ros2_cuda_ipc_core/memory_types.hpp"

namespace {

using ros2_cuda_ipc_core::MemoryBackendKind;
using ros2_cuda_ipc_core::mapper::BufferViewMapper;

class BufferViewMapperTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    ros2_cuda_ipc_core::test::RclcppScope::SetUp();
  }
  static void TearDownTestSuite() {
    ros2_cuda_ipc_core::test::RclcppScope::TearDown();
  }
};

TEST_F(BufferViewMapperTest, LeaseFailureReturnsInvalid) {
  const std::string shm_name =
      ros2_cuda_ipc_core::test::make_unique_shm_name("buffer_mapper_fail");
  ASSERT_TRUE(ros2_cuda_ipc_core::LeaseHandle::init(shm_name, 1));

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
      ros2_cuda_ipc_core::test::make_unique_shm_name("buffer_mapper_backend");
  ASSERT_TRUE(ros2_cuda_ipc_core::LeaseHandle::init(shm_name, 1));
  auto generation =
      ros2_cuda_ipc_core::LeaseHandle::bump_generation(shm_name, 0, 0);
  ASSERT_TRUE(generation.has_value());

  ros2_cuda_ipc_msgs::msg::BufferCore msg;
  msg.shm_name = shm_name;
  msg.slot_id = 0;
  msg.device_id = 0;
  msg.generation = generation.value();
  msg.byte_size = 64;
  msg.backend = 255;

  BufferViewMapper mapper;
  auto view = mapper.map(msg);
  EXPECT_FALSE(view.valid());

  auto refcnt = ros2_cuda_ipc_core::LeaseHandle::current_refcount(shm_name, 0);
  ASSERT_TRUE(refcnt.has_value());
  EXPECT_EQ(refcnt.value(), 0u);

  ::shm_unlink(shm_name.c_str());
}

TEST_F(BufferViewMapperTest, InvalidVmmPayloadReturnsInvalid) {
  const std::string shm_name = ros2_cuda_ipc_core::test::make_unique_shm_name(
      "buffer_mapper_vmm_payload");
  ASSERT_TRUE(ros2_cuda_ipc_core::LeaseHandle::init(shm_name, 1));
  auto generation =
      ros2_cuda_ipc_core::LeaseHandle::bump_generation(shm_name, 0, 1);
  ASSERT_TRUE(generation.has_value());

  ros2_cuda_ipc_msgs::msg::BufferCore msg;
  msg.shm_name = shm_name;
  msg.slot_id = 0;
  msg.device_id = 0;
  msg.generation = generation.value();
  msg.byte_size = 64;
  msg.backend = ros2_cuda_ipc_msgs::msg::BufferCore::VMM_FD;
  msg.mem_handle.fill(0);
  msg.event_handle.fill(0);

  BufferViewMapper mapper;
  auto view = mapper.map(msg);
  EXPECT_FALSE(view.valid());

  auto refcnt = ros2_cuda_ipc_core::LeaseHandle::current_refcount(shm_name, 0);
  ASSERT_TRUE(refcnt.has_value());
  EXPECT_EQ(refcnt.value(), 0u);

  ::shm_unlink(shm_name.c_str());
}

TEST_F(BufferViewMapperTest, MissingVmmSocketReturnsInvalidAndReleasesLease) {
  const std::string shm_name =
      ros2_cuda_ipc_core::test::make_unique_shm_name("buffer_mapper_vmm_sock");
  ASSERT_TRUE(ros2_cuda_ipc_core::LeaseHandle::init(shm_name, 1));
  auto generation =
      ros2_cuda_ipc_core::LeaseHandle::bump_generation(shm_name, 0, 1);
  ASSERT_TRUE(generation.has_value());

  ros2_cuda_ipc_msgs::msg::BufferCore msg;
  msg.shm_name = shm_name;
  msg.slot_id = 0;
  msg.device_id = 0;
  msg.generation = generation.value();
  msg.byte_size = 64;
  msg.backend = ros2_cuda_ipc_msgs::msg::BufferCore::VMM_FD;
  msg.event_handle.fill(0);
  ASSERT_TRUE(ros2_cuda_ipc_core::encode_uuid_payload(
      "12345678-1234-5678-1234-567812345678", msg.mem_handle));

  BufferViewMapper mapper;
  auto view = mapper.map(msg);
  EXPECT_FALSE(view.valid());

  auto refcnt = ros2_cuda_ipc_core::LeaseHandle::current_refcount(shm_name, 0);
  ASSERT_TRUE(refcnt.has_value());
  EXPECT_EQ(refcnt.value(), 0u);

  ::shm_unlink(shm_name.c_str());
}

TEST_F(BufferViewMapperTest, FillBufferCoreMessageCopiesCudaIpcHandles) {
  ros2_cuda_ipc_core::BufferView view;
  view.device_id = 2;
  view.byte_size = 64;
  view.slot_id = 5;
  view.generation = 9;
  view.shm_name = "/buf_meta";

  cudaIpcMemHandle_t mem_handle{};
  std::memset(&mem_handle, 0x12, sizeof(mem_handle));
  cudaIpcEventHandle_t event_handle{};
  std::memset(&event_handle, 0x34, sizeof(event_handle));
  view.set_ipc_handles(MemoryBackendKind::CUDA_IPC,
                       reinterpret_cast<const uint8_t*>(&mem_handle),
                       sizeof(mem_handle), event_handle);

  ros2_cuda_ipc_msgs::msg::BufferCore msg;
  ros2_cuda_ipc_core::mapper::fill_buffer_core_message(view, msg);

  EXPECT_EQ(msg.device_id, 2u);
  EXPECT_EQ(msg.byte_size, 64u);
  EXPECT_EQ(msg.slot_id, 5u);
  EXPECT_EQ(msg.generation, 9u);
  EXPECT_EQ(msg.shm_name, "/buf_meta");
  EXPECT_EQ(msg.backend, ros2_cuda_ipc_msgs::msg::BufferCore::CUDA_IPC);
  EXPECT_EQ(msg.mem_handle[0], 0x12);
  EXPECT_EQ(msg.event_handle[0], 0x34);
}

TEST_F(BufferViewMapperTest, FillBufferCoreMessagePreservesVmmPayload) {
  ros2_cuda_ipc_core::BufferView view;
  view.device_id = 3;
  view.byte_size = 128;
  view.slot_id = 7;
  view.generation = 11;
  view.shm_name = "/vmm_buf_meta";

  ros2_cuda_ipc_core::MemoryHandlePayload payload{};
  ASSERT_TRUE(
      ros2_cuda_ipc_core::encode_uuid_payload("test-vmm-handle", payload));

  cudaIpcEventHandle_t event_handle{};
  view.set_ipc_handles(MemoryBackendKind::VMM_FD, payload.data(),
                       payload.size(), event_handle);

  ros2_cuda_ipc_msgs::msg::BufferCore msg;
  ros2_cuda_ipc_core::mapper::fill_buffer_core_message(view, msg);

  EXPECT_EQ(msg.backend, ros2_cuda_ipc_msgs::msg::BufferCore::VMM_FD);
  EXPECT_TRUE(
      std::equal(payload.begin(), payload.end(), msg.mem_handle.begin()));
}

}  // namespace
