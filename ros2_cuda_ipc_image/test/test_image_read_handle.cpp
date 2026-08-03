#include <gtest/gtest.h>
#include <sys/mman.h>

#include <utility>

#include "ros2_cuda_ipc_core/subscriber/buffer_mapper.hpp"
#include "ros2_cuda_ipc_image/image_read_handle.hpp"
#include "test_mapper_utils.hpp"

namespace ros2_cuda_ipc_image {

class ImageReadHandleTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    ros2_cuda_ipc_core::test::RclcppScope::SetUp();
  }
  static void TearDownTestSuite() {
    ros2_cuda_ipc_core::test::RclcppScope::TearDown();
  }
};

TEST_F(ImageReadHandleTest, RejectsInvalidMetadataAndReleasesRead) {
  const auto core =
      ros2_cuda_ipc_core::test::make_seeded_buffer_core_message(41);
  const auto name = ros2_cuda_ipc_core::test::metadata_shm_name(core);
  auto mapping =
      ros2_cuda_ipc_core::buffer_metadata::BufferMetadata::attach(name);
  ASSERT_TRUE(mapping);
  auto read = (ros2_cuda_ipc_core::subscriber::BufferMapper{})
                  .map(core, CU_STREAM_LEGACY);
  ASSERT_TRUE(read);

  ros2_cuda_ipc_msgs::msg::GpuImage message;
  message.shape = {100, 100, 1};
  message.strides = {100, 1, 1};
  message.dtype = static_cast<uint8_t>(DType::U8);
  message.core = core;
  EXPECT_FALSE(ImageReadHandle::from_message(message, std::move(*read)));
  EXPECT_EQ(
      ros2_cuda_ipc_core::buffer_metadata::BufferRef::current_refcount(mapping),
      0u);
  ::shm_unlink(name.c_str());
}

TEST_F(ImageReadHandleTest, CopiesAndValidatesMessageMetadata) {
  const auto core =
      ros2_cuda_ipc_core::test::make_seeded_buffer_core_message(42);
  const auto name = ros2_cuda_ipc_core::test::metadata_shm_name(core);
  auto mapping =
      ros2_cuda_ipc_core::buffer_metadata::BufferMetadata::attach(name);
  ASSERT_TRUE(mapping);
  auto read = (ros2_cuda_ipc_core::subscriber::BufferMapper{})
                  .map(core, CU_STREAM_LEGACY);
  ASSERT_TRUE(read);

  ros2_cuda_ipc_msgs::msg::GpuImage message;
  message.header.frame_id = "camera_frame";
  message.dtype = static_cast<uint8_t>(DType::U16);
  message.shape = {2, 3, 4};
  message.strides = {24, 8, 2};
  message.encoding = "mono16";
  auto image = ImageReadHandle::from_message(message, std::move(*read));
  ASSERT_TRUE(image);
  EXPECT_EQ(image->header.frame_id, "camera_frame");
  EXPECT_TRUE(image->sanity_check());
  image.reset();
  EXPECT_EQ(
      ros2_cuda_ipc_core::buffer_metadata::BufferRef::current_refcount(mapping),
      0u);
  ::shm_unlink(name.c_str());
}

}  // namespace ros2_cuda_ipc_image
