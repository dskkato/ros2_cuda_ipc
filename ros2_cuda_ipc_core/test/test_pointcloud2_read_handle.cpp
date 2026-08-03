#include <gtest/gtest.h>
#include <sys/mman.h>

#include <utility>

#include "ros2_cuda_ipc_core/pointcloud2/pointcloud2_read_handle.hpp"
#include "ros2_cuda_ipc_core/subscriber/buffer_mapper.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "test_mapper_utils.hpp"

namespace ros2_cuda_ipc_core {

class PointCloud2ReadHandleTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { test::RclcppScope::SetUp(); }
  static void TearDownTestSuite() { test::RclcppScope::TearDown(); }
};

TEST_F(PointCloud2ReadHandleTest, CopiesAndValidatesMessageMetadata) {
  const auto core = test::make_seeded_buffer_core_message(43);
  const auto name = test::metadata_shm_name(core);
  auto mapping = buffer_metadata::BufferMetadata::attach(name);
  ASSERT_TRUE(mapping);
  auto read = (subscriber::BufferMapper{}).map(core, CU_STREAM_LEGACY);
  ASSERT_TRUE(read);

  ros2_cuda_ipc_msgs::msg::GpuPointCloud2 message;
  message.header.frame_id = "lidar";
  message.height = 1;
  message.width = 2;
  message.point_step = 12;
  message.row_step = 24;
  message.is_dense = true;
  sensor_msgs::msg::PointField field;
  field.name = "x";
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.offset = 0;
  field.count = 1;
  message.fields = {field};
  auto cloud = pointcloud2::PointCloud2ReadHandle::from_message(
      message, std::move(*read));
  ASSERT_TRUE(cloud);
  EXPECT_EQ(cloud->header.frame_id, "lidar");
  EXPECT_EQ(cloud->fields.front().name, "x");
  cloud.reset();
  EXPECT_EQ(buffer_metadata::BufferRef::current_refcount(mapping), 0u);
  ::shm_unlink(name.c_str());
}

}  // namespace ros2_cuda_ipc_core
