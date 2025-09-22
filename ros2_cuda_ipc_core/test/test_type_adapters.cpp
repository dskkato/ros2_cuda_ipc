#include <cuda_runtime_api.h>
#include <gtest/gtest.h>
#include <sys/mman.h>
#include <unistd.h>

#include <atomic>
#include <cstring>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>

#include "ros2_cuda_ipc_core/buffer_view.hpp"
#include "ros2_cuda_ipc_core/lease_handle.hpp"
#include "ros2_cuda_ipc_core/type_adapters.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace {

std::string make_unique_shm_name(const std::string &prefix) {
  static std::atomic<int> counter{0};
  std::ostringstream oss;
  oss << "/" << prefix << "_" << ::getpid() << "_" << counter.fetch_add(1);
  return oss.str();
}

class TypeAdapterTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      int argc = 0;
      char **argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override {
    int device_count = 0;
    auto err = cudaGetDeviceCount(&device_count);
    if (err != cudaSuccess || device_count == 0) {
      GTEST_SKIP() << "CUDA device not available for tests";
    }

    err = cudaSetDevice(0);
    if (err != cudaSuccess) {
      GTEST_SKIP() << "Failed to select CUDA device 0";
    }
  }
};

}  // namespace

TEST_F(TypeAdapterTest, ConvertToCustomSuccess) {
  struct CudaAllocation {
    void *ptr = nullptr;
    cudaEvent_t event = nullptr;

    ~CudaAllocation() {
      if (event) {
        cudaEventDestroy(event);
      }
      if (ptr) {
        cudaFree(ptr);
      }
    }

    void release() {
      if (event) {
        EXPECT_EQ(cudaSuccess, cudaEventDestroy(event));
        event = nullptr;
      }
      if (ptr) {
        EXPECT_EQ(cudaSuccess, cudaFree(ptr));
        ptr = nullptr;
      }
    }
  } allocation;

  const std::string shm_name = make_unique_shm_name("adapter");
  ASSERT_TRUE(ros2_cuda_ipc_core::LeaseHandle::init(shm_name, 1));
  auto gen = ros2_cuda_ipc_core::LeaseHandle::bump_generation(shm_name, 0);
  ASSERT_TRUE(gen.has_value());

  constexpr size_t kBufferSize = 1024;
  ASSERT_EQ(cudaSuccess, cudaMalloc(&allocation.ptr, kBufferSize));
  ASSERT_EQ(cudaSuccess, cudaEventCreateWithFlags(
                             &allocation.event,
                             cudaEventDisableTiming | cudaEventInterprocess));

  cudaIpcMemHandle_t mem_handle{};
  ASSERT_EQ(cudaSuccess, cudaIpcGetMemHandle(&mem_handle, allocation.ptr));
  cudaIpcEventHandle_t event_handle{};
  ASSERT_EQ(cudaSuccess,
            cudaIpcGetEventHandle(&event_handle, allocation.event));

  ros2_cuda_ipc_msgs::msg::BufferCore msg;
  msg.shm_name = shm_name;
  msg.slot_id = 0;
  msg.device_id = 1;
  msg.generation = gen.value();
  msg.byte_size = kBufferSize;
  std::memcpy(msg.mem_handle.data(), &mem_handle, sizeof(mem_handle));
  std::memcpy(msg.event_handle.data(), &event_handle, sizeof(event_handle));

  ros2_cuda_ipc_core::BufferView view;
  rclcpp::TypeAdapter<
      ros2_cuda_ipc_core::BufferView,
      ros2_cuda_ipc_msgs::msg::BufferCore>::convert_to_custom(msg, view);

  ASSERT_TRUE(view.valid());
  EXPECT_EQ(view.byte_size, 1024u);
  EXPECT_EQ(view.device_id, 1);
  EXPECT_EQ(view.slot_id, 0u);
  EXPECT_EQ(view.generation, gen.value());
  EXPECT_TRUE(view.lease);

  auto refcnt = ros2_cuda_ipc_core::LeaseHandle::current_refcount(shm_name, 0);
  ASSERT_TRUE(refcnt.has_value());
  EXPECT_EQ(refcnt.value(), 1u);

  view.reset();
  allocation.release();

  refcnt = ros2_cuda_ipc_core::LeaseHandle::current_refcount(shm_name, 0);
  ASSERT_TRUE(refcnt.has_value());
  EXPECT_EQ(refcnt.value(), 0u);

  ::shm_unlink(shm_name.c_str());
}

TEST_F(TypeAdapterTest, ConvertToCustomLeaseFailureReturnsInvalid) {
  const std::string shm_name = make_unique_shm_name("adapter_fail");
  ASSERT_TRUE(ros2_cuda_ipc_core::LeaseHandle::init(shm_name, 1));

  ros2_cuda_ipc_msgs::msg::BufferCore msg;
  msg.shm_name = shm_name;
  msg.slot_id = 0;
  msg.device_id = 1;
  msg.generation = 42;  // wrong generation
  msg.byte_size = 256;

  ros2_cuda_ipc_core::BufferView view;
  rclcpp::TypeAdapter<
      ros2_cuda_ipc_core::BufferView,
      ros2_cuda_ipc_msgs::msg::BufferCore>::convert_to_custom(msg, view);
  EXPECT_FALSE(view.valid());

  ::shm_unlink(shm_name.c_str());
}

TEST_F(TypeAdapterTest, ConvertToRosMessageCopiesMetadata) {
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
  view.set_ipc_handles(mem_handle, event_handle);

  ros2_cuda_ipc_msgs::msg::BufferCore msg;
  rclcpp::TypeAdapter<
      ros2_cuda_ipc_core::BufferView,
      ros2_cuda_ipc_msgs::msg::BufferCore>::convert_to_ros_message(view, msg);

  EXPECT_EQ(msg.device_id, 2u);
  EXPECT_EQ(msg.byte_size, 64u);
  EXPECT_EQ(msg.slot_id, 5u);
  EXPECT_EQ(msg.generation, 9u);
  EXPECT_EQ(msg.shm_name, "/buf_meta");
  EXPECT_EQ(msg.mem_handle[0], 0x12);
  EXPECT_EQ(msg.event_handle[0], 0x34);
}

TEST_F(TypeAdapterTest, ImageViewRoundTripTransfersHeaderAndLayout) {
  const std::string shm_name = make_unique_shm_name("adapter_image");
  ASSERT_TRUE(ros2_cuda_ipc_core::LeaseHandle::init(shm_name, 1));
  auto gen = ros2_cuda_ipc_core::LeaseHandle::bump_generation(shm_name, 0);
  ASSERT_TRUE(gen.has_value());

  ros2_cuda_ipc_msgs::msg::GpuImage msg;
  msg.header.frame_id = "camera_frame";
  msg.dtype = static_cast<uint8_t>(ros2_cuda_ipc_core::DType::U8);
  msg.shape = {4, 5, 3};
  msg.strides = {15, 3, 1};
  msg.encoding = "rgb8";
  msg.core.shm_name = shm_name;
  msg.core.slot_id = 0;
  msg.core.device_id = 0;
  msg.core.generation = gen.value();
  msg.core.byte_size = 60;
  msg.core.mem_handle.fill(0);
  msg.core.event_handle.fill(0);

  ros2_cuda_ipc_core::ImageView view;
  rclcpp::TypeAdapter<
      ros2_cuda_ipc_core::ImageView,
      ros2_cuda_ipc_msgs::msg::GpuImage>::convert_to_custom(msg, view);
  EXPECT_TRUE(view.core.valid());
  EXPECT_EQ(view.header.frame_id, "camera_frame");
  EXPECT_EQ(view.shape[0], 4u);
  view.core.reset();

  ::shm_unlink(shm_name.c_str());

  ros2_cuda_ipc_core::ImageView emit;
  emit.dtype = ros2_cuda_ipc_core::DType::U8;
  emit.shape = {4, 5, 3};
  emit.strides = {15, 3, 1};
  emit.encoding = "rgb8";
  emit.header.frame_id = "frame";
  emit.core.device_id = 1;
  emit.core.byte_size = 60;
  emit.core.slot_id = 3;
  emit.core.generation = 7;
  emit.core.shm_name = "/demo";
  cudaIpcMemHandle_t dummy_mem{};
  cudaIpcEventHandle_t dummy_evt{};
  emit.core.set_ipc_handles(dummy_mem, dummy_evt);

  ros2_cuda_ipc_msgs::msg::GpuImage out_msg;
  rclcpp::TypeAdapter<
      ros2_cuda_ipc_core::ImageView,
      ros2_cuda_ipc_msgs::msg::GpuImage>::convert_to_ros_message(emit, out_msg);
  EXPECT_EQ(out_msg.header.frame_id, "frame");
  EXPECT_EQ(out_msg.encoding, "rgb8");
  EXPECT_EQ(out_msg.shape[1], 5u);
  EXPECT_EQ(out_msg.core.slot_id, 3u);
  EXPECT_EQ(out_msg.core.shm_name, "/demo");
}

TEST_F(TypeAdapterTest, PointCloudViewRoundTripTransfersLayout) {
  const std::string shm_name = make_unique_shm_name("adapter_pc");
  ASSERT_TRUE(ros2_cuda_ipc_core::LeaseHandle::init(shm_name, 1));
  auto gen = ros2_cuda_ipc_core::LeaseHandle::bump_generation(shm_name, 0);
  ASSERT_TRUE(gen.has_value());

  ros2_cuda_ipc_msgs::msg::GpuPointCloud2 msg;
  msg.header.frame_id = "frame_pc";
  msg.height = 1;
  msg.width = 2;
  msg.point_step = 12;
  msg.row_step = 24;
  msg.is_dense = true;
  msg.core.shm_name = shm_name;
  msg.core.slot_id = 0;
  msg.core.device_id = 0;
  msg.core.generation = gen.value();
  msg.core.byte_size = 24;
  msg.core.mem_handle.fill(0);
  msg.core.event_handle.fill(0);
  sensor_msgs::msg::PointField field_x;
  field_x.name = "x";
  field_x.offset = 0;
  field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field_x.count = 1;
  sensor_msgs::msg::PointField field_y = field_x;
  field_y.name = "y";
  field_y.offset = 4;
  sensor_msgs::msg::PointField field_z = field_x;
  field_z.name = "z";
  field_z.offset = 8;
  msg.fields = {field_x, field_y, field_z};

  ros2_cuda_ipc_core::PointCloud2View view;
  rclcpp::TypeAdapter<
      ros2_cuda_ipc_core::PointCloud2View,
      ros2_cuda_ipc_msgs::msg::GpuPointCloud2>::convert_to_custom(msg, view);
  EXPECT_TRUE(view.core.valid());
  EXPECT_EQ(view.header.frame_id, "frame_pc");
  EXPECT_EQ(view.width, 2u);
  EXPECT_EQ(view.fields.size(), 3u);
  view.core.reset();

  ::shm_unlink(shm_name.c_str());

  ros2_cuda_ipc_core::PointCloud2View emit;
  emit.header.frame_id = "frame";
  emit.core.device_id = 1;
  emit.core.byte_size = 120;
  emit.core.slot_id = 5;
  emit.core.generation = 9;
  emit.core.shm_name = "/pc_demo";
  cudaIpcMemHandle_t dummy_mem{};
  cudaIpcEventHandle_t dummy_evt{};
  emit.core.set_ipc_handles(dummy_mem, dummy_evt);
  emit.height = 1;
  emit.width = 10;
  emit.point_step = 12;
  emit.row_step = 120;
  emit.is_dense = true;
  emit.fields = {{"x", 0u, sensor_msgs::msg::PointField::FLOAT32, 1u},
                 {"y", 4u, sensor_msgs::msg::PointField::FLOAT32, 1u},
                 {"z", 8u, sensor_msgs::msg::PointField::FLOAT32, 1u}};

  ros2_cuda_ipc_msgs::msg::GpuPointCloud2 out_msg;
  rclcpp::TypeAdapter<
      ros2_cuda_ipc_core::PointCloud2View,
      ros2_cuda_ipc_msgs::msg::GpuPointCloud2>::convert_to_ros_message(emit,
                                                                       out_msg);
  EXPECT_EQ(out_msg.header.frame_id, "frame");
  EXPECT_EQ(out_msg.width, 10u);
  EXPECT_EQ(out_msg.fields.size(), 3u);
  EXPECT_EQ(out_msg.core.slot_id, 5u);
  EXPECT_EQ(out_msg.core.shm_name, "/pc_demo");
}
