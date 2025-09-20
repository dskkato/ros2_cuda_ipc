#pragma once

#include <algorithm>
#include <cstring>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/type_adapter.hpp>

#include "ros2_cuda_ipc_core/buffer_view.hpp"
#include "ros2_cuda_ipc_core/image_view.hpp"
#include "ros2_cuda_ipc_core/lease_handle.hpp"
#include "ros2_cuda_ipc_core/pointcloud2_view.hpp"
#include "ros2_cuda_ipc_msgs/msg/buffer_core.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_image.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/header.hpp"

namespace ros2_cuda_ipc_core {

inline cudaIpcMemHandle_t to_cuda_mem_handle(
    const ros2_cuda_ipc_msgs::msg::BufferCore &msg) {
  cudaIpcMemHandle_t handle{};
  std::memcpy(&handle, msg.mem_handle.data(), sizeof(handle));
  return handle;
}

inline cudaIpcEventHandle_t to_cuda_event_handle(
    const ros2_cuda_ipc_msgs::msg::BufferCore &msg) {
  cudaIpcEventHandle_t handle{};
  std::memcpy(&handle, msg.event_handle.data(), sizeof(handle));
  return handle;
}

inline void fill_buffer_core_message(const BufferView &view,
                                     ros2_cuda_ipc_msgs::msg::BufferCore &msg) {
  msg.shm_name = view.shm_name;
  msg.device_id = static_cast<uint32_t>(view.device_id);
  msg.slot_id = view.slot_id;
  msg.generation = view.generation;
  msg.byte_size = view.byte_size;
  if (view.handles_ready()) {
    std::memcpy(msg.mem_handle.data(), &view.mem_handle(),
                sizeof(cudaIpcMemHandle_t));
    std::memcpy(msg.event_handle.data(), &view.event_handle(),
                sizeof(cudaIpcEventHandle_t));
  } else {
    std::memset(msg.mem_handle.data(), 0, msg.mem_handle.size());
    std::memset(msg.event_handle.data(), 0, msg.event_handle.size());
  }
}

inline BufferView make_invalid_buffer_view() {
  BufferView view;
  view.reset();
  return view;
}

}  // namespace ros2_cuda_ipc_core

namespace rclcpp {

template <>
struct TypeAdapter<ros2_cuda_ipc_core::BufferView,
                   ros2_cuda_ipc_msgs::msg::BufferCore> {
  using is_specialized = std::true_type;
  using custom_type = ros2_cuda_ipc_core::BufferView;
  using ros_message_type = ros2_cuda_ipc_msgs::msg::BufferCore;

  static void convert_to_custom(const ros_message_type &msg,
                                custom_type &view) {
    view.reset();

    auto lease = ros2_cuda_ipc_core::LeaseHandle::acquire(
        msg.shm_name, msg.slot_id, msg.generation);
    if (!lease.valid()) {
      RCLCPP_WARN(rclcpp::get_logger("ros2_cuda_ipc_core.BufferView"),
                  "Failed to acquire lease shm=%s slot=%u gen=%u",
                  msg.shm_name.c_str(), msg.slot_id, msg.generation);
      view = ros2_cuda_ipc_core::BufferView{};
      return;
    }

    auto lease_ptr =
        std::make_shared<ros2_cuda_ipc_core::LeaseHandle>(std::move(lease));

    cudaIpcMemHandle_t mem_handle = ros2_cuda_ipc_core::to_cuda_mem_handle(msg);
    void *dev_ptr = nullptr;
    auto err = ros2_cuda_ipc_core::CudaSupport::open_ipc_memory(
        &dev_ptr, mem_handle, cudaIpcMemLazyEnablePeerAccess);
    if (err != cudaSuccess) {
      RCLCPP_WARN(rclcpp::get_logger("ros2_cuda_ipc_core.BufferView"),
                  "cudaIpcOpenMemHandle failed: %s",
                  ros2_cuda_ipc_core::CudaSupport::error_string(err));
      view = ros2_cuda_ipc_core::BufferView{};
      return;
    }

    cudaEvent_t evt{};
    cudaIpcEventHandle_t event_handle =
        ros2_cuda_ipc_core::to_cuda_event_handle(msg);
    err = ros2_cuda_ipc_core::CudaSupport::open_ipc_event(&evt, event_handle);
    if (err != cudaSuccess) {
      ros2_cuda_ipc_core::CudaSupport::close_ipc_memory(dev_ptr);
      RCLCPP_WARN(rclcpp::get_logger("ros2_cuda_ipc_core.BufferView"),
                  "cudaIpcOpenEventHandle failed: %s",
                  ros2_cuda_ipc_core::CudaSupport::error_string(err));
      view = ros2_cuda_ipc_core::BufferView{};
      return;
    }

    custom_type opened;
    opened.dev_ptr = dev_ptr;
    opened.ready_evt = evt;
    opened.device_id = static_cast<int>(msg.device_id);
    opened.byte_size = msg.byte_size;
    opened.slot_id = msg.slot_id;
    opened.generation = msg.generation;
    opened.shm_name = msg.shm_name;
    opened.lease = std::move(lease_ptr);
    opened.set_ipc_handles(mem_handle, event_handle);
    opened.mark_opened_via_ipc(true, true);

    view = std::move(opened);
  }

  static void convert_to_ros_message(const custom_type &view,
                                     ros_message_type &msg) {
    fill_buffer_core_message(view, msg);
  }
};

template <>
struct TypeAdapter<ros2_cuda_ipc_core::ImageView,
                   ros2_cuda_ipc_msgs::msg::GpuImage> {
  using is_specialized = std::true_type;
  using custom_type = ros2_cuda_ipc_core::ImageView;
  using ros_message_type = ros2_cuda_ipc_msgs::msg::GpuImage;

  static void convert_to_custom(const ros_message_type &msg,
                                custom_type &view) {
    ros2_cuda_ipc_core::BufferView core;
    TypeAdapter<
        ros2_cuda_ipc_core::BufferView,
        ros2_cuda_ipc_msgs::msg::BufferCore>::convert_to_custom(msg.core, core);
    if (!core.valid()) {
      view = ros2_cuda_ipc_core::ImageView();
      return;
    }

    custom_type converted;
    converted.core = std::move(core);
    converted.dtype = static_cast<ros2_cuda_ipc_core::DType>(msg.dtype);
    std::copy(msg.shape.begin(), msg.shape.end(), converted.shape.begin());
    std::copy(msg.strides.begin(), msg.strides.end(),
              converted.strides.begin());
    converted.encoding = msg.encoding;
    converted.header = msg.header;

    view = std::move(converted);
  }

  static void convert_to_ros_message(const custom_type &view,
                                     ros_message_type &msg) {
    msg.dtype = static_cast<uint8_t>(view.dtype);
    for (size_t i = 0; i < view.shape.size(); ++i) {
      msg.shape[i] = view.shape[i];
      msg.strides[i] = view.strides[i];
    }
    msg.encoding = view.encoding;
    msg.header = view.header;
    TypeAdapter<
        ros2_cuda_ipc_core::BufferView,
        ros2_cuda_ipc_msgs::msg::BufferCore>::convert_to_ros_message(view.core,
                                                                     msg.core);
  }
};

template <>
struct TypeAdapter<ros2_cuda_ipc_core::PointCloud2View,
                   ros2_cuda_ipc_msgs::msg::GpuPointCloud2> {
  using is_specialized = std::true_type;
  using custom_type = ros2_cuda_ipc_core::PointCloud2View;
  using ros_message_type = ros2_cuda_ipc_msgs::msg::GpuPointCloud2;

  static void convert_to_custom(const ros_message_type &msg,
                                custom_type &view) {
    ros2_cuda_ipc_core::BufferView core;
    TypeAdapter<
        ros2_cuda_ipc_core::BufferView,
        ros2_cuda_ipc_msgs::msg::BufferCore>::convert_to_custom(msg.core, core);
    if (!core.valid()) {
      view = ros2_cuda_ipc_core::PointCloud2View{};
      return;
    }

    custom_type converted;
    converted.core = std::move(core);
    converted.height = msg.height;
    converted.width = msg.width;
    converted.point_step = msg.point_step;
    converted.row_step = msg.row_step;
    converted.is_dense = msg.is_dense;
    converted.fields.reserve(msg.fields.size());
    for (const auto &field : msg.fields) {
      custom_type::Field f;
      f.name = field.name;
      f.offset = field.offset;
      f.datatype = field.datatype;
      f.count = field.count;
      converted.fields.emplace_back(std::move(f));
    }

    view = std::move(converted);
  }

  static void convert_to_ros_message(const custom_type &view,
                                     ros_message_type &msg) {
    msg.height = view.height;
    msg.width = view.width;
    msg.point_step = view.point_step;
    msg.row_step = view.row_step;
    msg.is_dense = view.is_dense;
    msg.fields.clear();
    msg.fields.reserve(view.fields.size());
    for (const auto &field : view.fields) {
      sensor_msgs::msg::PointField f;
      f.name = field.name;
      f.offset = field.offset;
      f.datatype = field.datatype;
      f.count = field.count;
      msg.fields.emplace_back(std::move(f));
    }

    TypeAdapter<
        ros2_cuda_ipc_core::BufferView,
        ros2_cuda_ipc_msgs::msg::BufferCore>::convert_to_ros_message(view.core,
                                                                     msg.core);
  }
};

}  // namespace rclcpp
