#pragma once

#include <rclcpp/type_adapter.hpp>
#include <type_traits>

#include "ros2_cuda_ipc_core/buffer_view_mapper.hpp"
#include "ros2_cuda_ipc_core/image_view_mapper.hpp"
#include "ros2_cuda_ipc_core/pointcloud2_view_mapper.hpp"
#include "ros2_cuda_ipc_msgs/msg/buffer_core.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_image.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_point_cloud2.hpp"

namespace rclcpp {

template <>
struct TypeAdapter<ros2_cuda_ipc_core::BufferView,
                   ros2_cuda_ipc_msgs::msg::BufferCore> {
  using is_specialized = std::true_type;
  using custom_type = ros2_cuda_ipc_core::BufferView;
  using ros_message_type = ros2_cuda_ipc_msgs::msg::BufferCore;

  static void convert_to_custom(const ros_message_type& msg,
                                custom_type& view) {
    view = ros2_cuda_ipc_core::map_buffer_view(msg);
  }

  static void convert_to_ros_message(const custom_type& view,
                                     ros_message_type& msg) {
    ros2_cuda_ipc_core::fill_buffer_core_message(view, msg);
  }
};

template <>
struct TypeAdapter<ros2_cuda_ipc_core::ImageView,
                   ros2_cuda_ipc_msgs::msg::GpuImage> {
  using is_specialized = std::true_type;
  using custom_type = ros2_cuda_ipc_core::ImageView;
  using ros_message_type = ros2_cuda_ipc_msgs::msg::GpuImage;

  static void convert_to_custom(const ros_message_type& msg,
                                custom_type& view) {
    view = ros2_cuda_ipc_core::map_image_view(msg);
  }

  static void convert_to_ros_message(const custom_type& view,
                                     ros_message_type& msg) {
    ros2_cuda_ipc_core::fill_gpu_image_message(view, msg);
  }
};

template <>
struct TypeAdapter<ros2_cuda_ipc_core::PointCloud2View,
                   ros2_cuda_ipc_msgs::msg::GpuPointCloud2> {
  using is_specialized = std::true_type;
  using custom_type = ros2_cuda_ipc_core::PointCloud2View;
  using ros_message_type = ros2_cuda_ipc_msgs::msg::GpuPointCloud2;

  static void convert_to_custom(const ros_message_type& msg,
                                custom_type& view) {
    view = ros2_cuda_ipc_core::map_pointcloud2_view(msg);
  }

  static void convert_to_ros_message(const custom_type& view,
                                     ros_message_type& msg) {
    ros2_cuda_ipc_core::fill_gpu_pointcloud2_message(view, msg);
  }
};

}  // namespace rclcpp

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
    ros2_cuda_ipc_core::BufferView, ros2_cuda_ipc_msgs::msg::BufferCore);
RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(ros2_cuda_ipc_core::ImageView,
                                             ros2_cuda_ipc_msgs::msg::GpuImage);
RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
    ros2_cuda_ipc_core::PointCloud2View,
    ros2_cuda_ipc_msgs::msg::GpuPointCloud2);
