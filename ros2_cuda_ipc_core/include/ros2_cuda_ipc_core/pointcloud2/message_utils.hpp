// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <utility>

#include "ros2_cuda_ipc_core/pointcloud2/pointcloud2_view.hpp"
#include "ros2_cuda_ipc_core/transport/message_utils.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace ros2_cuda_ipc_core::pointcloud2 {

// Build the point-cloud-specific part of a wire message from publisher
// metadata. Buffer ownership and transport handles remain in BufferDescriptor.
inline void fill_gpu_pointcloud2_message(
    const transport::BufferDescriptor& descriptor,
    const PointCloud2View& metadata,
    ros2_cuda_ipc_msgs::msg::GpuPointCloud2& message) {
  transport::fill_buffer_core_message(descriptor, message.core);
  message.header = metadata.header;
  message.height = metadata.height;
  message.width = metadata.width;
  message.point_step = metadata.point_step;
  message.row_step = metadata.row_step;
  message.is_dense = metadata.is_dense;
  message.fields.clear();
  message.fields.reserve(metadata.fields.size());
  for (const auto& source : metadata.fields) {
    sensor_msgs::msg::PointField field;
    field.name = source.name;
    field.offset = source.offset;
    field.datatype = source.datatype;
    field.count = source.count;
    message.fields.emplace_back(std::move(field));
  }
}

}  // namespace ros2_cuda_ipc_core::pointcloud2
