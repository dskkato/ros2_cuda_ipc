// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/pointcloud2/pointcloud2_read_handle.hpp"

#include <limits>
#include <utility>

#include "sensor_msgs/msg/point_field.hpp"

namespace ros2_cuda_ipc_core::pointcloud2 {

std::optional<PointCloud2ReadHandle> PointCloud2ReadHandle::from_message(
    const ros2_cuda_ipc_msgs::msg::GpuPointCloud2& message,
    subscriber::ReadHandle read) {
  PointCloud2ReadHandle result;
  result.header = message.header;
  result.read = std::move(read);
  result.height = message.height;
  result.width = message.width;
  result.point_step = message.point_step;
  result.row_step = message.row_step;
  result.is_dense = message.is_dense;
  result.fields.reserve(message.fields.size());
  for (const auto& field : message.fields) {
    result.fields.push_back(
        Field{field.name, field.offset, field.datatype, field.count});
  }

  if (!result.valid() || result.height == 0 ||
      result.row_step <
          static_cast<uint64_t>(result.point_step) * result.width) {
    return std::nullopt;
  }
  using WideUnsigned = unsigned __int128;
  const WideUnsigned needed =
      static_cast<WideUnsigned>(result.row_step) * (result.height - 1) +
      static_cast<WideUnsigned>(result.point_step) * result.width;
  if (needed > std::numeric_limits<uint64_t>::max() ||
      needed > result.read.byte_size()) {
    return std::nullopt;
  }
  for (const auto& field : result.fields) {
    uint32_t element_size = 0;
    switch (field.datatype) {
      case sensor_msgs::msg::PointField::INT8:
      case sensor_msgs::msg::PointField::UINT8:
        element_size = 1;
        break;
      case sensor_msgs::msg::PointField::INT16:
      case sensor_msgs::msg::PointField::UINT16:
        element_size = 2;
        break;
      case sensor_msgs::msg::PointField::INT32:
      case sensor_msgs::msg::PointField::UINT32:
      case sensor_msgs::msg::PointField::FLOAT32:
        element_size = 4;
        break;
      case sensor_msgs::msg::PointField::FLOAT64:
        element_size = 8;
        break;
      default:
        return std::nullopt;
    }
    const uint64_t field_bytes =
        static_cast<uint64_t>(element_size) * field.count;
    if (field.count == 0 || field.offset >= result.point_step ||
        field_bytes > result.point_step - field.offset) {
      return std::nullopt;
    }
  }
  return std::optional<PointCloud2ReadHandle>(std::move(result));
}

}  // namespace ros2_cuda_ipc_core::pointcloud2
