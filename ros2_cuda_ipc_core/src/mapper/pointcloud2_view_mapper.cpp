// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/mapper/pointcloud2_view_mapper.hpp"

namespace ros2_cuda_ipc_core::mapper {

namespace {

PointCloud2ViewMapper& default_pointcloud2_view_mapper() {
  static PointCloud2ViewMapper mapper;
  return mapper;
}

}  // namespace

PointCloud2ViewMapper::PointCloud2ViewMapper(BufferViewMapper buffer_mapper)
    : buffer_mapper_(std::move(buffer_mapper)) {}

view::PointCloud2View PointCloud2ViewMapper::map(
    const ros2_cuda_ipc_msgs::msg::GpuPointCloud2& msg) const {
  view::BufferView core = buffer_mapper_.map(msg.core);

  view::PointCloud2View mapped_view;
  mapped_view.header = msg.header;
  if (!core.valid()) {
    return mapped_view;
  }

  mapped_view.core = std::move(core);
  mapped_view.height = msg.height;
  mapped_view.width = msg.width;
  mapped_view.point_step = msg.point_step;
  mapped_view.row_step = msg.row_step;
  mapped_view.is_dense = msg.is_dense;
  mapped_view.fields.reserve(msg.fields.size());
  for (const auto& field : msg.fields) {
    view::PointCloud2View::Field converted;
    converted.name = field.name;
    converted.offset = field.offset;
    converted.datatype = field.datatype;
    converted.count = field.count;
    mapped_view.fields.emplace_back(std::move(converted));
  }

  return mapped_view;
}

view::PointCloud2View map_pointcloud2_view(
    const ros2_cuda_ipc_msgs::msg::GpuPointCloud2& msg) {
  return default_pointcloud2_view_mapper().map(msg);
}

}  // namespace ros2_cuda_ipc_core::mapper
