#include "ros2_cuda_ipc_core/pointcloud2_view_mapper.hpp"

#include "sensor_msgs/msg/point_field.hpp"

namespace ros2_cuda_ipc_core {

namespace {

PointCloud2ViewMapper& default_pointcloud2_view_mapper() {
  static PointCloud2ViewMapper mapper;
  return mapper;
}

}  // namespace

PointCloud2ViewMapper::PointCloud2ViewMapper(BufferViewMapper buffer_mapper)
    : buffer_mapper_(std::move(buffer_mapper)) {}

PointCloud2View PointCloud2ViewMapper::map(
    const ros2_cuda_ipc_msgs::msg::GpuPointCloud2& msg) const {
  BufferView core = buffer_mapper_.map(msg.core);

  PointCloud2View view;
  view.header = msg.header;
  if (!core.valid()) {
    return view;
  }

  view.core = std::move(core);
  view.height = msg.height;
  view.width = msg.width;
  view.point_step = msg.point_step;
  view.row_step = msg.row_step;
  view.is_dense = msg.is_dense;
  view.fields.reserve(msg.fields.size());
  for (const auto& field : msg.fields) {
    PointCloud2View::Field converted;
    converted.name = field.name;
    converted.offset = field.offset;
    converted.datatype = field.datatype;
    converted.count = field.count;
    view.fields.emplace_back(std::move(converted));
  }

  return view;
}

PointCloud2View map_pointcloud2_view(
    const ros2_cuda_ipc_msgs::msg::GpuPointCloud2& msg) {
  return default_pointcloud2_view_mapper().map(msg);
}

void fill_gpu_pointcloud2_message(
    const PointCloud2View& view, ros2_cuda_ipc_msgs::msg::GpuPointCloud2& msg) {
  msg.header = view.header;
  msg.height = view.height;
  msg.width = view.width;
  msg.point_step = view.point_step;
  msg.row_step = view.row_step;
  msg.is_dense = view.is_dense;
  msg.fields.clear();
  msg.fields.reserve(view.fields.size());
  for (const auto& field : view.fields) {
    sensor_msgs::msg::PointField converted;
    converted.name = field.name;
    converted.offset = field.offset;
    converted.datatype = field.datatype;
    converted.count = field.count;
    msg.fields.emplace_back(std::move(converted));
  }
  fill_buffer_core_message(view.core, msg.core);
}

}  // namespace ros2_cuda_ipc_core
