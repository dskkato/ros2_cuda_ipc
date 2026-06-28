#pragma once

#include "ros2_cuda_ipc_core/buffer_view_mapper.hpp"
#include "ros2_cuda_ipc_core/pointcloud2_view.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_point_cloud2.hpp"

namespace ros2_cuda_ipc_core {

class PointCloud2ViewMapper {
 public:
  explicit PointCloud2ViewMapper(
      BufferViewMapper buffer_mapper = BufferViewMapper{});

  PointCloud2View map(const ros2_cuda_ipc_msgs::msg::GpuPointCloud2& msg) const;

 private:
  BufferViewMapper buffer_mapper_;
};

PointCloud2View map_pointcloud2_view(
    const ros2_cuda_ipc_msgs::msg::GpuPointCloud2& msg);

void fill_gpu_pointcloud2_message(const PointCloud2View& view,
                                  ros2_cuda_ipc_msgs::msg::GpuPointCloud2& msg);

}  // namespace ros2_cuda_ipc_core
