// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include "ros2_cuda_ipc_core/pointcloud2/pointcloud2_view.hpp"
#include "ros2_cuda_ipc_core/subscriber/buffer_mapper.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_point_cloud2.hpp"

namespace ros2_cuda_ipc_core::pointcloud2 {

class PointCloud2ViewMapper {
 public:
  explicit PointCloud2ViewMapper(
      subscriber::BufferMapper buffer_mapper = subscriber::BufferMapper{});

  PointCloud2View map(const ros2_cuda_ipc_msgs::msg::GpuPointCloud2& msg,
                      CUstream consumer_stream) const;
  PointCloud2View map(const ros2_cuda_ipc_msgs::msg::GpuPointCloud2& msg) const;

 private:
  subscriber::BufferMapper buffer_mapper_;
};

PointCloud2View map_pointcloud2_view(
    const ros2_cuda_ipc_msgs::msg::GpuPointCloud2& msg);

}  // namespace ros2_cuda_ipc_core::pointcloud2
