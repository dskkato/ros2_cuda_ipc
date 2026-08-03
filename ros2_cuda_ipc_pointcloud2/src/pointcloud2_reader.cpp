// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_pointcloud2/pointcloud2_reader.hpp"

#include <utility>

namespace ros2_cuda_ipc_pointcloud2 {

std::optional<PointCloud2ReadHandle> PointCloud2Reader::read(
    const ros2_cuda_ipc_msgs::msg::GpuPointCloud2& message,
    CUstream consumer_stream) {
  auto mapped = mapper_.map(message.core, consumer_stream);
  if (!mapped) {
    return std::nullopt;
  }
  return PointCloud2ReadHandle::from_message(message, std::move(*mapped));
}

}  // namespace ros2_cuda_ipc_pointcloud2
