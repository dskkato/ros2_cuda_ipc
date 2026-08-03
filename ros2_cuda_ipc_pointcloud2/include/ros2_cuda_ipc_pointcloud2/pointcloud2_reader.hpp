// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda.h>

#include <optional>

#include "ros2_cuda_ipc_core/subscriber/buffer_mapper.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_point_cloud2.hpp"
#include "ros2_cuda_ipc_pointcloud2/pointcloud2_read_handle.hpp"

namespace ros2_cuda_ipc_pointcloud2 {

/// Maps and validates one GpuPointCloud2 message for a consumer stream.
///
/// The mapper is kept across calls so its metadata cache can be reused. The
/// returned PointCloud2ReadHandle owns the mapped read and its stream-bound
/// lifetime.
class PointCloud2Reader {
 public:
  PointCloud2Reader() = default;
  ~PointCloud2Reader() = default;

  PointCloud2Reader(const PointCloud2Reader&) = delete;
  PointCloud2Reader& operator=(const PointCloud2Reader&) = delete;
  PointCloud2Reader(PointCloud2Reader&&) noexcept = default;
  PointCloud2Reader& operator=(PointCloud2Reader&&) noexcept = default;

  std::optional<PointCloud2ReadHandle> read(
      const ros2_cuda_ipc_msgs::msg::GpuPointCloud2& message,
      CUstream consumer_stream);

 private:
  ros2_cuda_ipc_core::subscriber::BufferMapper mapper_;
};

}  // namespace ros2_cuda_ipc_pointcloud2
