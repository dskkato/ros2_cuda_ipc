// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "ros2_cuda_ipc_core/subscriber/read_handle.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"

namespace ros2_cuda_ipc_core::pointcloud2 {

struct PointCloud2ReadHandle {
  struct Field {
    std::string name;
    uint32_t offset = 0;
    uint8_t datatype = 0;
    uint32_t count = 0;
  };

  struct DeviceField {
    uint32_t offset;
    uint8_t datatype;
    uint32_t count;
  };

  struct DeviceView {
    uint8_t* data;
    int width;
    int height;
    size_t point_step;
    size_t row_step;
    bool is_dense;
    const DeviceField* fields;
    int num_fields;
  };

  std_msgs::msg::Header header{};
  subscriber::ReadHandle read;
  uint32_t height = 1;
  uint32_t width = 0;
  uint32_t point_step = 0;
  uint32_t row_step = 0;
  bool is_dense = true;
  std::vector<Field> fields;

  static std::optional<PointCloud2ReadHandle> from_message(
      const ros2_cuda_ipc_msgs::msg::GpuPointCloud2& message,
      subscriber::ReadHandle read);

  size_t num_points() const noexcept {
    return static_cast<size_t>(width) * height;
  }
  bool valid() const noexcept { return read.valid() && point_step > 0; }

  DeviceView as_device_view(const DeviceField* device_fields,
                            int n) const noexcept {
    return DeviceView{read.data<uint8_t>(),
                      static_cast<int>(width),
                      static_cast<int>(height),
                      point_step,
                      row_step,
                      is_dense,
                      device_fields,
                      n};
  }
};

}  // namespace ros2_cuda_ipc_core::pointcloud2
