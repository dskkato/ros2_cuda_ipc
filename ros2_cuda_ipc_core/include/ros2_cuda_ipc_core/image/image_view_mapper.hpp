// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include "ros2_cuda_ipc_core/image/image_view.hpp"
#include "ros2_cuda_ipc_core/subscriber/buffer_view_mapper.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_image.hpp"

namespace ros2_cuda_ipc_core::image {

class ImageViewMapper {
 public:
  explicit ImageViewMapper(subscriber::BufferViewMapper buffer_mapper =
                               subscriber::BufferViewMapper{});

  ImageView map(const ros2_cuda_ipc_msgs::msg::GpuImage& msg) const;

 private:
  subscriber::BufferViewMapper buffer_mapper_;
};

ImageView map_image_view(const ros2_cuda_ipc_msgs::msg::GpuImage& msg);

}  // namespace ros2_cuda_ipc_core::image
