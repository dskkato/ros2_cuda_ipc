// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include "ros2_cuda_ipc_core/image/image_view.hpp"
#include "ros2_cuda_ipc_core/subscriber/buffer_mapper.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_image.hpp"

namespace ros2_cuda_ipc_core::image {

class ImageViewMapper {
 public:
  explicit ImageViewMapper(
      subscriber::BufferMapper buffer_mapper = subscriber::BufferMapper{});

  ImageView map(const ros2_cuda_ipc_msgs::msg::GpuImage& msg,
                CUstream consumer_stream) const;
  ImageView map(const ros2_cuda_ipc_msgs::msg::GpuImage& msg) const;

  /// Build a DLPack typed adapter without binding it to a CUDA stream.
  ImageView map_for_dlpack(const ros2_cuda_ipc_msgs::msg::GpuImage& msg) const;

 private:
  subscriber::BufferMapper buffer_mapper_;
};

ImageView map_image_view(const ros2_cuda_ipc_msgs::msg::GpuImage& msg);
ImageView map_image_view(const ros2_cuda_ipc_msgs::msg::GpuImage& msg,
                         CUstream consumer_stream);

}  // namespace ros2_cuda_ipc_core::image
