// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/image/image_view_mapper.hpp"

#include <utility>

namespace ros2_cuda_ipc_core::image {

namespace {

ImageViewMapper& default_image_view_mapper() {
  static ImageViewMapper mapper;
  return mapper;
}

}  // namespace

ImageViewMapper::ImageViewMapper(subscriber::BufferViewMapper buffer_mapper)
    : buffer_mapper_(std::move(buffer_mapper)) {}

ImageView ImageViewMapper::map(
    const ros2_cuda_ipc_msgs::msg::GpuImage& msg) const {
  subscriber::BufferView core = buffer_mapper_.map(msg.core);
  if (!core.valid()) {
    return ImageView{};
  }

  ImageView mapped_view;
  mapped_view.core = std::move(core);
  mapped_view.dtype = static_cast<DType>(msg.dtype);
  mapped_view.shape = msg.shape;
  mapped_view.strides = msg.strides;
  mapped_view.encoding = msg.encoding;
  mapped_view.header = msg.header;
  return mapped_view;
}

ImageView map_image_view(const ros2_cuda_ipc_msgs::msg::GpuImage& msg) {
  return default_image_view_mapper().map(msg);
}

}  // namespace ros2_cuda_ipc_core::image
