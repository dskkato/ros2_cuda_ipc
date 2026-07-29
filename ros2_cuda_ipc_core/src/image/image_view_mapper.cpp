// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/image/image_view_mapper.hpp"

#include <utility>

#include "ros2_cuda_ipc_core/detail/mapped_publication.hpp"

namespace ros2_cuda_ipc_core::image {

namespace {

ImageViewMapper& default_image_view_mapper() {
  static ImageViewMapper mapper;
  return mapper;
}

}  // namespace

ImageViewMapper::ImageViewMapper(subscriber::BufferMapper buffer_mapper)
    : buffer_mapper_(std::move(buffer_mapper)) {}

ImageView ImageViewMapper::map(const ros2_cuda_ipc_msgs::msg::GpuImage& msg,
                               CUstream consumer_stream) const {
  auto core = buffer_mapper_.map(msg.core, consumer_stream);
  if (!core) {
    return ImageView{};
  }

  ImageView mapped_view;
  mapped_view.core = std::move(*core);
  mapped_view.dtype = static_cast<DType>(msg.dtype);
  mapped_view.shape = msg.shape;
  mapped_view.strides = msg.strides;
  mapped_view.encoding = msg.encoding;
  mapped_view.header = msg.header;
  return mapped_view;
}

ImageView ImageViewMapper::map(
    const ros2_cuda_ipc_msgs::msg::GpuImage& msg) const {
  return map_for_dlpack(msg);
}

ImageView ImageViewMapper::map_for_dlpack(
    const ros2_cuda_ipc_msgs::msg::GpuImage& msg) const {
  auto publication = buffer_mapper_.map_publication(msg.core);
  if (!publication) {
    return ImageView{};
  }

  ImageView mapped_view;
  mapped_view.publication_ = std::move(publication);
  mapped_view.dtype = static_cast<DType>(msg.dtype);
  mapped_view.shape = msg.shape;
  mapped_view.strides = msg.strides;
  mapped_view.encoding = msg.encoding;
  mapped_view.header = msg.header;
  return mapped_view;
}

ImageView map_image_view(const ros2_cuda_ipc_msgs::msg::GpuImage& msg) {
  return default_image_view_mapper().map(msg, CU_STREAM_LEGACY);
}

ImageView map_image_view(const ros2_cuda_ipc_msgs::msg::GpuImage& msg,
                         CUstream consumer_stream) {
  return default_image_view_mapper().map(msg, consumer_stream);
}

}  // namespace ros2_cuda_ipc_core::image
