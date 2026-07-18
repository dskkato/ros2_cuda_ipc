// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include "ros2_cuda_ipc_core/image/image_view.hpp"
#include "ros2_cuda_ipc_core/transport/message_utils.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_image.hpp"

namespace ros2_cuda_ipc_core::image {

// Build the image-specific part of a wire message from publisher metadata.
// Buffer ownership and transport handles remain in BufferDescriptor.
inline void fill_gpu_image_message(
    const transport::BufferDescriptor& descriptor, const ImageView& metadata,
    ros2_cuda_ipc_msgs::msg::GpuImage& message) {
  transport::fill_buffer_core_message(descriptor, message.core);
  message.header = metadata.header;
  message.dtype = static_cast<uint8_t>(metadata.dtype);
  message.shape = metadata.shape;
  message.strides = metadata.strides;
  message.encoding = metadata.encoding;
}

}  // namespace ros2_cuda_ipc_core::image
