// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include "ros2_cuda_ipc_core/transport/buffer_descriptor.hpp"
#include "ros2_cuda_ipc_core/transport/message_utils.hpp"
#include "ros2_cuda_ipc_image/image_read_handle.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_image.hpp"

namespace ros2_cuda_ipc_image {

// Build the image-specific part of a wire message from publisher metadata.
// Buffer ownership and transport handles remain in BufferDescriptor.
inline void fill_gpu_image_message(
    const ros2_cuda_ipc_core::transport::BufferDescriptor& descriptor,
    const ImageReadHandle& metadata,
    ros2_cuda_ipc_msgs::msg::GpuImage& message) {
  ros2_cuda_ipc_core::transport::fill_buffer_core_message(descriptor,
                                                          message.core);
  message.header = metadata.header;
  message.dtype = static_cast<uint8_t>(metadata.dtype);
  message.shape = metadata.shape;
  message.strides = metadata.strides;
  message.encoding = metadata.encoding;
}

}  // namespace ros2_cuda_ipc_image
