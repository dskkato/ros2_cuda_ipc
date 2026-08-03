// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_image/image_reader.hpp"

#include <utility>

namespace ros2_cuda_ipc_image {

std::optional<ImageReadHandle> ImageReader::read(
    const ros2_cuda_ipc_msgs::msg::GpuImage& message,
    CUstream consumer_stream) {
  auto mapped = mapper_.map(message.core, consumer_stream);
  if (!mapped) {
    return std::nullopt;
  }
  return ImageReadHandle::from_message(message, std::move(*mapped));
}

}  // namespace ros2_cuda_ipc_image
