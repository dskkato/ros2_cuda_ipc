// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda.h>

#include <optional>

#include "ros2_cuda_ipc_core/subscriber/buffer_mapper.hpp"
#include "ros2_cuda_ipc_image/image_read_handle.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_image.hpp"

namespace ros2_cuda_ipc_image {

/// Maps and validates one GpuImage message for a consumer stream.
///
/// The mapper is kept across calls so its metadata cache can be reused. The
/// returned ImageReadHandle owns the mapped read and its stream-bound
/// lifetime.
class ImageReader {
 public:
  ImageReader() = default;
  ~ImageReader() = default;

  ImageReader(const ImageReader&) = delete;
  ImageReader& operator=(const ImageReader&) = delete;
  ImageReader(ImageReader&&) noexcept = default;
  ImageReader& operator=(ImageReader&&) noexcept = default;

  std::optional<ImageReadHandle> read(
      const ros2_cuda_ipc_msgs::msg::GpuImage& message,
      CUstream consumer_stream);

 private:
  ros2_cuda_ipc_core::subscriber::BufferMapper mapper_;
};

}  // namespace ros2_cuda_ipc_image
