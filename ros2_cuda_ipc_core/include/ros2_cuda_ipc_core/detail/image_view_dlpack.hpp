// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda.h>

#include <cstddef>

#include "ros2_cuda_ipc_core/image/image_view.hpp"

namespace ros2_cuda_ipc_core::image::detail {

/// Internal DLPack boundary for the typed ImageView adapter.
struct DLPackImageView {
  static bool bind(ImageView& view, CUstream consumer_stream);
  static void* device_ptr(const ImageView& view) noexcept;
  static std::size_t byte_size(const ImageView& view) noexcept;
  static int device_id(const ImageView& view) noexcept;
};

}  // namespace ros2_cuda_ipc_core::image::detail
