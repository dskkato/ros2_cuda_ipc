// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cstddef>

namespace ros2_cuda_ipc_core::image {

struct ImageReadHandle;

namespace detail {

/// Internal DLPack boundary for the typed ImageReadHandle adapter.
struct DLPackImageReadHandle {
  static void* device_ptr(const ImageReadHandle& image) noexcept;
  static std::size_t byte_size(const ImageReadHandle& image) noexcept;
  static int device_id(const ImageReadHandle& image) noexcept;
};

}  // namespace detail

}  // namespace ros2_cuda_ipc_core::image
