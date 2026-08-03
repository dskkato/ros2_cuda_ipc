// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/detail/image_read_handle_dlpack.hpp"

#include "ros2_cuda_ipc_core/image/image_read_handle.hpp"

namespace ros2_cuda_ipc_core::image::detail {

void* DLPackImageReadHandle::device_ptr(const ImageReadHandle& image) noexcept {
  return image.read.device_ptr();
}

std::size_t DLPackImageReadHandle::byte_size(
    const ImageReadHandle& image) noexcept {
  return image.read.byte_size();
}

int DLPackImageReadHandle::device_id(const ImageReadHandle& image) noexcept {
  return image.read.device_id();
}

}  // namespace ros2_cuda_ipc_core::image::detail
