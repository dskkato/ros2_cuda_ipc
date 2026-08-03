// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/image/image_read_handle.hpp"

#include <limits>
#include <utility>

namespace ros2_cuda_ipc_core::image {

std::optional<ImageReadHandle> ImageReadHandle::from_message(
    const ros2_cuda_ipc_msgs::msg::GpuImage& message,
    subscriber::ReadHandle read) {
  ImageReadHandle result;
  result.header = message.header;
  result.read = std::move(read);
  result.dtype = static_cast<DType>(message.dtype);
  result.shape = message.shape;
  result.strides = message.strides;
  result.encoding = message.encoding;
  if (!result.sanity_check()) {
    return std::nullopt;
  }
  return std::optional<ImageReadHandle>(std::move(result));
}

bool ImageReadHandle::valid() const noexcept {
  return read.valid() && rows() > 0 && cols() > 0 && channels() > 0;
}

uint32_t ImageReadHandle::elem_size_bytes() const noexcept {
  switch (dtype) {
    case DType::U8:
      return 1;
    case DType::U16:
    case DType::F16:
    case DType::S16:
      return 2;
    case DType::F32:
    case DType::S32:
    case DType::U32:
      return 4;
    case DType::F64:
      return 8;
  }
  return 0;
}

bool ImageReadHandle::sanity_check() const noexcept {
  if (!valid() || elem_size_bytes() == 0) {
    return false;
  }

  using WideUnsigned = unsigned __int128;
  const WideUnsigned needed =
      (static_cast<WideUnsigned>(rows() - 1) * strideH()) +
      (static_cast<WideUnsigned>(cols() - 1) * strideW()) +
      (static_cast<WideUnsigned>(channels() - 1) * strideC()) +
      elem_size_bytes();
  return needed <= std::numeric_limits<uint64_t>::max() &&
         needed <= read.byte_size();
}

}  // namespace ros2_cuda_ipc_core::image
