// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/image/image_view.hpp"

#include <limits>

namespace ros2_cuda_ipc_core::image {

uint32_t ImageView::elem_size_bytes() const noexcept {
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
  return 1;
}

bool ImageView::sanity_check() const noexcept {
  if (!valid() || channels() == 0) {
    return false;
  }

  using WideUnsigned = unsigned __int128;
  const WideUnsigned needed =
      (static_cast<WideUnsigned>(rows() - 1) * strideH()) +
      (static_cast<WideUnsigned>(cols() - 1) * strideW()) +
      (static_cast<WideUnsigned>(channels() - 1) * strideC()) +
      elem_size_bytes();
  return needed <= std::numeric_limits<uint64_t>::max() &&
         needed <= core.byte_size;
}

}  // namespace ros2_cuda_ipc_core::image
