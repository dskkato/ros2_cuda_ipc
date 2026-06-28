// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/image_view.hpp"

namespace ros2_cuda_ipc_core {

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

  const uint64_t last_row = (rows() - 1) * strideH();
  const uint64_t last_col = (cols() - 1) * strideW();
  const uint64_t last_chan = (channels() - 1) * strideC();
  const uint64_t needed = last_row + last_col + last_chan + elem_size_bytes();
  return core.byte_size >= needed;
}

}  // namespace ros2_cuda_ipc_core
