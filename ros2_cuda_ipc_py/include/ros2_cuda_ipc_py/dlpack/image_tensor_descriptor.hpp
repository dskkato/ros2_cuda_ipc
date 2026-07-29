// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <dlpack/dlpack.h>

#include <array>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include "ros2_cuda_ipc_core/detail/image_view_dlpack.hpp"
#include "ros2_cuda_ipc_core/image/image_view.hpp"

namespace ros2_cuda_ipc_py::dlpack {

// A value object containing the DLPack representation projected from one
// mapped ImageView. It deliberately does not retain the source ImageView;
// ownership belongs to DlpackExportContext.
struct ImageTensorDescriptor {
  void* data = nullptr;
  int device_id = -1;
  uint64_t allocation_size = 0;
  uint64_t byte_offset = 0;
  int32_t rank = 0;
  std::array<int64_t, 3> shape{};
  std::array<int64_t, 3> element_strides{};
  DLDataType dl_dtype{};
};

inline DLDataType tensor_dl_dtype(ros2_cuda_ipc_core::image::DType dtype) {
  using ros2_cuda_ipc_core::image::DType;
  switch (dtype) {
    case DType::U8:
      return {kDLUInt, 8, 1};
    case DType::U16:
      return {kDLUInt, 16, 1};
    case DType::F16:
      return {kDLFloat, 16, 1};
    case DType::F32:
      return {kDLFloat, 32, 1};
    case DType::F64:
      return {kDLFloat, 64, 1};
    case DType::S16:
      return {kDLInt, 16, 1};
    case DType::S32:
      return {kDLInt, 32, 1};
    case DType::U32:
      return {kDLUInt, 32, 1};
  }
  throw std::invalid_argument("unsupported ros2_cuda_ipc image dtype");
}

inline ImageTensorDescriptor project_to_tensor(
    const ros2_cuda_ipc_core::image::ImageView& image) {
  if (!image.valid()) {
    throw std::invalid_argument("cannot export an invalid ImageView");
  }
  if (!image.sanity_check()) {
    throw std::invalid_argument(
        "ImageView shape/strides exceed the mapped allocation");
  }

  ImageTensorDescriptor result;
  result.data =
      ros2_cuda_ipc_core::image::detail::DLPackImageView::device_ptr(image);
  result.device_id =
      ros2_cuda_ipc_core::image::detail::DLPackImageView::device_id(image);
  result.allocation_size =
      ros2_cuda_ipc_core::image::detail::DLPackImageView::byte_size(image);
  result.rank = 3;
  result.dl_dtype = tensor_dl_dtype(image.dtype);

  const uint64_t element_size = image.elem_size_bytes();
  if (element_size == 0) {
    throw std::invalid_argument("ImageView dtype has zero-sized elements");
  }

  unsigned __int128 last_byte = 0;
  for (std::size_t index = 0; index < 3; ++index) {
    const uint64_t dimension = image.shape[index];
    const uint64_t byte_stride = image.strides[index];
    if (dimension == 0) {
      throw std::invalid_argument("ImageView dimensions must be positive");
    }
    if (byte_stride % element_size != 0) {
      throw std::invalid_argument(
          "ImageView byte strides must be divisible by the dtype size");
    }
    if (dimension > 1 && byte_stride < element_size) {
      throw std::invalid_argument(
          "ImageView strides overlap elements for a non-singleton dimension");
    }

    const uint64_t element_stride = byte_stride / element_size;
    if (element_stride >
        static_cast<uint64_t>(std::numeric_limits<int64_t>::max())) {
      throw std::invalid_argument(
          "ImageView element stride exceeds the DLPack int64 range");
    }
    result.shape[index] = static_cast<int64_t>(dimension);
    result.element_strides[index] = static_cast<int64_t>(element_stride);
    last_byte += static_cast<unsigned __int128>(dimension - 1) * byte_stride;
  }
  last_byte += element_size;

  if (last_byte > std::numeric_limits<uint64_t>::max() ||
      last_byte > result.allocation_size) {
    throw std::invalid_argument(
        "ImageView shape/strides exceed the mapped allocation");
  }

  const auto base = reinterpret_cast<uintptr_t>(result.data);
  if (base == 0 ||
      result.allocation_size > std::numeric_limits<uintptr_t>::max() - base ||
      result.byte_offset > std::numeric_limits<uintptr_t>::max() - base ||
      last_byte >
          std::numeric_limits<uintptr_t>::max() - base - result.byte_offset) {
    throw std::invalid_argument("ImageView pointer arithmetic overflows");
  }

  return result;
}

}  // namespace ros2_cuda_ipc_py::dlpack
