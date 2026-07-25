// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda.h>
#include <dlpack/dlpack.h>

#include <array>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

#include "ros2_cuda_ipc_core/image/image_view.hpp"

namespace ros2_cuda_ipc_py {

// Private, framework-independent metadata for one mapped ImageView.  The
// owner is a retained native view, not a Python object, and therefore keeps
// the imported resource and slot lease alive for every adapter using this
// representation.
struct TensorMetadata {
  void* data = nullptr;
  int device_id = -1;
  uint64_t allocation_size = 0;
  uint64_t byte_offset = 0;
  int32_t rank = 0;
  std::array<int64_t, 3> shape{};
  std::array<uint64_t, 3> byte_strides{};
  std::array<int64_t, 3> element_strides{};
  DLDataType dl_dtype{};
  std::string dtype_name;
  ros2_cuda_ipc_core::image::ImageView owner;
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

inline const char* tensor_dtype_name(ros2_cuda_ipc_core::image::DType dtype) {
  using ros2_cuda_ipc_core::image::DType;
  switch (dtype) {
    case DType::U8:
      return "uint8";
    case DType::U16:
      return "uint16";
    case DType::F16:
      return "float16";
    case DType::F32:
      return "float32";
    case DType::F64:
      return "float64";
    case DType::S16:
      return "int16";
    case DType::S32:
      return "int32";
    case DType::U32:
      return "uint32";
  }
  throw std::invalid_argument("unsupported ros2_cuda_ipc image dtype");
}

inline TensorMetadata make_tensor_metadata(
    const ros2_cuda_ipc_core::image::ImageView& view) {
  if (!view.valid()) {
    throw std::invalid_argument("cannot export an invalid ImageView");
  }
  if (!view.sanity_check()) {
    throw std::invalid_argument(
        "ImageView shape/strides exceed the mapped allocation");
  }

  TensorMetadata result;
  result.data = view.core.device_ptr();
  result.device_id = view.core.device_id;
  result.allocation_size = view.core.byte_size;
  result.rank = 3;
  result.dl_dtype = tensor_dl_dtype(view.dtype);
  result.dtype_name = tensor_dtype_name(view.dtype);

  const uint64_t element_size = view.elem_size_bytes();
  if (element_size == 0) {
    throw std::invalid_argument("ImageView dtype has zero-sized elements");
  }

  unsigned __int128 last_byte = 0;
  for (std::size_t index = 0; index < 3; ++index) {
    const uint64_t dimension = view.shape[index];
    const uint64_t byte_stride = view.strides[index];
    if (dimension == 0) {
      throw std::invalid_argument(
          "ImageView dimensions must be non-negative and non-zero");
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
    result.byte_strides[index] = byte_stride;
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

  const int imported_device = view.core.imported_device_id();
  if (imported_device >= 0 && imported_device != result.device_id) {
    throw std::invalid_argument(
        "mapped image device does not match the imported CUDA allocation");
  }

  result.owner = view;
  return result;
}

}  // namespace ros2_cuda_ipc_py
