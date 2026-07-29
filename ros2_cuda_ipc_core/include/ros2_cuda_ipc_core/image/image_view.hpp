// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda.h>

#include <array>
#include <cstdint>
#include <memory>
#include <string>

#include "ros2_cuda_ipc_core/subscriber/read_handle.hpp"
#include "std_msgs/msg/header.hpp"

namespace ros2_cuda_ipc_core::subscriber::detail {
class MappedPublication;
}

namespace ros2_cuda_ipc_core::image {

namespace detail {
struct DLPackImageView;
}

enum class DType : uint8_t {
  U8 = 0,
  U16 = 1,
  F16 = 2,
  F32 = 3,
  F64 = 4,
  S16 = 5,
  S32 = 6,
  U32 = 7,
};

struct ImageView {
  std_msgs::msg::Header header{};
  subscriber::ReadHandle core;
  std::array<uint32_t, 3> shape{0, 0, 0};
  std::array<uint64_t, 3> strides{0, 0, 0};
  DType dtype = DType::U8;
  std::string encoding;

  ImageView() noexcept;
  ~ImageView() noexcept;
  ImageView(const ImageView&) = delete;
  ImageView& operator=(const ImageView&) = delete;
  ImageView(ImageView&&) noexcept;
  ImageView& operator=(ImageView&&) noexcept;

  uint32_t rows() const noexcept { return shape[0]; }
  uint32_t cols() const noexcept { return shape[1]; }
  uint32_t channels() const noexcept { return shape[2]; }
  uint64_t strideH() const noexcept { return strides[0]; }
  uint64_t strideW() const noexcept { return strides[1]; }
  uint64_t strideC() const noexcept { return strides[2]; }

  bool valid() const noexcept;

  uint32_t elem_size_bytes() const noexcept;

  struct DeviceView {
    uint8_t* data;
    int height;
    int width;
    int channels;
    uint64_t strideH;
    uint64_t strideW;
    uint64_t strideC;
    uint8_t dtype;
  };

  DeviceView as_device_view() const noexcept {
    return DeviceView{core.data<uint8_t>(),
                      static_cast<int>(rows()),
                      static_cast<int>(cols()),
                      static_cast<int>(channels()),
                      strideH(),
                      strideW(),
                      strideC(),
                      static_cast<uint8_t>(dtype)};
  }

  bool sanity_check() const noexcept;

 private:
  std::unique_ptr<subscriber::detail::MappedPublication> publication_;

  friend class ImageViewMapper;
  friend struct detail::DLPackImageView;
};

}  // namespace ros2_cuda_ipc_core::image
