// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda.h>

#include <memory>
#include <optional>

#include "ros2_cuda_ipc_core/detail/mapped_publication.hpp"
#include "ros2_cuda_ipc_core/subscriber/read_handle.hpp"
#include "ros2_cuda_ipc_msgs/msg/buffer_core.hpp"

namespace ros2_cuda_ipc_core::image {
class ImageViewMapper;
}

namespace ros2_cuda_ipc_core::subscriber {

/// Maps a BufferCore descriptor into a stream-bound ReadHandle.
///
/// An empty optional is the complete public failure result.  Detailed lease,
/// import, backend, and CUDA diagnostics are written to the internal log.
class BufferMapper {
 public:
  BufferMapper();
  ~BufferMapper();

  BufferMapper(BufferMapper&&) noexcept;
  BufferMapper& operator=(BufferMapper&&) noexcept;
  BufferMapper(const BufferMapper&) = delete;
  BufferMapper& operator=(const BufferMapper&) = delete;

  std::optional<ReadHandle> map(const ros2_cuda_ipc_msgs::msg::BufferCore& msg,
                                CUstream consumer_stream) const;

 private:
  std::unique_ptr<detail::MappedPublication> map_publication(
      const ros2_cuda_ipc_msgs::msg::BufferCore& msg) const;

  class Impl;
  std::unique_ptr<Impl> impl_;

  friend class ros2_cuda_ipc_core::image::ImageViewMapper;
};

}  // namespace ros2_cuda_ipc_core::subscriber
