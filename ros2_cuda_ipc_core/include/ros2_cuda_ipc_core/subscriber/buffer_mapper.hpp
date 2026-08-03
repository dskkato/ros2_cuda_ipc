// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda.h>

#include <memory>
#include <optional>

#include "ros2_cuda_ipc_core/detail/mapped_publication.hpp"
#include "ros2_cuda_ipc_core/subscriber/read_handle.hpp"
#include "ros2_cuda_ipc_msgs/msg/buffer_core.hpp"

namespace ros2_cuda_ipc_core::subscriber {

class BufferMapper;
namespace detail {
std::unique_ptr<MappedPublication> acquire_publication(
    const BufferMapper& mapper, const ros2_cuda_ipc_msgs::msg::BufferCore& msg);
}  // namespace detail

/// Maps a BufferCore descriptor into a stream-bound ReadHandle.
///
/// An empty optional is the complete public failure result.  Detailed
/// buffer_ref, import, backend, and CUDA diagnostics are written to the
/// internal log.
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
  friend std::unique_ptr<detail::MappedPublication> detail::acquire_publication(
      const BufferMapper&, const ros2_cuda_ipc_msgs::msg::BufferCore&);

  std::unique_ptr<detail::MappedPublication> acquire_publication_impl(
      const ros2_cuda_ipc_msgs::msg::BufferCore& msg) const;

  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ros2_cuda_ipc_core::subscriber
