// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include "ros2_cuda_ipc_core/transport/buffer_descriptor.hpp"

namespace ros2_cuda_ipc_core::transport {

// Copies transport metadata into the ROS wire message. Publisher-local CUDA
// pointers and process-local buffer references are intentionally not part of
// BufferCore.
void fill_buffer_core_message(const BlockDescriptor& descriptor,
                              ros2_cuda_ipc_msgs::msg::BufferCore& message);

}  // namespace ros2_cuda_ipc_core::transport
