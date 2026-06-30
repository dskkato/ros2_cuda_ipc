// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include "rclcpp/logging.hpp"
#include "ros2_cuda_ipc_core/view/buffer_view.hpp"
#include "ros2_cuda_ipc_msgs/msg/buffer_core.hpp"

namespace ros2_cuda_ipc_core::mapper {

struct BufferViewMapperOptions {
  rclcpp::Logger logger =
      rclcpp::get_logger("ros2_cuda_ipc_core.BufferViewMapper");
};

class BufferViewMapper {
 public:
  explicit BufferViewMapper(BufferViewMapperOptions options = {});

  view::BufferView map(const ros2_cuda_ipc_msgs::msg::BufferCore& msg) const;

 private:
  BufferViewMapperOptions options_;
};

view::BufferView map_buffer_view(
    const ros2_cuda_ipc_msgs::msg::BufferCore& msg);

void fill_buffer_core_message(const view::BufferView& view,
                              ros2_cuda_ipc_msgs::msg::BufferCore& msg);

}  // namespace ros2_cuda_ipc_core::mapper
