#pragma once

#include "rclcpp/logging.hpp"
#include "ros2_cuda_ipc_core/buffer_view.hpp"
#include "ros2_cuda_ipc_msgs/msg/buffer_core.hpp"

namespace ros2_cuda_ipc_core::mapper {

struct BufferViewMapperOptions {
  rclcpp::Logger logger =
      rclcpp::get_logger("ros2_cuda_ipc_core.BufferViewMapper");
};

class BufferViewMapper {
 public:
  explicit BufferViewMapper(BufferViewMapperOptions options = {});

  BufferView map(const ros2_cuda_ipc_msgs::msg::BufferCore& msg) const;

 private:
  BufferViewMapperOptions options_;
};

BufferView map_buffer_view(const ros2_cuda_ipc_msgs::msg::BufferCore& msg);

void fill_buffer_core_message(const BufferView& view,
                              ros2_cuda_ipc_msgs::msg::BufferCore& msg);

}  // namespace ros2_cuda_ipc_core::mapper
