#pragma once

#include "ros2_cuda_ipc_core/detail/backend_importer.hpp"

namespace ros2_cuda_ipc_core::detail {

class VmmFdImporter final : public BackendImporter {
 public:
  std::optional<ImportedBuffer> import(
      const ros2_cuda_ipc_msgs::msg::BufferCore& msg,
      const cudaIpcEventHandle_t& event_handle,
      const rclcpp::Logger& logger) const override;
};

}  // namespace ros2_cuda_ipc_core::detail
