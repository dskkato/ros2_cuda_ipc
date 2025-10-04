#pragma once

#include <cuda_runtime_api.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros2_cuda_ipc_core/image_view.hpp"
#include "ros2_cuda_ipc_core/nvtx_scoped_range.hpp"
#include "ros2_cuda_ipc_core/type_adapters.hpp"

namespace gpu_image_transport {

class GpuImageTransportNodeBase : public rclcpp::Node {
 public:
  explicit GpuImageTransportNodeBase(
      const std::string &node_name,
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~GpuImageTransportNodeBase() override;

 protected:
  virtual void publish_frame(const ros2_cuda_ipc_core::ImageView &view,
                             std::uint64_t available_bytes) = 0;

  const std::string &input_topic() const { return input_topic_; }
  const std::string &output_topic() const { return output_topic_; }
  uint8_t *host_buffer_data() const { return pinned_host_buffer_; }
  std::size_t host_buffer_capacity() const { return pinned_host_capacity_; }

  static std::string cuda_error_to_string(cudaError_t err);

 private:
  void on_image(const ros2_cuda_ipc_core::ImageView &view);
  cudaError_t ensure_pinned_capacity(std::size_t bytes);
  void release_pinned_host();

  rclcpp::Subscription<ros2_cuda_ipc_core::ImageView>::SharedPtr subscription_;
  cudaStream_t stream_ = nullptr;
  std::string input_topic_;
  std::string output_topic_;
  uint8_t *pinned_host_buffer_ = nullptr;
  std::size_t pinned_host_capacity_ = 0;
};

}  // namespace gpu_image_transport
