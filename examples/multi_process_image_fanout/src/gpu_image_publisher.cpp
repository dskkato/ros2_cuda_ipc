// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <cuda_runtime_api.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "multi_process_image_fanout/cuda_checks.hpp"
#include "multi_process_image_fanout/kernels.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_cuda_ipc_core/backend/memory_backend_utils.hpp"
#include "ros2_cuda_ipc_core/detail/nvtx_scoped_range.hpp"
#include "ros2_cuda_ipc_core/image/image_view.hpp"
#include "ros2_cuda_ipc_core/publisher/gpu_buffer_manager.hpp"
#include "ros2_cuda_ipc_core/transport/message_utils.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_image.hpp"

namespace multi_process_image_fanout {

using ros2_cuda_ipc_core::detail::NvtxScopedRange;

namespace {
constexpr uint64_t kBytesPerPixel = 4;
}  // namespace

class GpuImagePublisherNode : public rclcpp::Node {
 public:
  GpuImagePublisherNode()
      : rclcpp::Node("gpu_image_publisher",
                     rclcpp::NodeOptions().use_intra_process_comms(false)),
        publish_rate_hz_(declare_parameter<double>("publish_rate_hz", 30.0)),
        frame_id_(
            declare_parameter<std::string>("frame_id", "fanout_camera_frame")),
        topic_name_(
            declare_parameter<std::string>("topic_name", "/fanout/image_gpu")) {
    const int width = declare_parameter<int>("width", 1920);
    const int height = declare_parameter<int>("height", 1080);
    const int slot_count_parameter = declare_parameter<int>("slot_count", 4);
    const auto pending_ttl = std::chrono::milliseconds(
        declare_parameter<int>("pending_ttl_ms", 300));
    const auto shm_name_prefix = declare_parameter<std::string>(
        "shm_name_prefix", "/ros2_cuda_ipc_fanout");
    const int device_index = declare_parameter<int>("device_index", 0);
    const auto backend = ros2_cuda_ipc_core::backend::parse_memory_backend(
        declare_parameter<std::string>("memory_backend", "cuda_ipc"),
        get_logger());
    encoding_ = declare_parameter<std::string>("encoding", kDefaultEncoding);

    if (width <= 0 || height <= 0) {
      throw std::runtime_error("width and height must be greater than zero");
    }
    if (slot_count_parameter <= 0) {
      throw std::runtime_error("slot_count must be greater than zero");
    }
    width_ = static_cast<uint32_t>(width);
    height_ = static_cast<uint32_t>(height);
    const auto slot_count = static_cast<std::size_t>(slot_count_parameter);

    throw_on_cuda_error(cudaSetDevice(device_index), "cudaSetDevice");
    const uint64_t frame_size_bytes =
        static_cast<uint64_t>(width_) * height_ * kBytesPerPixel;
    manager_ =
        std::make_unique<ros2_cuda_ipc_core::publisher::GpuBufferManager>(
            ros2_cuda_ipc_core::publisher::GpuBufferManager::Config{
                shm_name_prefix, slot_count, frame_size_bytes, device_index,
                pending_ttl, backend},
            get_logger().get_child("GpuBufferManager"));
    if (!manager_->initialise()) {
      throw std::runtime_error("Failed to initialise GPU buffer manager");
    }
    throw_on_cuda_error(
        cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking),
        "cudaStreamCreateWithFlags");

    try {
      rclcpp::PublisherOptions options;
      options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
      publisher_ = create_publisher<ros2_cuda_ipc_msgs::msg::GpuImage>(
          topic_name_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), options);

      if (publish_rate_hz_ <= 0.0) {
        publish_rate_hz_ = 30.0;
      }

      const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
      timer_ = create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(period),
          [this]() { on_timer(); });
    } catch (...) {
      cudaStreamDestroy(stream_);
      stream_ = nullptr;
      manager_.reset();
      throw;
    }

    RCLCPP_INFO(get_logger(),
                "Publishing fanout GPU image %ux%u on %s with %zu slots",
                width_, height_, topic_name_.c_str(), slot_count);
  }

  ~GpuImagePublisherNode() override {
    if (stream_ != nullptr) {
      const cudaError_t error = cudaStreamDestroy(stream_);
      if (error != cudaSuccess) {
        RCLCPP_ERROR(
            get_logger(), "cudaStreamDestroy failed: %s",
            ros2_cuda_ipc_core::detail::cuda_error_to_string(error).c_str());
      }
      stream_ = nullptr;
    }
    manager_.reset();
  }

 private:
  void on_timer() {
    NvtxScopedRange timer_range("GpuImagePublisherNode::on_timer");

    const std::size_t subscribers = publisher_->get_subscription_count();
    std::optional<ros2_cuda_ipc_core::publisher::PublishSlot> slot;
    {
      NvtxScopedRange acquire_range("GpuImagePublisherNode::acquire_slot");
      slot = manager_->acquire_for_publish(static_cast<uint32_t>(subscribers));
    }
    if (!slot) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "No available GPU slots (all leases in use)");
      return;
    }

    cudaError_t error = cudaSuccess;
    {
      NvtxScopedRange kernel_range(
          "GpuImagePublisherNode::generate_rgba_pattern_kernel");
      error = launch_generate_rgba_pattern_kernel(
          static_cast<uint8_t*>(slot->device_ptr()), static_cast<int>(width_),
          static_cast<int>(height_), width_ * kBytesPerPixel, frame_index_,
          stream_);
    }
    if (!log_cuda_error(get_logger(), "launch_generate_rgba_pattern_kernel",
                        error)) {
      return;
    }

    {
      NvtxScopedRange event_range("GpuImagePublisherNode::record_ready");
      auto result = slot->record_ready(stream_);
      if (!result) {
        RCLCPP_WARN(get_logger(), "record_ready failed: %s",
                    result.error().to_string().c_str());
        return;
      }
    }

    const auto descriptor = slot->descriptor();
    if (!descriptor) {
      RCLCPP_ERROR(get_logger(), "Failed to create GPU buffer descriptor");
      return;
    }

    ros2_cuda_ipc_msgs::msg::GpuImage message;
    ros2_cuda_ipc_core::transport::fill_buffer_core_message(*descriptor,
                                                            message.core);
    message.dtype = static_cast<uint8_t>(ros2_cuda_ipc_core::image::DType::U8);
    message.shape = {height_, width_, kDefaultChannels};
    message.strides = {width_ * kBytesPerPixel, kDefaultChannels, 1};
    message.encoding = encoding_;
    message.header.stamp = now();
    message.header.frame_id = frame_id_;

    publisher_->publish(message);
    slot->commit_publish();
    ++frame_index_;
  }

  rclcpp::Publisher<ros2_cuda_ipc_msgs::msg::GpuImage>::SharedPtr publisher_;
  std::unique_ptr<ros2_cuda_ipc_core::publisher::GpuBufferManager> manager_;
  rclcpp::TimerBase::SharedPtr timer_;
  cudaStream_t stream_ = nullptr;
  double publish_rate_hz_ = 30.0;
  uint64_t frame_index_ = 0;
  uint32_t width_ = 0;
  uint32_t height_ = 0;
  std::string frame_id_;
  std::string topic_name_;
  std::string encoding_;
};

}  // namespace multi_process_image_fanout

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node =
        std::make_shared<multi_process_image_fanout::GpuImagePublisherNode>();
    rclcpp::spin(node);
  } catch (const std::exception& ex) {
    RCLCPP_FATAL(rclcpp::get_logger("gpu_image_publisher"), "Exception: %s",
                 ex.what());
  }
  rclcpp::shutdown();
  return 0;
}
