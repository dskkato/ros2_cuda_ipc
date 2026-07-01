// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include "multi_process_image_fanout/image_publisher_helper.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_cuda_ipc_core/cuda/nvtx_scoped_range.hpp"
#include "ros2_cuda_ipc_core/memory_backend_utils.hpp"
#include "ros2_cuda_ipc_core/type_adapters.hpp"

namespace multi_process_image_fanout {

using ros2_cuda_ipc_core::cuda::NvtxScopedRange;

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
    ImagePublisherHelper::Config config;
    config.width = static_cast<uint32_t>(declare_parameter<int>("width", 1920));
    config.height =
        static_cast<uint32_t>(declare_parameter<int>("height", 1080));
    config.slot_count =
        static_cast<std::size_t>(declare_parameter<int>("slot_count", 4));
    config.pending_ttl = std::chrono::milliseconds(
        declare_parameter<int>("pending_ttl_ms", 300));
    config.shm_name =
        declare_parameter<std::string>("shm_name", "/ros2_cuda_ipc_fanout");
    config.device_index = declare_parameter<int>("device_index", 0);
    config.backend = ros2_cuda_ipc_core::parse_memory_backend(
        declare_parameter<std::string>("memory_backend", "cuda_ipc"),
        get_logger());

    helper_ = std::make_unique<ImagePublisherHelper>(config, get_logger());

    rclcpp::PublisherOptions options;
    options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    publisher_ = create_publisher<ros2_cuda_ipc_core::view::ImageView>(
        topic_name_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), options);

    if (publish_rate_hz_ <= 0.0) {
      publish_rate_hz_ = 30.0;
    }

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        [this]() { on_timer(); });

    RCLCPP_INFO(
        get_logger(), "Publishing fanout GPU image %ux%u on %s with %zu slots",
        config.width, config.height, topic_name_.c_str(), config.slot_count);
  }

 private:
  void on_timer() {
    NvtxScopedRange timer_range("GpuImagePublisherNode::on_timer");

    const std::size_t subscribers = publisher_->get_subscription_count();
    auto view = helper_->produce(subscribers, frame_index_);
    if (!view.has_value()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Failed to produce GPU image frame");
      return;
    }

    view->header.stamp = now();
    view->header.frame_id = frame_id_;
    publisher_->publish(*view);
    ++frame_index_;
  }

  rclcpp::Publisher<ros2_cuda_ipc_core::view::ImageView>::SharedPtr publisher_;
  std::unique_ptr<ImagePublisherHelper> helper_;
  rclcpp::TimerBase::SharedPtr timer_;
  double publish_rate_hz_ = 30.0;
  uint64_t frame_index_ = 0;
  std::string frame_id_;
  std::string topic_name_;
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
