// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "julia_set/julia_publisher_helper.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_cuda_ipc_core/memory_backend_utils.hpp"
#include "ros2_cuda_ipc_core/nvtx_scoped_range.hpp"
#include "ros2_cuda_ipc_core/type_adapters.hpp"

namespace julia_set {

using ros2_cuda_ipc_core::NvtxScopedRange;
namespace {

ros2_cuda_ipc_core::DType parse_dtype(const std::string& name) {
  if (name == "u8") {
    return ros2_cuda_ipc_core::DType::U8;
  }
  if (name == "u16") {
    return ros2_cuda_ipc_core::DType::U16;
  }
  if (name == "f32") {
    return ros2_cuda_ipc_core::DType::F32;
  }
  if (name == "f64") {
    return ros2_cuda_ipc_core::DType::F64;
  }
  if (name == "s16") {
    return ros2_cuda_ipc_core::DType::S16;
  }
  if (name == "s32") {
    return ros2_cuda_ipc_core::DType::S32;
  }
  if (name == "u32") {
    return ros2_cuda_ipc_core::DType::U32;
  }
  throw std::runtime_error("Unsupported dtype: " + name);
}

}  // namespace

class JuliaSetPublisherNode : public rclcpp::Node {
 public:
  JuliaSetPublisherNode()
      : rclcpp::Node("julia_set_publisher",
                     rclcpp::NodeOptions().use_intra_process_comms(false)),
        publish_rate_hz_(declare_parameter<double>("publish_rate_hz", 30.0)),
        animate_(declare_parameter<bool>("animate", true)),
        animation_speed_(declare_parameter<double>("animation_speed", 1.0)),
        frame_id_(declare_parameter<std::string>("frame_id", "julia_frame")),
        topic_name_(
            declare_parameter<std::string>("topic_name", "julia_set/image")) {
    JuliaPublisherHelper::Config config;
    config.width = static_cast<uint32_t>(declare_parameter<int>("width", 1280));
    config.height =
        static_cast<uint32_t>(declare_parameter<int>("height", 720));
    config.channels =
        static_cast<uint32_t>(declare_parameter<int>("channels", 1));
    config.slot_count =
        static_cast<std::size_t>(declare_parameter<int>("slot_count", 4));
    config.shm_name =
        declare_parameter<std::string>("shm_name", "/ros2_cuda_ipc_julia");
    config.device_index = declare_parameter<int>("device_index", 0);
    const std::string dtype_str = declare_parameter<std::string>("dtype", "u8");
    config.dtype = parse_dtype(dtype_str);
    config.encoding = declare_parameter<std::string>("encoding", "mono8");
    if (config.channels != 1) {
      RCLCPP_WARN(
          get_logger(),
          "Only single-channel output is supported; overriding channels=%u",
          config.channels);
      config.channels = 1;
    }
    const int pending_ttl_ms = declare_parameter<int>(
        "pending_ttl_ms", static_cast<int>(config.pending_ttl.count()));
    config.pending_ttl = std::chrono::milliseconds{pending_ttl_ms};
    config.zoom = declare_parameter<double>("zoom", 1.5);
    config.offset_x =
        static_cast<float>(declare_parameter<double>("offset_x", 0.0));
    config.offset_y =
        static_cast<float>(declare_parameter<double>("offset_y", 0.0));
    config.constant_real =
        static_cast<float>(declare_parameter<double>("constant_real", -0.8));
    config.constant_imag =
        static_cast<float>(declare_parameter<double>("constant_imag", 0.156));
    config.max_iterations =
        static_cast<uint32_t>(declare_parameter<int>("max_iterations", 300));
    const std::string backend_param =
        declare_parameter<std::string>("memory_backend", "cuda_ipc");
    config.backend =
        ros2_cuda_ipc_core::parse_memory_backend(backend_param, get_logger());

    helper_ = std::make_unique<JuliaPublisherHelper>(config, get_logger());

    rclcpp::PublisherOptions options;
    options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    publisher_ = create_publisher<ros2_cuda_ipc_core::ImageView>(
        topic_name_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), options);

    if (publish_rate_hz_ <= 0.0) {
      publish_rate_hz_ = 30.0;
    }
    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&JuliaSetPublisherNode::on_timer, this));

    start_time_ = now();

    RCLCPP_INFO(get_logger(),
                "Publishing Julia set %ux%u dtype=%s encoding=%s slots=%zu",
                config.width, config.height, dtype_str.c_str(),
                config.encoding.empty() ? "<empty>" : config.encoding.c_str(),
                config.slot_count);
  }

 private:
  void on_timer() {
    NvtxScopedRange timer_range("JuliaSetPublisherNode::on_timer");
    const std::size_t subscribers = publisher_->get_subscription_count();
    float phase = 0.0f;
    if (animate_) {
      const auto elapsed = now() - start_time_;
      phase = static_cast<float>(elapsed.seconds() * animation_speed_);
    }

    auto view = helper_->produce(subscribers, phase);
    if (!view.has_value()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Failed to produce Julia set frame");
      return;
    }

    view->header.stamp = now();
    view->header.frame_id = frame_id_;
    publisher_->publish(*view);
  }

  rclcpp::Publisher<ros2_cuda_ipc_core::ImageView>::SharedPtr publisher_;
  std::unique_ptr<JuliaPublisherHelper> helper_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_{};
  double publish_rate_hz_ = 30.0;
  bool animate_ = true;
  double animation_speed_ = 1.0;
  std::string frame_id_;
  std::string topic_name_;
};

}  // namespace julia_set

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<julia_set::JuliaSetPublisherNode>();
    rclcpp::spin(node);
  } catch (const std::exception& ex) {
    RCLCPP_FATAL(rclcpp::get_logger("julia_set_publisher"), "Exception: %s",
                 ex.what());
  }
  rclcpp::shutdown();
  return 0;
}
