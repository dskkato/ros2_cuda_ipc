// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <cuda_runtime_api.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "multi_process_image_fanout/cuda_checks.hpp"
#include "multi_process_image_fanout/status_format.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_cuda_ipc_core/cuda/cuda_util.hpp"
#include "ros2_cuda_ipc_core/cuda/nvtx_scoped_range.hpp"
#include "ros2_cuda_ipc_core/type_adapters.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace multi_process_image_fanout {

using ros2_cuda_ipc_core::cuda::NvtxScopedRange;

namespace {

constexpr uint32_t kChannels = 4;
constexpr uint64_t kBytesPerPixel = 4;

}  // namespace

class PreviewNode : public rclcpp::Node {
 public:
  PreviewNode()
      : rclcpp::Node("preview_node",
                     rclcpp::NodeOptions().use_intra_process_comms(false)),
        copy_every_n_(static_cast<std::size_t>(
            declare_parameter<int>("copy_every_n", 1))),
        log_every_n_(static_cast<std::size_t>(
            declare_parameter<int>("log_every_n", 30))),
        input_topic_name_(declare_parameter<std::string>("input_topic_name",
                                                         "/fanout/image_gpu")),
        output_topic_name_(declare_parameter<std::string>(
            "output_topic_name", "/fanout/preview/image")) {
    if (copy_every_n_ == 0) {
      copy_every_n_ = 1;
    }
    if (log_every_n_ == 0) {
      log_every_n_ = 1;
    }

    rclcpp::PublisherOptions pub_options;
    pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    publisher_ = create_publisher<sensor_msgs::msg::Image>(
        output_topic_name_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        pub_options);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    subscription_ = create_subscription<ros2_cuda_ipc_core::view::ImageView>(
        input_topic_name_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        [this](const ros2_cuda_ipc_core::view::ImageView& view) {
          on_image(view);
        },
        sub_options);

    RCLCPP_INFO(get_logger(), "Preview node subscribed to %s and publishes %s",
                input_topic_name_.c_str(), output_topic_name_.c_str());
  }

  ~PreviewNode() override {
    if (stream_ != nullptr) {
      cudaStreamDestroy(stream_);
      stream_ = nullptr;
    }
  }

 private:
  void on_image(const ros2_cuda_ipc_core::view::ImageView& view) {
    NvtxScopedRange callback_range("PreviewNode::on_image");

    ++received_;
    if ((received_ - 1) % copy_every_n_ != 0) {
      return;
    }

    if (!view.core.valid()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Skipping invalid GPU image");
      return;
    }
    if (!view.sanity_check()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Skipping GPU image with invalid layout");
      return;
    }
    if (view.dtype != ros2_cuda_ipc_core::view::DType::U8 ||
        view.channels() != kChannels) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Skipping image with unsupported dtype/channels");
      return;
    }

    if (ensure_stream(view.core.device_id) != cudaSuccess) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Failed to prepare CUDA stream for preview");
      return;
    }

    sensor_msgs::msg::Image msg;
    msg.header = view.header;
    msg.height = view.rows();
    msg.width = view.cols();
    msg.encoding = view.encoding.empty() ? std::string("rgba8") : view.encoding;
    msg.is_bigendian = false;
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(view.cols() *
                                                                kBytesPerPixel);
    msg.data.resize(static_cast<std::size_t>(msg.step) * msg.height);

    cudaEvent_t copy_start = nullptr;
    cudaEvent_t copy_stop = nullptr;
    if (cudaEventCreate(&copy_start) != cudaSuccess ||
        cudaEventCreate(&copy_stop) != cudaSuccess) {
      RCLCPP_WARN(get_logger(),
                  "Failed to create CUDA events for preview copy");
      if (copy_start != nullptr) {
        cudaEventDestroy(copy_start);
      }
      if (copy_stop != nullptr) {
        cudaEventDestroy(copy_stop);
      }
      return;
    }

    cudaError_t err = cudaSuccess;
    {
      NvtxScopedRange wait_range("PreviewNode::wait_input_event");
      err = view.enqueue_ready_event(stream_);
    }
    if (err != cudaSuccess) {
      RCLCPP_WARN(get_logger(), "cudaStreamWaitEvent failed: %s",
                  ros2_cuda_ipc_core::cuda::cuda_error_to_string(err).c_str());
      cudaEventDestroy(copy_start);
      cudaEventDestroy(copy_stop);
      return;
    }

    {
      NvtxScopedRange copy_range("PreviewNode::cudaMemcpy2DAsync_to_host");
      cudaEventRecord(copy_start, stream_);
      err =
          cudaMemcpy2DAsync(msg.data.data(), msg.step, view.core.data(),
                            view.strideH(), static_cast<std::size_t>(msg.step),
                            msg.height, cudaMemcpyDeviceToHost, stream_);
      cudaEventRecord(copy_stop, stream_);
    }
    if (err != cudaSuccess) {
      RCLCPP_WARN(get_logger(), "cudaMemcpy2DAsync failed: %s",
                  ros2_cuda_ipc_core::cuda::cuda_error_to_string(err).c_str());
      cudaEventDestroy(copy_start);
      cudaEventDestroy(copy_stop);
      return;
    }

    err = cudaStreamSynchronize(stream_);
    if (err != cudaSuccess) {
      RCLCPP_WARN(get_logger(), "cudaStreamSynchronize failed: %s",
                  ros2_cuda_ipc_core::cuda::cuda_error_to_string(err).c_str());
      cudaEventDestroy(copy_start);
      cudaEventDestroy(copy_stop);
      return;
    }

    float copy_ms = 0.0f;
    if (cudaEventElapsedTime(&copy_ms, copy_start, copy_stop) != cudaSuccess) {
      copy_ms = 0.0f;
    }
    cudaEventDestroy(copy_start);
    cudaEventDestroy(copy_stop);

    {
      NvtxScopedRange publish_range("PreviewNode::publish_sensor_msgs_image");
      publisher_->publish(std::move(msg));
    }

    if (received_ % log_every_n_ == 0) {
      RCLCPP_INFO(get_logger(), "Preview copy_ms=%.3f received=%zu", copy_ms,
                  received_);
    }
  }

  cudaError_t ensure_stream(int device_id) {
    if (stream_ != nullptr && current_device_id_ == device_id) {
      return cudaSuccess;
    }

    if (stream_ != nullptr) {
      cudaStreamDestroy(stream_);
      stream_ = nullptr;
    }

    cudaError_t err = cudaSetDevice(device_id);
    if (err != cudaSuccess) {
      return err;
    }

    err = cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking);
    if (err != cudaSuccess) {
      return err;
    }

    current_device_id_ = device_id;
    return cudaSuccess;
  }

  rclcpp::Subscription<ros2_cuda_ipc_core::view::ImageView>::SharedPtr
      subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  cudaStream_t stream_ = nullptr;
  int current_device_id_ = -1;
  std::size_t received_ = 0;
  std::size_t copy_every_n_ = 1;
  std::size_t log_every_n_ = 30;
  std::string input_topic_name_;
  std::string output_topic_name_;
};

}  // namespace multi_process_image_fanout

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<multi_process_image_fanout::PreviewNode>();
    rclcpp::spin(node);
  } catch (const std::exception& ex) {
    RCLCPP_FATAL(rclcpp::get_logger("preview_node"), "Exception: %s",
                 ex.what());
  }
  rclcpp::shutdown();
  return 0;
}
