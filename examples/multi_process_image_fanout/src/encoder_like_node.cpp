// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <cuda_runtime_api.h>

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "multi_process_image_fanout/cuda_checks.hpp"
#include "multi_process_image_fanout/kernels.hpp"
#include "multi_process_image_fanout/status_format.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_cuda_ipc_core/cuda/nvtx_scoped_range.hpp"
#include "ros2_cuda_ipc_core/type_adapters.hpp"
#include "std_msgs/msg/string.hpp"

namespace multi_process_image_fanout {
namespace {

constexpr uint32_t kChannels = 4;

}  // namespace

using ros2_cuda_ipc_core::cuda::NvtxScopedRange;

class EncoderLikeNode : public rclcpp::Node {
 public:
  EncoderLikeNode()
      : rclcpp::Node("encoder_like_node",
                     rclcpp::NodeOptions().use_intra_process_comms(false)),
        input_topic_name_(declare_parameter<std::string>("input_topic_name",
                                                         "/fanout/image_gpu")),
        status_topic_name_(declare_parameter<std::string>(
            "status_topic_name", "/fanout/encoder_like/status")),
        log_every_n_(static_cast<std::size_t>(
            declare_parameter<int>("log_every_n", 30))),
        downscale_(declare_parameter<int>("downscale", 2)) {
    if (log_every_n_ == 0) {
      log_every_n_ = 1;
    }
    if (downscale_ != 2) {
      RCLCPP_WARN(get_logger(),
                  "Unsupported downscale=%d; using downscale=2 instead",
                  downscale_);
      downscale_ = 2;
    }

    rclcpp::PublisherOptions pub_options;
    pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    status_publisher_ = create_publisher<std_msgs::msg::String>(
        status_topic_name_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        pub_options);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    subscription_ = create_subscription<ros2_cuda_ipc_core::view::ImageView>(
        input_topic_name_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        [this](const ros2_cuda_ipc_core::view::ImageView& view) {
          on_image(view);
        },
        sub_options);

    RCLCPP_INFO(get_logger(),
                "Encoder-like node subscribed to %s and publishes %s",
                input_topic_name_.c_str(), status_topic_name_.c_str());
  }

  ~EncoderLikeNode() override { cleanup_cuda_state(); }

 private:
  void on_image(const ros2_cuda_ipc_core::view::ImageView& view) {
    NvtxScopedRange callback_range("EncoderLikeNode::on_image");

    ++received_;

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
    if (view.rows() < 2 || view.cols() < 2) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Skipping image that is too small for downscale");
      return;
    }

    if (!ensure_cuda_state(view.core.device_id)) {
      return;
    }
    if (!ensure_buffers(view)) {
      return;
    }

    cudaEvent_t kernel_start = nullptr;
    cudaEvent_t kernel_stop = nullptr;
    if (!log_cuda_error(get_logger(), "cudaEventCreate",
                        cudaEventCreate(&kernel_start)) ||
        !log_cuda_error(get_logger(), "cudaEventCreate",
                        cudaEventCreate(&kernel_stop))) {
      if (kernel_start != nullptr) {
        cudaEventDestroy(kernel_start);
      }
      if (kernel_stop != nullptr) {
        cudaEventDestroy(kernel_stop);
      }
      return;
    }

    cudaError_t err =
        cudaMemsetAsync(device_checksum_, 0, sizeof(uint64_t), stream_);
    if (err != cudaSuccess) {
      RCLCPP_WARN(get_logger(), "cudaMemsetAsync failed: %s",
                  cuda_error_to_string(err).c_str());
      cudaEventDestroy(kernel_start);
      cudaEventDestroy(kernel_stop);
      return;
    }

    {
      NvtxScopedRange wait_range("EncoderLikeNode::wait_input_event");
      err = view.enqueue_ready_event(stream_);
    }
    if (err != cudaSuccess) {
      RCLCPP_WARN(get_logger(), "cudaStreamWaitEvent failed: %s",
                  cuda_error_to_string(err).c_str());
      cudaEventDestroy(kernel_start);
      cudaEventDestroy(kernel_stop);
      return;
    }

    err = cudaEventRecord(kernel_start, stream_);
    if (err != cudaSuccess) {
      RCLCPP_WARN(get_logger(), "cudaEventRecord failed: %s",
                  cuda_error_to_string(err).c_str());
      cudaEventDestroy(kernel_start);
      cudaEventDestroy(kernel_stop);
      return;
    }

    {
      NvtxScopedRange kernel_range(
          "EncoderLikeNode::rgba_to_luma_downscale2_kernel");
      err = launch_rgba_to_luma_downscale2_kernel(
          static_cast<const uint8_t*>(view.core.data()), device_luma_,
          static_cast<int>(view.cols()), static_cast<int>(view.rows()),
          view.strideH(), stream_);
    }
    if (err != cudaSuccess) {
      RCLCPP_WARN(get_logger(),
                  "launch_rgba_to_luma_downscale2_kernel failed: %s",
                  cuda_error_to_string(err).c_str());
      cudaEventDestroy(kernel_start);
      cudaEventDestroy(kernel_stop);
      return;
    }

    {
      NvtxScopedRange checksum_range("EncoderLikeNode::checksum_u8_kernel");
      err = launch_checksum_u8_kernel(device_luma_, output_pixel_count_,
                                      device_checksum_, stream_);
    }
    if (err != cudaSuccess) {
      RCLCPP_WARN(get_logger(), "launch_checksum_u8_kernel failed: %s",
                  cuda_error_to_string(err).c_str());
      cudaEventDestroy(kernel_start);
      cudaEventDestroy(kernel_stop);
      return;
    }

    err = cudaEventRecord(kernel_stop, stream_);
    if (err != cudaSuccess) {
      RCLCPP_WARN(get_logger(), "cudaEventRecord failed: %s",
                  cuda_error_to_string(err).c_str());
      cudaEventDestroy(kernel_start);
      cudaEventDestroy(kernel_stop);
      return;
    }

    err = cudaStreamSynchronize(stream_);
    if (err != cudaSuccess) {
      RCLCPP_WARN(get_logger(), "cudaStreamSynchronize failed: %s",
                  cuda_error_to_string(err).c_str());
      cudaEventDestroy(kernel_start);
      cudaEventDestroy(kernel_stop);
      return;
    }

    if (!log_cuda_error(get_logger(), "cudaMemcpy",
                        cudaMemcpy(&host_checksum_, device_checksum_,
                                   sizeof(uint64_t), cudaMemcpyDeviceToHost))) {
      cudaEventDestroy(kernel_start);
      cudaEventDestroy(kernel_stop);
      return;
    }

    float kernel_ms = 0.0f;
    err = cudaEventElapsedTime(&kernel_ms, kernel_start, kernel_stop);
    if (err != cudaSuccess) {
      RCLCPP_WARN(get_logger(), "cudaEventElapsedTime failed: %s",
                  cuda_error_to_string(err).c_str());
      kernel_ms = 0.0f;
    }

    cudaEventDestroy(kernel_start);
    cudaEventDestroy(kernel_stop);

    const int64_t stamp_ns = rclcpp::Time(view.header.stamp).nanoseconds();
    std_msgs::msg::String status_msg;
    status_msg.data = format_encoder_status(
        received_, stamp_ns, static_cast<int>(output_width_),
        static_cast<int>(output_height_), host_checksum_, kernel_ms);
    const std::string status_text = status_msg.data;

    {
      NvtxScopedRange publish_range("EncoderLikeNode::publish_status");
      status_publisher_->publish(std::move(status_msg));
    }

    if (received_ % log_every_n_ == 0) {
      RCLCPP_INFO(get_logger(), "Encoder status published: %s",
                  status_text.c_str());
    }
  }

  bool ensure_cuda_state(int device_id) {
    if (stream_ != nullptr && current_device_id_ == device_id) {
      return true;
    }

    cleanup_cuda_state();

    const cudaError_t set_device_err = cudaSetDevice(device_id);
    if (set_device_err != cudaSuccess) {
      RCLCPP_WARN(get_logger(), "cudaSetDevice failed: %s",
                  cuda_error_to_string(set_device_err).c_str());
      return false;
    }

    const cudaError_t stream_err =
        cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking);
    if (stream_err != cudaSuccess) {
      RCLCPP_WARN(get_logger(), "cudaStreamCreateWithFlags failed: %s",
                  cuda_error_to_string(stream_err).c_str());
      stream_ = nullptr;
      return false;
    }

    const cudaError_t checksum_alloc_err = cudaMalloc(
        reinterpret_cast<void**>(&device_checksum_), sizeof(uint64_t));
    if (checksum_alloc_err != cudaSuccess) {
      RCLCPP_WARN(get_logger(), "cudaMalloc for checksum failed: %s",
                  cuda_error_to_string(checksum_alloc_err).c_str());
      cudaStreamDestroy(stream_);
      stream_ = nullptr;
      return false;
    }

    current_device_id_ = device_id;
    return true;
  }

  bool ensure_buffers(const ros2_cuda_ipc_core::view::ImageView& view) {
    const uint32_t required_width = view.cols() / 2;
    const uint32_t required_height = view.rows() / 2;
    const std::size_t required_bytes =
        static_cast<std::size_t>(required_width) * required_height;

    if (required_width == 0 || required_height == 0) {
      RCLCPP_WARN(get_logger(), "Downscaled output would be empty");
      return false;
    }

    if (device_luma_ != nullptr && required_width == output_width_ &&
        required_height == output_height_) {
      return true;
    }

    NvtxScopedRange buffer_range("EncoderLikeNode::ensure_buffers");

    if (stream_ == nullptr) {
      return false;
    }

    cudaError_t err = cudaStreamSynchronize(stream_);
    if (err != cudaSuccess) {
      RCLCPP_WARN(get_logger(), "cudaStreamSynchronize failed: %s",
                  cuda_error_to_string(err).c_str());
      return false;
    }

    if (device_luma_ != nullptr) {
      err = cudaFree(device_luma_);
      if (err != cudaSuccess) {
        RCLCPP_WARN(get_logger(), "cudaFree failed: %s",
                    cuda_error_to_string(err).c_str());
        return false;
      }
      device_luma_ = nullptr;
    }

    err = cudaMalloc(reinterpret_cast<void**>(&device_luma_), required_bytes);
    if (err != cudaSuccess) {
      RCLCPP_WARN(get_logger(), "cudaMalloc for luma buffer failed: %s",
                  cuda_error_to_string(err).c_str());
      return false;
    }

    output_width_ = required_width;
    output_height_ = required_height;
    output_pixel_count_ = required_bytes;
    return true;
  }

  void cleanup_cuda_state() {
    if (stream_ != nullptr) {
      if (current_device_id_ >= 0) {
        const cudaError_t set_device_err = cudaSetDevice(current_device_id_);
        if (set_device_err != cudaSuccess) {
          RCLCPP_WARN(get_logger(), "cudaSetDevice during cleanup failed: %s",
                      cuda_error_to_string(set_device_err).c_str());
        }
      }
      cudaStreamSynchronize(stream_);
    }

    if (device_luma_ != nullptr) {
      const cudaError_t free_err = cudaFree(device_luma_);
      if (free_err != cudaSuccess) {
        RCLCPP_WARN(get_logger(), "cudaFree for luma buffer failed: %s",
                    cuda_error_to_string(free_err).c_str());
      }
      device_luma_ = nullptr;
    }

    if (device_checksum_ != nullptr) {
      const cudaError_t free_err = cudaFree(device_checksum_);
      if (free_err != cudaSuccess) {
        RCLCPP_WARN(get_logger(), "cudaFree for checksum failed: %s",
                    cuda_error_to_string(free_err).c_str());
      }
      device_checksum_ = nullptr;
    }

    if (stream_ != nullptr) {
      const cudaError_t destroy_err = cudaStreamDestroy(stream_);
      if (destroy_err != cudaSuccess) {
        RCLCPP_WARN(get_logger(), "cudaStreamDestroy failed: %s",
                    cuda_error_to_string(destroy_err).c_str());
      }
      stream_ = nullptr;
    }

    current_device_id_ = -1;
    output_width_ = 0;
    output_height_ = 0;
    output_pixel_count_ = 0;
  }

  rclcpp::Subscription<ros2_cuda_ipc_core::view::ImageView>::SharedPtr
      subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  cudaStream_t stream_ = nullptr;
  uint8_t* device_luma_ = nullptr;
  uint64_t* device_checksum_ = nullptr;
  int current_device_id_ = -1;
  uint32_t output_width_ = 0;
  uint32_t output_height_ = 0;
  std::size_t output_pixel_count_ = 0;
  std::string input_topic_name_;
  std::string status_topic_name_;
  std::size_t received_ = 0;
  std::size_t log_every_n_ = 30;
  int downscale_ = 2;
  uint64_t host_checksum_ = 0;
};

}  // namespace multi_process_image_fanout

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<multi_process_image_fanout::EncoderLikeNode>();
    rclcpp::spin(node);
  } catch (const std::exception& ex) {
    RCLCPP_FATAL(rclcpp::get_logger("encoder_like_node"), "Exception: %s",
                 ex.what());
  }
  rclcpp::shutdown();
  return 0;
}
