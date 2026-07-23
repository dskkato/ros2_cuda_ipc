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
#include "ros2_cuda_ipc_core/detail/nvtx_scoped_range.hpp"
#include "ros2_cuda_ipc_core/image/image_view_mapper.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_image.hpp"
#include "std_msgs/msg/string.hpp"

namespace multi_process_image_fanout {
namespace {

constexpr uint32_t kChannels = 4;
constexpr uint64_t kBytesPerPixel = 4;

bool is_supported_rgba8_layout(
    const ros2_cuda_ipc_core::image::ImageView& view) noexcept {
  const uint64_t row_bytes =
      static_cast<uint64_t>(view.cols()) * kBytesPerPixel;
  return view.strideC() == 1 && view.strideW() == kBytesPerPixel &&
         view.strideH() >= row_bytes;
}

}  // namespace

using ros2_cuda_ipc_core::detail::NvtxScopedRange;

class InferenceLikeNode : public rclcpp::Node {
 public:
  InferenceLikeNode()
      : rclcpp::Node("inference_like_node",
                     rclcpp::NodeOptions().use_intra_process_comms(false)),
        input_topic_name_(declare_parameter<std::string>("input_topic_name",
                                                         "/fanout/image_gpu")),
        status_topic_name_(declare_parameter<std::string>(
            "status_topic_name", "/fanout/inference_like/status")) {
    const auto log_every_n = declare_parameter<int>("log_every_n", 30);
    if (log_every_n > 0) {
      log_every_n_ = static_cast<std::size_t>(log_every_n);
    } else {
      RCLCPP_WARN(get_logger(),
                  "log_every_n was zero or negative; defaulting to 30");
      log_every_n_ = 30;
    }

    rclcpp::PublisherOptions pub_options;
    pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    status_publisher_ = create_publisher<std_msgs::msg::String>(
        status_topic_name_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        pub_options);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    subscription_ = create_subscription<ros2_cuda_ipc_msgs::msg::GpuImage>(
        input_topic_name_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        [this](const ros2_cuda_ipc_msgs::msg::GpuImage& message) {
          auto view = ros2_cuda_ipc_core::image::map_image_view(message);
          if (!view.valid()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Skipping GPU image mapping failure");
            return;
          }
          on_image(view);
        },
        sub_options);

    RCLCPP_INFO(get_logger(),
                "Inference-like node subscribed to %s and publishes %s",
                input_topic_name_.c_str(), status_topic_name_.c_str());
  }

  ~InferenceLikeNode() override { cleanup_cuda_state(); }

 private:
  void on_image(const ros2_cuda_ipc_core::image::ImageView& view) {
    NvtxScopedRange callback_range("InferenceLikeNode::on_image");

    ++received_;

    // Validate the shared GPU view and keep the full image and normalized
    // tensor on the device.
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
    if (view.dtype != ros2_cuda_ipc_core::image::DType::U8 ||
        view.channels() != kChannels) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Skipping image with unsupported dtype/channels");
      return;
    }
    if (!is_supported_rgba8_layout(view)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Skipping image with unsupported RGBA8 stride layout");
      return;
    }
    if (view.rows() == 0 || view.cols() == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Skipping empty image");
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

    {
      // Wait on the publisher's ready event before reading the shared input
      // slot from this node's stream.
      NvtxScopedRange wait_range("InferenceLikeNode::wait_input_event");
      auto result = view.enqueue_ready_event(stream_);
      if (!result) {
        RCLCPP_WARN(get_logger(), "enqueue_ready_event failed: %s",
                    result.error().to_string().c_str());
        cudaEventDestroy(kernel_start);
        cudaEventDestroy(kernel_stop);
        return;
      }
    }

    cudaError_t err = cudaEventRecord(kernel_start, stream_);
    if (err != cudaSuccess) {
      RCLCPP_WARN(get_logger(), "cudaEventRecord failed: %s",
                  cuda_error_to_string(err).c_str());
      cudaEventDestroy(kernel_start);
      cudaEventDestroy(kernel_stop);
      return;
    }

    {
      // Run preprocessing and compact stat extraction on the GPU before copying
      // the small result struct.
      NvtxScopedRange gray_range(
          "InferenceLikeNode::rgba_to_normalized_gray_kernel");
      err = launch_rgba_to_normalized_gray_kernel(
          static_cast<const uint8_t*>(view.core.data()), device_gray_,
          static_cast<int>(view.cols()), static_cast<int>(view.rows()),
          view.strideH(), stream_);
    }
    if (err != cudaSuccess) {
      RCLCPP_WARN(get_logger(),
                  "launch_rgba_to_normalized_gray_kernel failed: %s",
                  cuda_error_to_string(err).c_str());
      cudaEventDestroy(kernel_start);
      cudaEventDestroy(kernel_stop);
      return;
    }

    {
      NvtxScopedRange stats_range("InferenceLikeNode::stats_kernel");
      err = launch_inference_stats_kernel(device_gray_, gray_element_count_,
                                          device_stats_, stream_);
    }
    if (err != cudaSuccess) {
      RCLCPP_WARN(get_logger(), "launch_inference_stats_kernel failed: %s",
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

    {
      // Only the compact stats struct crosses back to host.
      NvtxScopedRange copy_range("InferenceLikeNode::copy_stats_to_host");
      err = cudaMemcpy(&host_stats_, device_stats_, sizeof(InferenceStats),
                       cudaMemcpyDeviceToHost);
      if (err != cudaSuccess) {
        RCLCPP_WARN(get_logger(), "cudaMemcpy failed: %s",
                    cuda_error_to_string(err).c_str());
        cudaEventDestroy(kernel_start);
        cudaEventDestroy(kernel_stop);
        return;
      }
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
    status_msg.data = format_inference_status(
        received_, stamp_ns, host_stats_.mean, host_stats_.min, host_stats_.max,
        host_stats_.checksum, kernel_ms);
    const std::string status_text = status_msg.data;

    {
      // Publish the status derived from the GPU stats.
      NvtxScopedRange publish_range("InferenceLikeNode::publish_status");
      status_publisher_->publish(std::move(status_msg));
    }

    if (received_ % log_every_n_ == 0) {
      RCLCPP_INFO(get_logger(), "Inference status published: %s",
                  status_text.c_str());
    }
  }

  // Select the ImageView device and create the one non-blocking stream used for
  // event waits, preprocessing, and stats kernels. Device-local stat storage is
  // recreated when the source device changes.
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

    const cudaError_t stats_alloc_err = cudaMalloc(
        reinterpret_cast<void**>(&device_stats_), sizeof(InferenceStats));
    if (stats_alloc_err != cudaSuccess) {
      RCLCPP_WARN(get_logger(), "cudaMalloc for stats failed: %s",
                  cuda_error_to_string(stats_alloc_err).c_str());
      cudaStreamDestroy(stream_);
      stream_ = nullptr;
      return false;
    }

    current_device_id_ = device_id;
    return true;
  }

  // Lazily allocate or resize the internal normalized gray tensor when input
  // dimensions change. The tensor remains GPU-only.
  bool ensure_buffers(const ros2_cuda_ipc_core::image::ImageView& view) {
    const std::size_t required_count = static_cast<std::size_t>(view.rows()) *
                                       static_cast<std::size_t>(view.cols());
    if (required_count == 0) {
      RCLCPP_WARN(get_logger(), "Gray buffer would be empty");
      return false;
    }

    if (device_gray_ != nullptr && required_count == gray_element_count_) {
      return true;
    }

    NvtxScopedRange buffer_range("InferenceLikeNode::ensure_buffers");

    if (stream_ == nullptr) {
      return false;
    }

    cudaError_t err = cudaStreamSynchronize(stream_);
    if (err != cudaSuccess) {
      RCLCPP_WARN(get_logger(), "cudaStreamSynchronize failed: %s",
                  cuda_error_to_string(err).c_str());
      return false;
    }

    if (device_gray_ != nullptr) {
      err = cudaFree(device_gray_);
      if (err != cudaSuccess) {
        RCLCPP_WARN(get_logger(), "cudaFree failed: %s",
                    cuda_error_to_string(err).c_str());
        return false;
      }
      device_gray_ = nullptr;
    }

    err = cudaMalloc(reinterpret_cast<void**>(&device_gray_),
                     required_count * sizeof(float));
    if (err != cudaSuccess) {
      RCLCPP_WARN(get_logger(), "cudaMalloc for gray buffer failed: %s",
                  cuda_error_to_string(err).c_str());
      return false;
    }

    gray_element_count_ = required_count;
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

    if (device_gray_ != nullptr) {
      const cudaError_t free_err = cudaFree(device_gray_);
      if (free_err != cudaSuccess) {
        RCLCPP_WARN(get_logger(), "cudaFree for gray buffer failed: %s",
                    cuda_error_to_string(free_err).c_str());
      }
      device_gray_ = nullptr;
    }

    if (device_stats_ != nullptr) {
      const cudaError_t free_err = cudaFree(device_stats_);
      if (free_err != cudaSuccess) {
        RCLCPP_WARN(get_logger(), "cudaFree for stats failed: %s",
                    cuda_error_to_string(free_err).c_str());
      }
      device_stats_ = nullptr;
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
    gray_element_count_ = 0;
  }

  rclcpp::Subscription<ros2_cuda_ipc_msgs::msg::GpuImage>::SharedPtr
      subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  cudaStream_t stream_ = nullptr;
  float* device_gray_ = nullptr;
  InferenceStats* device_stats_ = nullptr;
  int current_device_id_ = -1;
  std::size_t gray_element_count_ = 0;
  std::string input_topic_name_;
  std::string status_topic_name_;
  std::size_t received_ = 0;
  std::size_t log_every_n_ = 30;
  InferenceStats host_stats_{};
};

}  // namespace multi_process_image_fanout

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node =
        std::make_shared<multi_process_image_fanout::InferenceLikeNode>();
    rclcpp::spin(node);
  } catch (const std::exception& ex) {
    RCLCPP_FATAL(rclcpp::get_logger("inference_like_node"), "Exception: %s",
                 ex.what());
  }
  rclcpp::shutdown();
  return 0;
}
