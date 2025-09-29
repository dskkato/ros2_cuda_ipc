#include <cuda_runtime_api.h>

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ros2_cuda_ipc_core/image_view.hpp"
#include "ros2_cuda_ipc_core/nvtx_scoped_range.hpp"
#include "ros2_cuda_ipc_core/type_adapters.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

namespace gpu_image_transport {

using ros2_cuda_ipc_core::NvtxScopedRange;

namespace {

std::string cuda_error_to_string(cudaError_t err) {
  const char *error_string = cudaGetErrorString(err);
  return error_string ? std::string(error_string) : std::string{};
}

}  // namespace

class GpuImageTransportCompressedNode : public rclcpp::Node {
 public:
  GpuImageTransportCompressedNode()
      : rclcpp::Node("gpu_image_transport_compressed"),
        input_topic_(
            declare_parameter<std::string>("input_topic_name", "image_gpu")),
        output_topic_(
            declare_parameter<std::string>("cpu_topic_name", "image")),
        compressed_format_(
            declare_parameter<std::string>("compressed_format", "jpeg")),
        jpeg_quality_(declare_parameter<int>("jpeg_quality", 95)) {
    const cudaError_t err =
        cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking);
    if (err != cudaSuccess) {
      throw std::runtime_error("cudaStreamCreateWithFlags failed: " +
                               cuda_error_to_string(err));
    }

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.use_intra_process_comm =
        rclcpp::IntraProcessSetting::Disable;
    subscription_ = create_subscription<ros2_cuda_ipc_core::ImageView>(
        input_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
        std::bind(&GpuImageTransportCompressedNode::on_image, this,
                  std::placeholders::_1),
        subscription_options);

    const std::string format_lower = to_lower(compressed_format_);
    compressed_extension_ = select_extension(format_lower);
    if (compressed_extension_.empty()) {
      RCLCPP_WARN(get_logger(),
                  "Compression format '%s' is not recognized; publishing will"
                  " fail until a supported format (jpeg/png/bmp) is set",
                  compressed_format_.c_str());
    }
    configure_compression_params(format_lower);

    publisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(
        output_topic_, rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(),
                "gpu_image_transport_compressed listening on %s, publishing %s"
                " (format=%s)",
                input_topic_.c_str(), output_topic_.c_str(),
                compressed_format_.c_str());
  }

  ~GpuImageTransportCompressedNode() override {
    release_pinned_host();
    if (stream_) {
      const cudaError_t err = cudaStreamDestroy(stream_);
      if (err != cudaSuccess) {
        RCLCPP_ERROR(get_logger(), "cudaStreamDestroy failed: %s",
                     cuda_error_to_string(err).c_str());
      }
      stream_ = nullptr;
    }
  }

 private:
  void on_image(const ros2_cuda_ipc_core::ImageView &view) {
    NvtxScopedRange callback_range("GpuImageTransportCompressedNode::on_image");
    if (!view.core.valid()) {
      RCLCPP_WARN(get_logger(), "Received invalid GPU image view");
      return;
    }

    if (!view.sanity_check()) {
      RCLCPP_WARN(
          get_logger(),
          "Skipping frame: failed sanity check rows=%u cols=%u channels=%u",
          view.rows(), view.cols(), view.channels());
      return;
    }

    cudaError_t err = cudaSuccess;
    {
      NvtxScopedRange wait_range(
          "GpuImageTransportCompressedNode::stream_wait_event");
      err = view.enqueue_ready_event(stream_);
    }
    if (err != cudaSuccess) {
      RCLCPP_ERROR(get_logger(), "cudaStreamWaitEvent failed: %s",
                   cuda_error_to_string(err).c_str());
      return;
    }

    const std::uint64_t bytes_to_copy = view.core.byte_size;
    if (bytes_to_copy == 0) {
      return;
    }

    {
      NvtxScopedRange ensure_range(
          "GpuImageTransportCompressedNode::ensure_pinned_host");
      const cudaError_t ensure_err = ensure_pinned_capacity(bytes_to_copy);
      if (ensure_err != cudaSuccess) {
        RCLCPP_ERROR(get_logger(), "cudaMallocHost failed: %s",
                     cuda_error_to_string(ensure_err).c_str());
        return;
      }
    }

    {
      NvtxScopedRange memcpy_range(
          "GpuImageTransportCompressedNode::cudaMemcpyAsync");
      err = cudaMemcpyAsync(pinned_host_buffer_, view.core.data<uint8_t>(),
                            bytes_to_copy, cudaMemcpyDeviceToHost, stream_);
    }
    if (err != cudaSuccess) {
      RCLCPP_ERROR(get_logger(), "cudaMemcpyAsync failed: %s",
                   cuda_error_to_string(err).c_str());
      return;
    }

    {
      NvtxScopedRange sync_range(
          "GpuImageTransportCompressedNode::cudaStreamSynchronize");
      err = cudaStreamSynchronize(stream_);
    }
    if (err != cudaSuccess) {
      RCLCPP_ERROR(get_logger(), "cudaStreamSynchronize failed: %s",
                   cuda_error_to_string(err).c_str());
      return;
    }

    publish_compressed_image(view, bytes_to_copy);
  }

  void publish_compressed_image(const ros2_cuda_ipc_core::ImageView &view,
                                std::uint64_t available_bytes) {
    const std::uint32_t height = view.rows();
    const std::uint64_t step_bytes = view.strideH();
    if (height == 0 || step_bytes == 0) {
      RCLCPP_WARN(get_logger(),
                  "Skipping compressed publish: invalid dimensions height=%u "
                  "width=%u step=%zu",
                  view.rows(), view.cols(), step_bytes);
      return;
    }

    if (step_bytes > std::numeric_limits<uint32_t>::max()) {
      RCLCPP_WARN(get_logger(),
                  "Skipping compressed publish: step too large step=%zu "
                  "exceeds uint32_t",
                  step_bytes);
      return;
    }

    if (height > 0 &&
        step_bytes > std::numeric_limits<std::size_t>::max() / height) {
      RCLCPP_WARN(
          get_logger(),
          "Skipping compressed publish: size overflow height=%u step=%zu",
          height, step_bytes);
      return;
    }

    const std::uint64_t required_bytes = step_bytes * height;
    if (available_bytes < required_bytes) {
      RCLCPP_WARN(get_logger(),
                  "Skipping compressed publish: not enough data copied (have "
                  "%zu need %zu)",
                  available_bytes, required_bytes);
      return;
    }

    using ros2_cuda_ipc_core::DType;
    if (view.dtype != DType::U8) {
      RCLCPP_WARN(
          get_logger(),
          "Skipping compressed publish: unsupported dtype %d (expected U8)",
          static_cast<int>(view.dtype));
      return;
    }

    const int channels = static_cast<int>(view.channels());
    if (channels <= 0) {
      RCLCPP_WARN(get_logger(),
                  "Skipping compressed publish: invalid channel count %d",
                  channels);
      return;
    }

    const int cv_type = CV_MAKETYPE(CV_8U, channels);
    if (cv_type < 0) {
      RCLCPP_WARN(get_logger(),
                  "Skipping compressed publish: unable to build OpenCV type "
                  "for channels=%d",
                  channels);
      return;
    }

    if (compressed_extension_.empty()) {
      RCLCPP_WARN(get_logger(),
                  "Skipping compressed publish: unsupported format '%s'",
                  compressed_format_.c_str());
      return;
    }

    const auto width = view.cols();
    if (width > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
        height > static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
      RCLCPP_WARN(get_logger(),
                  "Skipping compressed publish: dimensions exceed OpenCV "
                  "limits width=%u height=%u",
                  width, height);
      return;
    }

    cv::Mat host_view(static_cast<int>(height), static_cast<int>(width),
                      cv_type, pinned_host_buffer_, step_bytes);

    sensor_msgs::msg::CompressedImage msg;
    msg.header = view.header;
    msg.format = compressed_format_;

    if (!cv::imencode(compressed_extension_, host_view, msg.data,
                      compression_params_)) {
      RCLCPP_ERROR(get_logger(), "cv::imencode failed for format '%s'",
                   compressed_format_.c_str());
      return;
    }

    publisher_->publish(std::move(msg));
  }

  cudaError_t ensure_pinned_capacity(std::size_t bytes) {
    if (bytes == 0) {
      return cudaSuccess;
    }
    if (bytes <= pinned_host_capacity_) {
      return cudaSuccess;
    }
    release_pinned_host();
    const cudaError_t err =
        cudaMallocHost(reinterpret_cast<void **>(&pinned_host_buffer_), bytes);
    if (err == cudaSuccess) {
      pinned_host_capacity_ = bytes;
    }
    return err;
  }

  void release_pinned_host() {
    if (pinned_host_buffer_) {
      const cudaError_t err = cudaFreeHost(pinned_host_buffer_);
      if (err != cudaSuccess) {
        RCLCPP_ERROR(get_logger(), "cudaFreeHost failed: %s",
                     cuda_error_to_string(err).c_str());
      }
      pinned_host_buffer_ = nullptr;
      pinned_host_capacity_ = 0;
    }
  }

  std::string to_lower(std::string value) const {
    for (char &ch : value) {
      ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    return value;
  }

  std::string select_extension(const std::string &format_lower) const {
    if (format_lower == "jpeg" || format_lower == "jpg") {
      return ".jpg";
    }
    if (format_lower == "png") {
      return ".png";
    }
    if (format_lower == "bmp") {
      return ".bmp";
    }
    return std::string{};
  }

  void configure_compression_params(const std::string &format_lower) {
    compression_params_.clear();
    if (format_lower == "jpeg" || format_lower == "jpg") {
      const int quality = std::clamp(jpeg_quality_, 0, 100);
      if (quality != jpeg_quality_) {
        RCLCPP_WARN(
            get_logger(),
            "jpeg_quality parameter (%d) out of range [0, 100]; clamped to %d",
            jpeg_quality_, quality);
      }
      jpeg_quality_ = quality;
      compression_params_.push_back(cv::IMWRITE_JPEG_QUALITY);
      compression_params_.push_back(jpeg_quality_);
      return;
    }
    if (format_lower == "png") {
      constexpr int kDefaultPngLevel = 3;
      compression_params_.push_back(cv::IMWRITE_PNG_COMPRESSION);
      compression_params_.push_back(kDefaultPngLevel);
      return;
    }
    if (format_lower == "bmp") {
      return;
    }
    RCLCPP_WARN(
        get_logger(),
        "Compression format '%s' not explicitly supported; using defaults",
        format_lower.c_str());
  }

  rclcpp::Subscription<ros2_cuda_ipc_core::ImageView>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
  cudaStream_t stream_ = nullptr;
  std::string input_topic_;
  std::string output_topic_;
  std::string compressed_format_;
  std::string compressed_extension_;
  int jpeg_quality_ = 95;
  std::vector<int> compression_params_;
  uint8_t *pinned_host_buffer_ = nullptr;
  std::size_t pinned_host_capacity_ = 0;
};

}  // namespace gpu_image_transport

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<
        gpu_image_transport::GpuImageTransportCompressedNode>();
    rclcpp::spin(node);
  } catch (const std::exception &ex) {
    RCLCPP_FATAL(rclcpp::get_logger("gpu_image_transport_compressed"),
                 "Exception: %s", ex.what());
  }
  rclcpp::shutdown();
  return 0;
}
