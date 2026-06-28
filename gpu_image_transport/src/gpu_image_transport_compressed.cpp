#include <algorithm>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <utility>
#include <vector>

#include "gpu_image_transport/gpu_image_transport_base.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

namespace gpu_image_transport {

class GpuImageTransportCompressedNode : public GpuImageTransportNodeBase {
 public:
  GpuImageTransportCompressedNode()
      : GpuImageTransportNodeBase("gpu_image_transport_compressed"),
        compressed_format_(
            declare_parameter<std::string>("compressed_format", "jpeg")),
        jpeg_quality_(declare_parameter<int>("jpeg_quality", 95)) {
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
        output_topic(), rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(),
                "gpu_image_transport_compressed listening on %s, publishing %s"
                " (format=%s)",
                input_topic().c_str(), output_topic().c_str(),
                compressed_format_.c_str());
  }

 private:
  void publish_frame(const ros2_cuda_ipc_core::ImageView& view,
                     std::uint64_t available_bytes) override {
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
                      cv_type, host_buffer_data(), step_bytes);

    const std::string encoding_lower = to_lower(view.encoding);
    const cv::Mat* source_view = &host_view;
    cv::Mat converted_view;
    if (encoding_lower == "rgb8" && channels == 3) {
      cv::cvtColor(host_view, converted_view, cv::COLOR_RGB2BGR);
      source_view = &converted_view;
    } else if (encoding_lower == "rgba8" && channels == 4) {
      cv::cvtColor(host_view, converted_view, cv::COLOR_RGBA2BGRA);
      source_view = &converted_view;
    }

    sensor_msgs::msg::CompressedImage msg;
    msg.header = view.header;
    msg.format = compressed_format_;

    if (!cv::imencode(compressed_extension_, *source_view, msg.data,
                      compression_params_)) {
      RCLCPP_ERROR(get_logger(), "cv::imencode failed for format '%s'",
                   compressed_format_.c_str());
      return;
    }

    publisher_->publish(std::move(msg));
  }

  std::string to_lower(std::string value) const {
    for (char& ch : value) {
      ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    return value;
  }

  std::string select_extension(const std::string& format_lower) const {
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

  void configure_compression_params(const std::string& format_lower) {
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

  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
  std::string compressed_format_;
  std::string compressed_extension_;
  int jpeg_quality_ = 95;
  std::vector<int> compression_params_;
};

}  // namespace gpu_image_transport

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<
        gpu_image_transport::GpuImageTransportCompressedNode>();
    rclcpp::spin(node);
  } catch (const std::exception& ex) {
    RCLCPP_FATAL(rclcpp::get_logger("gpu_image_transport_compressed"),
                 "Exception: %s", ex.what());
  }
  rclcpp::shutdown();
  return 0;
}
