#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <limits>
#include <string>
#include <utility>

#include "gpu_image_transport/gpu_image_transport_base.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace gpu_image_transport {

class GpuImageTransportNode : public GpuImageTransportNodeBase {
 public:
  GpuImageTransportNode() : GpuImageTransportNodeBase("gpu_image_transport") {
    publisher_ = create_publisher<sensor_msgs::msg::Image>(
        output_topic(), rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(),
                "gpu_image_transport listening on %s, publishing %s",
                input_topic().c_str(), output_topic().c_str());
  }

 private:
  void publish_frame(const ros2_cuda_ipc_core::ImageView& view,
                     std::uint64_t available_bytes) override {
    const std::uint32_t height = view.rows();
    const std::uint64_t step_bytes = view.strideH();
    if (height == 0 || step_bytes == 0) {
      RCLCPP_WARN(
          get_logger(),
          "Skipping publish: invalid dimensions height=%u width=%u step=%zu",
          view.rows(), view.cols(), step_bytes);
      return;
    }

    if (step_bytes > std::numeric_limits<uint32_t>::max()) {
      RCLCPP_WARN(get_logger(),
                  "Skipping publish: step too large step=%zu exceeds uint32_t",
                  step_bytes);
      return;
    }

    if (height > 0 &&
        step_bytes > std::numeric_limits<std::size_t>::max() / height) {
      RCLCPP_WARN(get_logger(),
                  "Skipping publish: size overflow height=%u step=%zu", height,
                  step_bytes);
      return;
    }

    const std::uint64_t required_bytes = step_bytes * height;
    if (available_bytes < required_bytes) {
      RCLCPP_WARN(
          get_logger(),
          "Skipping publish: not enough data copied (have %zu need %zu)",
          available_bytes, required_bytes);
      return;
    }

    sensor_msgs::msg::Image msg;
    msg.header = view.header;
    msg.height = view.rows();
    msg.width = view.cols();
    msg.encoding = view.encoding.empty() ? infer_encoding(view) : view.encoding;
    msg.is_bigendian = false;
    msg.step = static_cast<uint32_t>(step_bytes);
    msg.data.resize(required_bytes);
    std::memcpy(msg.data.data(), host_buffer_data(), required_bytes);

    publisher_->publish(std::move(msg));
  }

  std::string infer_encoding(const ros2_cuda_ipc_core::ImageView& view) const {
    const auto channels = view.channels();
    const auto suffix = std::to_string(channels);
    using ros2_cuda_ipc_core::DType;
    switch (view.dtype) {
      case DType::U8:
        if (channels == 1) {
          return "mono8";
        }
        if (channels == 3) {
          return "rgb8";
        }
        if (channels == 4) {
          return "rgba8";
        }
        return "8UC" + suffix;
      case DType::U16:
        if (channels == 1) {
          return "mono16";
        }
        return "16UC" + suffix;
      case DType::F16:
        return "16FC" + suffix;
      case DType::F32:
        return "32FC" + suffix;
      case DType::F64:
        return "64FC" + suffix;
      case DType::S16:
        return "16SC" + suffix;
      case DType::S32:
        return "32SC" + suffix;
      case DType::U32:
        return "32UC" + suffix;
    }
    return "8UC" + suffix;
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

}  // namespace gpu_image_transport

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<gpu_image_transport::GpuImageTransportNode>();
    rclcpp::spin(node);
  } catch (const std::exception& ex) {
    RCLCPP_FATAL(rclcpp::get_logger("gpu_image_transport"), "Exception: %s",
                 ex.what());
  }
  rclcpp::shutdown();
  return 0;
}
