#include <cuda_runtime_api.h>

#include <algorithm>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ros2_cuda_ipc_core/image_view.hpp"
#include "ros2_cuda_ipc_core/type_adapters.hpp"

namespace julia_set {

class JuliaSetSubscriberNode : public rclcpp::Node {
 public:
  JuliaSetSubscriberNode()
      : rclcpp::Node("julia_set_subscriber",
                     rclcpp::NodeOptions().use_intra_process_comms(false)),
        topic_name_(
            declare_parameter<std::string>("topic_name", "julia_set/image")),
        sample_bytes_(static_cast<std::size_t>(
            declare_parameter<int>("sample_bytes", 64))),
        log_full_copy_(declare_parameter<bool>("log_full_copy", false)) {
    if (sample_bytes_ == 0) {
      sample_bytes_ = 1;
    }
    cudaError_t err =
        cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking);
    if (err != cudaSuccess) {
      throw std::runtime_error(
          std::string("cudaStreamCreateWithFlags failed: ") +
          cudaGetErrorString(err));
    }

    rclcpp::SubscriptionOptions options;
    options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    subscription_ = create_subscription<ros2_cuda_ipc_core::ImageView>(
        topic_name_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        std::bind(&JuliaSetSubscriberNode::on_image, this,
                  std::placeholders::_1),
        options);

    RCLCPP_INFO(get_logger(), "Listening for Julia set frames on %s",
                topic_name_.c_str());
  }

  ~JuliaSetSubscriberNode() override {
    if (stream_) {
      const cudaError_t err = cudaStreamDestroy(stream_);
      if (err != cudaSuccess) {
        RCLCPP_ERROR(get_logger(), "cudaStreamDestroy failed: %s",
                     cudaGetErrorString(err));
      }
      stream_ = nullptr;
    }
  }

 private:
  void on_image(const ros2_cuda_ipc_core::ImageView &view) {
    if (!view.core.valid()) {
      RCLCPP_WARN(get_logger(), "Received invalid Julia set view");
      return;
    }

    auto err = view.wait(stream_);
    if (err != cudaSuccess) {
      RCLCPP_ERROR(get_logger(), "cudaStreamWaitEvent failed: %s",
                   cudaGetErrorString(err));
      return;
    }

    const std::size_t bytes_to_copy =
        log_full_copy_
            ? view.core.byte_size
            : std::min<std::size_t>(view.core.byte_size, sample_bytes_);
    std::vector<uint8_t> host(bytes_to_copy, 0);
    err = cudaMemcpyAsync(host.data(), view.core.data<uint8_t>(), bytes_to_copy,
                          cudaMemcpyDeviceToHost, stream_);
    if (err != cudaSuccess) {
      RCLCPP_ERROR(get_logger(), "cudaMemcpyAsync failed: %s",
                   cudaGetErrorString(err));
      return;
    }

    err = cudaStreamSynchronize(stream_);
    if (err != cudaSuccess) {
      RCLCPP_ERROR(get_logger(), "cudaStreamSynchronize failed: %s",
                   cudaGetErrorString(err));
      return;
    }

    if (log_full_copy_) {
      RCLCPP_INFO(get_logger(),
                  "Frame %zu frame_id=%s encoding=%s copied=%zu bytes",
                  ++frame_counter_, view.header.frame_id.c_str(),
                  view.encoding.c_str(), bytes_to_copy);
    } else if (!host.empty()) {
      RCLCPP_INFO(get_logger(), "Frame %zu frame_id=%s encoding=%s sample=%s",
                  ++frame_counter_, view.header.frame_id.c_str(),
                  view.encoding.c_str(), describe_sample(host).c_str());
    }
  }

  std::string describe_sample(const std::vector<uint8_t> &buffer) const {
    constexpr std::size_t max_elements = 16;
    std::ostringstream oss;
    const std::size_t count = std::min(buffer.size(), max_elements);
    for (std::size_t i = 0; i < count; ++i) {
      if (i != 0) {
        oss << ",";
      }
      oss << static_cast<unsigned>(buffer[i]);
    }
    if (buffer.size() > count) {
      oss << ",...";
    }
    return oss.str();
  }

  rclcpp::Subscription<ros2_cuda_ipc_core::ImageView>::SharedPtr subscription_;
  cudaStream_t stream_ = nullptr;
  std::string topic_name_;
  std::size_t sample_bytes_ = 64;
  bool log_full_copy_ = false;
  std::size_t frame_counter_ = 0;
};

}  // namespace julia_set

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<julia_set::JuliaSetSubscriberNode>();
    rclcpp::spin(node);
  } catch (const std::exception &ex) {
    RCLCPP_FATAL(rclcpp::get_logger("julia_set_subscriber"), "Exception: %s",
                 ex.what());
  }
  rclcpp::shutdown();
  return 0;
}
