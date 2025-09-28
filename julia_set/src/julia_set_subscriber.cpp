#include <cuda_runtime_api.h>

#include <algorithm>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "julia_set/nvtx_scoped_range.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_cuda_ipc_core/image_view.hpp"
#include "ros2_cuda_ipc_core/type_adapters.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace julia_set {

class JuliaSetSubscriberNode : public rclcpp::Node {
 public:
  JuliaSetSubscriberNode()
      : rclcpp::Node("julia_set_subscriber",
                     rclcpp::NodeOptions().use_intra_process_comms(false)),
        topic_name_(
            declare_parameter<std::string>("topic_name", "julia_set/image")),
        publish_topic_(declare_parameter<std::string>("cpu_topic_name",
                                                      "julia_set/image_cpu")),
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

    image_pub_ = create_publisher<sensor_msgs::msg::Image>(
        publish_topic_, rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(), "Listening for Julia set frames on %s",
                topic_name_.c_str());
    RCLCPP_INFO(get_logger(), "Publishing CPU images on %s",
                publish_topic_.c_str());
  }

  ~JuliaSetSubscriberNode() override {
    release_pinned_host();
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
    NvtxScopedRange callback_range("JuliaSetSubscriberNode::on_image");
    if (!view.core.valid()) {
      RCLCPP_WARN(get_logger(), "Received invalid Julia set view");
      return;
    }
    if (!view.sanity_check()) {
      RCLCPP_WARN(
          get_logger(),
          "Skipping frame: failed sanity check rows=%u cols=%u channels=%u",
          view.rows(), view.cols(), view.channels());
      ++frame_counter_;
      return;
    }

    auto err = cudaSuccess;
    {
      NvtxScopedRange wait_range("JuliaSetSubscriber::stream_wait_event");
      err = view.enqueue_ready_event(stream_);
    }
    if (err != cudaSuccess) {
      RCLCPP_ERROR(get_logger(), "cudaStreamWaitEvent failed: %s",
                   cudaGetErrorString(err));
      return;
    }

    const bool has_subscribers =
        image_pub_ && (image_pub_->get_subscription_count() > 0 ||
                       image_pub_->get_intra_process_subscription_count() > 0);
    const bool need_full_frame = has_subscribers || log_full_copy_;
    const std::size_t requested_sample_bytes =
        log_full_copy_
            ? static_cast<std::size_t>(view.core.byte_size)
            : std::min<std::size_t>(view.core.byte_size, sample_bytes_);

    std::size_t bytes_to_copy = 0;
    std::size_t host_step_bytes = 0;

    if (need_full_frame) {
      const std::size_t height = static_cast<std::size_t>(view.rows());
      const std::size_t step_candidate =
          static_cast<std::size_t>(view.cols()) *
          static_cast<std::size_t>(view.strideW());
      if (height == 0 || step_candidate == 0) {
        RCLCPP_WARN(
            get_logger(),
            "Skipping frame: invalid dimensions height=%u width=%u channels=%u",
            view.rows(), view.cols(), view.channels());
        ++frame_counter_;
        return;
      }

      const std::size_t src_pitch = static_cast<std::size_t>(view.strideH());
      if (src_pitch < step_candidate) {
        RCLCPP_WARN(get_logger(),
                    "Skipping frame: stride mismatch strideH=%llu strideW=%llu",
                    static_cast<unsigned long long>(view.strideH()),
                    static_cast<unsigned long long>(view.strideW()));
        ++frame_counter_;
        return;
      }

      if (step_candidate > 0 &&
          height > std::numeric_limits<std::size_t>::max() / step_candidate) {
        RCLCPP_ERROR(get_logger(),
                     "Skipping frame: size overflow height=%zu step=%zu",
                     height, step_candidate);
        ++frame_counter_;
        return;
      }

      host_step_bytes = step_candidate;
      bytes_to_copy = step_candidate * height;

      {
        NvtxScopedRange ensure_range("JuliaSetSubscriber::ensure_pinned_host");
        const cudaError_t ensure_err = ensure_pinned_capacity(bytes_to_copy);
        if (ensure_err != cudaSuccess) {
          RCLCPP_ERROR(get_logger(), "cudaMallocHost failed: %s",
                       cudaGetErrorString(ensure_err));
          return;
        }
      }

      NvtxScopedRange memcpy_range("JuliaSetSubscriber::cudaMemcpyAsync");
      err = cudaMemcpy2DAsync(
          pinned_host_buffer_, host_step_bytes, view.core.data<uint8_t>(),
          src_pitch, host_step_bytes, height, cudaMemcpyDeviceToHost, stream_);
    } else {
      bytes_to_copy = requested_sample_bytes;
      if (bytes_to_copy == 0) {
        ++frame_counter_;
        return;
      }

      {
        NvtxScopedRange ensure_range("JuliaSetSubscriber::ensure_pinned_host");
        const cudaError_t ensure_err = ensure_pinned_capacity(bytes_to_copy);
        if (ensure_err != cudaSuccess) {
          RCLCPP_ERROR(get_logger(), "cudaMallocHost failed: %s",
                       cudaGetErrorString(ensure_err));
          return;
        }
      }

      NvtxScopedRange memcpy_range("JuliaSetSubscriber::cudaMemcpyAsync");
      err = cudaMemcpyAsync(pinned_host_buffer_, view.core.data<uint8_t>(),
                            bytes_to_copy, cudaMemcpyDeviceToHost, stream_);
    }
    if (err != cudaSuccess) {
      RCLCPP_ERROR(get_logger(), "%s failed: %s",
                   need_full_frame ? "cudaMemcpy2DAsync" : "cudaMemcpyAsync",
                   cudaGetErrorString(err));
      return;
    }

    {
      NvtxScopedRange sync_range("JuliaSetSubscriber::cudaStreamSynchronize");
      err = cudaStreamSynchronize(stream_);
    }
    if (err != cudaSuccess) {
      RCLCPP_ERROR(get_logger(), "cudaStreamSynchronize failed: %s",
                   cudaGetErrorString(err));
      return;
    }

    const std::size_t current_frame = ++frame_counter_;
    const std::size_t sample_bytes_to_log =
        std::min<std::size_t>(requested_sample_bytes, bytes_to_copy);

    if (has_subscribers) {
      publish_image(view, bytes_to_copy, host_step_bytes);
    }

    if (log_full_copy_) {
      RCLCPP_INFO(get_logger(),
                  "Frame %zu frame_id=%s encoding=%s copied=%zu bytes",
                  current_frame, view.header.frame_id.c_str(),
                  view.encoding.c_str(), bytes_to_copy);
    } else if (sample_bytes_to_log > 0) {
      RCLCPP_INFO(
          get_logger(), "Frame %zu frame_id=%s encoding=%s sample=%s",
          current_frame, view.header.frame_id.c_str(), view.encoding.c_str(),
          describe_sample(pinned_host_buffer_, sample_bytes_to_log).c_str());
    }
  }

  std::string describe_sample(const uint8_t *buffer,
                              std::size_t bytes_to_copy) const {
    constexpr std::size_t max_elements = 16;
    std::ostringstream oss;
    if (bytes_to_copy == 0) {
      return "";
    }
    const std::size_t count = std::min(bytes_to_copy, max_elements);
    for (std::size_t i = 0; i < count; ++i) {
      if (i != 0) {
        oss << ",";
      }
      oss << static_cast<unsigned>(buffer[i]);
    }
    if (bytes_to_copy > count) {
      oss << ",...";
    }
    return oss.str();
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
                     cudaGetErrorString(err));
      }
      pinned_host_buffer_ = nullptr;
      pinned_host_capacity_ = 0;
    }
  }

  void publish_image(const ros2_cuda_ipc_core::ImageView &view,
                     std::size_t available_bytes, std::size_t step_bytes) {
    if (!image_pub_) {
      return;
    }

    const std::size_t height = static_cast<std::size_t>(view.rows());
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
                  "Skipping publish: size overflow height=%zu step=%zu", height,
                  step_bytes);
      return;
    }

    const std::size_t required_bytes = step_bytes * height;
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
    std::memcpy(msg.data.data(), pinned_host_buffer_, required_bytes);

    image_pub_->publish(std::move(msg));
  }

  std::string infer_encoding(const ros2_cuda_ipc_core::ImageView &view) const {
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

  rclcpp::Subscription<ros2_cuda_ipc_core::ImageView>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  cudaStream_t stream_ = nullptr;
  std::string topic_name_;
  std::string publish_topic_;
  std::size_t sample_bytes_ = 64;
  bool log_full_copy_ = false;
  std::size_t frame_counter_ = 0;
  uint8_t *pinned_host_buffer_ = nullptr;
  std::size_t pinned_host_capacity_ = 0;
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
