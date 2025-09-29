#include <cuda_runtime_api.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros2_cuda_ipc_core/image_view.hpp"
#include "ros2_cuda_ipc_core/nvtx_scoped_range.hpp"
#include "ros2_cuda_ipc_core/type_adapters.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace gpu_image_transport {

using ros2_cuda_ipc_core::NvtxScopedRange;

namespace {

std::string cuda_error_to_string(cudaError_t err) {
  const char *error_string = cudaGetErrorString(err);
  return error_string ? std::string(error_string) : std::string{};
}

}  // namespace

class GpuImageTransportNode : public rclcpp::Node {
 public:
  GpuImageTransportNode()
      : rclcpp::Node("gpu_image_transport",
                     rclcpp::NodeOptions().use_intra_process_comms(false)),
        input_topic_(
            declare_parameter<std::string>("input_topic_name", "image_gpu")),
        output_topic_(
            declare_parameter<std::string>("cpu_topic_name", "image")) {
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
        std::bind(&GpuImageTransportNode::on_image, this,
                  std::placeholders::_1),
        subscription_options);

    publisher_ = create_publisher<sensor_msgs::msg::Image>(
        output_topic_, rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(),
                "gpu_image_transport listening on %s, publishing %s",
                input_topic_.c_str(), output_topic_.c_str());
  }

  ~GpuImageTransportNode() override {
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
    NvtxScopedRange callback_range("GpuImageTransportNode::on_image");
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
      NvtxScopedRange wait_range("GpuImageTransportNode::stream_wait_event");
      err = view.enqueue_ready_event(stream_);
    }
    if (err != cudaSuccess) {
      RCLCPP_ERROR(get_logger(), "cudaStreamWaitEvent failed: %s",
                   cuda_error_to_string(err).c_str());
      return;
    }

    const std::size_t bytes_to_copy =
        static_cast<std::size_t>(view.core.byte_size);
    if (bytes_to_copy == 0) {
      return;
    }

    {
      NvtxScopedRange ensure_range("GpuImageTransportNode::ensure_pinned_host");
      const cudaError_t ensure_err = ensure_pinned_capacity(bytes_to_copy);
      if (ensure_err != cudaSuccess) {
        RCLCPP_ERROR(get_logger(), "cudaMallocHost failed: %s",
                     cuda_error_to_string(ensure_err).c_str());
        return;
      }
    }

    {
      NvtxScopedRange memcpy_range("GpuImageTransportNode::cudaMemcpyAsync");
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
          "GpuImageTransportNode::cudaStreamSynchronize");
      err = cudaStreamSynchronize(stream_);
    }
    if (err != cudaSuccess) {
      RCLCPP_ERROR(get_logger(), "cudaStreamSynchronize failed: %s",
                   cuda_error_to_string(err).c_str());
      return;
    }

    publish_cpu_image(view, bytes_to_copy);
  }

  void publish_cpu_image(const ros2_cuda_ipc_core::ImageView &view,
                         std::size_t available_bytes) {
    if (!publisher_) {
      return;
    }

    const std::size_t height = static_cast<std::size_t>(view.rows());
    const std::size_t step_bytes = static_cast<std::size_t>(view.strideH());
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

    publisher_->publish(std::move(msg));
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

  rclcpp::Subscription<ros2_cuda_ipc_core::ImageView>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  cudaStream_t stream_ = nullptr;
  std::string input_topic_;
  std::string output_topic_;
  uint8_t *pinned_host_buffer_ = nullptr;
  std::size_t pinned_host_capacity_ = 0;
};

}  // namespace gpu_image_transport

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<gpu_image_transport::GpuImageTransportNode>();
    rclcpp::spin(node);
  } catch (const std::exception &ex) {
    RCLCPP_FATAL(rclcpp::get_logger("gpu_image_transport"), "Exception: %s",
                 ex.what());
  }
  rclcpp::shutdown();
  return 0;
}
