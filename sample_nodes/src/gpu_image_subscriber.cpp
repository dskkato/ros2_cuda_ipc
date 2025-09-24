#include <cuda_runtime_api.h>

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ros2_cuda_ipc_core/image_view.hpp"
#include "ros2_cuda_ipc_core/type_adapters.hpp"

namespace sample_nodes {

class GpuImageSubscriberNode : public rclcpp::Node {
 public:
  GpuImageSubscriberNode()
      : rclcpp::Node("gpu_image_subscriber",
                     rclcpp::NodeOptions().use_intra_process_comms(false)) {
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
        "gpu_image", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        std::bind(&GpuImageSubscriberNode::on_image, this,
                  std::placeholders::_1),
        options);

    RCLCPP_INFO(get_logger(), "Listening for GPU images on /gpu_image");
  }

  ~GpuImageSubscriberNode() override {
    if (stream_) {
      cudaStreamDestroy(stream_);
      stream_ = nullptr;
    }
  }

 private:
  void on_image(const ros2_cuda_ipc_core::ImageView &view) {
    if (!view.core.valid()) {
      RCLCPP_WARN(get_logger(), "Received invalid GPU image view");
      return;
    }

    auto err = view.wait(stream_);
    if (err != cudaSuccess) {
      RCLCPP_ERROR(get_logger(), "cudaStreamWaitEvent failed: %s",
                   cudaGetErrorString(err));
      return;
    }

    const std::size_t sample_size =
        std::min<std::size_t>(view.core.byte_size, 32);
    std::vector<uint8_t> host(sample_size, 0);
    err = cudaMemcpyAsync(host.data(), view.core.data<uint8_t>(), sample_size,
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

    RCLCPP_INFO(get_logger(),
                "Frame %zu received frame_id=%s encoding=%s first_byte=%u",
                ++frame_counter_, view.header.frame_id.c_str(),
                view.encoding.c_str(), static_cast<unsigned>(host[0]));
  }

  rclcpp::Subscription<ros2_cuda_ipc_core::ImageView>::SharedPtr subscription_;
  cudaStream_t stream_ = nullptr;
  std::size_t frame_counter_ = 0;
};

}  // namespace sample_nodes

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<sample_nodes::GpuImageSubscriberNode>();
    rclcpp::spin(node);
  } catch (const std::exception &ex) {
    RCLCPP_FATAL(rclcpp::get_logger("gpu_image_subscriber"), "Exception: %s",
                 ex.what());
  }
  rclcpp::shutdown();
  return 0;
}
