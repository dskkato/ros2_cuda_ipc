#include <cuda_runtime_api.h>

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ros2_cuda_ipc_core/pointcloud2_view.hpp"
#include "ros2_cuda_ipc_core/type_adapters.hpp"

namespace sample_nodes {

class GpuPointCloudSubscriberNode : public rclcpp::Node {
 public:
  GpuPointCloudSubscriberNode()
      : rclcpp::Node("gpu_pointcloud_subscriber",
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
    subscription_ = create_subscription<ros2_cuda_ipc_core::GpuPointCloud2View>(
        "gpu_pointcloud", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        std::bind(&GpuPointCloudSubscriberNode::on_pointcloud, this,
                  std::placeholders::_1),
        options);

    RCLCPP_INFO(get_logger(),
                "Listening for GPU point clouds on /gpu_pointcloud");
  }

  ~GpuPointCloudSubscriberNode() override {
    if (stream_) {
      cudaStreamDestroy(stream_);
      stream_ = nullptr;
    }
  }

 private:
  void on_pointcloud(const ros2_cuda_ipc_core::GpuPointCloud2View &msg) {
    const auto &view = msg.pointcloud;
    if (!view.core.valid()) {
      RCLCPP_WARN(get_logger(), "Received invalid GPU point cloud view");
      return;
    }

    auto err = view.wait(stream_);
    if (err != cudaSuccess) {
      RCLCPP_ERROR(get_logger(), "cudaStreamWaitEvent failed: %s",
                   cudaGetErrorString(err));
      return;
    }

    const std::size_t sample_bytes =
        std::min<std::size_t>(view.core.byte_size, sizeof(float) * 9);
    std::vector<float> host(sample_bytes / sizeof(float), 0.0f);
    err = cudaMemcpyAsync(host.data(), view.core.data<float>(), sample_bytes,
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

    float x = host.size() > 0 ? host[0] : 0.0F;
    float y = host.size() > 1 ? host[1] : 0.0F;
    float z = host.size() > 2 ? host[2] : 0.0F;
    RCLCPP_INFO(get_logger(),
                "PointCloud frame %zu received frame_id=%s first_point=(%.3f, "
                "%.3f, %.3f)",
                ++frame_counter_, msg.header.frame_id.c_str(), x, y, z);
  }

  rclcpp::Subscription<ros2_cuda_ipc_core::GpuPointCloud2View>::SharedPtr
      subscription_;
  cudaStream_t stream_ = nullptr;
  std::size_t frame_counter_ = 0;
};

}  // namespace sample_nodes

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<sample_nodes::GpuPointCloudSubscriberNode>();
    rclcpp::spin(node);
  } catch (const std::exception &ex) {
    RCLCPP_FATAL(rclcpp::get_logger("gpu_pointcloud_subscriber"),
                 "Exception: %s", ex.what());
  }
  rclcpp::shutdown();
  return 0;
}
