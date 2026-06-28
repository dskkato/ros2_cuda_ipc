#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros2_cuda_ipc_core/type_adapters.hpp"
#include "sample_nodes/gpu_pointcloud_publisher_helper.hpp"

namespace sample_nodes {
class GpuPointCloudPublisherNode : public rclcpp::Node {
 public:
  GpuPointCloudPublisherNode()
      : rclcpp::Node("gpu_pointcloud_publisher",
                     rclcpp::NodeOptions().use_intra_process_comms(false)) {
    sample_nodes::GpuPointCloudPublisherHelper::Config config;
    config.width = static_cast<uint32_t>(declare_parameter<int>("width", 1024));
    config.height = static_cast<uint32_t>(declare_parameter<int>("height", 1));
    config.slot_count =
        static_cast<std::size_t>(declare_parameter<int>("slot_count", 4));
    config.shm_name =
        declare_parameter<std::string>("shm_name", "/ros2_cuda_ipc_demo_pc");
    config.device_index = declare_parameter<int>("device_index", 0);
    config.is_dense = declare_parameter<bool>("is_dense", true);
    const int pending_ttl_ms = declare_parameter<int>(
        "pending_ttl_ms", static_cast<int>(config.pending_ttl.count()));
    config.pending_ttl = std::chrono::milliseconds{pending_ttl_ms};

    frame_id_ = declare_parameter<std::string>("frame_id", "gpu_lidar");
    fill_value_base_ =
        static_cast<float>(declare_parameter<double>("fill_value", 1.0));
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);

    helper_ =
        std::make_unique<sample_nodes::GpuPointCloudPublisherHelper>(config);

    rclcpp::PublisherOptions options;
    options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    publisher_ = create_publisher<ros2_cuda_ipc_core::PointCloud2View>(
        "gpu_pointcloud", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        options);

    if (publish_rate_hz_ <= 0.0) {
      publish_rate_hz_ = 10.0;
    }
    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&GpuPointCloudPublisherNode::on_timer, this));

    RCLCPP_INFO(get_logger(), "Publishing GPU point cloud %ux%u slots=%zu",
                config.width, config.height, config.slot_count);
  }

 private:
  void on_timer() {
    const float value =
        fill_value_base_ + static_cast<float>(frame_counter_ % 256) * 0.01F;
    auto view = helper_->produce(publisher_->get_subscription_count(), value);
    if (!view.has_value()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Failed to produce GPU point cloud");
      return;
    }

    view->header.stamp = now();
    view->header.frame_id = frame_id_;
    publisher_->publish(*view);
    ++frame_counter_;
  }

  rclcpp::Publisher<ros2_cuda_ipc_core::PointCloud2View>::SharedPtr publisher_;
  std::unique_ptr<sample_nodes::GpuPointCloudPublisherHelper> helper_;
  rclcpp::TimerBase::SharedPtr timer_;
  double publish_rate_hz_ = 10.0;
  uint64_t frame_counter_ = 0;
  std::string frame_id_;
  float fill_value_base_ = 1.0F;
};

}  // namespace sample_nodes

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<sample_nodes::GpuPointCloudPublisherNode>();
    rclcpp::spin(node);
  } catch (const std::exception& ex) {
    RCLCPP_FATAL(rclcpp::get_logger("gpu_pointcloud_publisher"),
                 "Exception: %s", ex.what());
  }
  rclcpp::shutdown();
  return 0;
}
