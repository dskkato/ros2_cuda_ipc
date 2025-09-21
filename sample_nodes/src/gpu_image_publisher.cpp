#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros2_cuda_ipc_core/type_adapters.hpp"
#include "sample_nodes/gpu_image_publisher_helper.hpp"

namespace sample_nodes {
namespace {

ros2_cuda_ipc_core::DType parse_dtype(const std::string &name) {
  if (name == "u8") {
    return ros2_cuda_ipc_core::DType::U8;
  }
  if (name == "u16") {
    return ros2_cuda_ipc_core::DType::U16;
  }
  if (name == "f32") {
    return ros2_cuda_ipc_core::DType::F32;
  }
  if (name == "f64") {
    return ros2_cuda_ipc_core::DType::F64;
  }
  if (name == "s16") {
    return ros2_cuda_ipc_core::DType::S16;
  }
  if (name == "s32") {
    return ros2_cuda_ipc_core::DType::S32;
  }
  if (name == "u32") {
    return ros2_cuda_ipc_core::DType::U32;
  }
  throw std::runtime_error("Unsupported dtype: " + name);
}

}  // namespace

class GpuImagePublisherNode : public rclcpp::Node {
 public:
  GpuImagePublisherNode()
      : rclcpp::Node("gpu_image_publisher",
                     rclcpp::NodeOptions().use_intra_process_comms(false)),
        publish_timer_period_(
            declare_parameter<double>("publish_rate_hz", 30.0)) {
    sample_nodes::GpuImagePublisherHelper::Config config;
    config.width = static_cast<uint32_t>(declare_parameter<int>("width", 640));
    config.height =
        static_cast<uint32_t>(declare_parameter<int>("height", 480));
    config.channels =
        static_cast<uint32_t>(declare_parameter<int>("channels", 3));
    config.slot_count =
        static_cast<std::size_t>(declare_parameter<int>("slot_count", 4));
    config.shm_name =
        declare_parameter<std::string>("shm_name", "/ros2_cuda_ipc_demo");
    config.device_index = declare_parameter<int>("device_index", 0);
    const std::string dtype_str = declare_parameter<std::string>("dtype", "u8");
    config.dtype = parse_dtype(dtype_str);

    frame_id_ = declare_parameter<std::string>("frame_id", "gpu_camera");

    helper_ = std::make_unique<sample_nodes::GpuImagePublisherHelper>(config);

    rclcpp::PublisherOptions options;
    options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    publisher_ = create_publisher<ros2_cuda_ipc_core::ImageView>(
        "gpu_image", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), options);

    if (publish_timer_period_ <= 0.0) {
      publish_timer_period_ = 30.0;
    }
    const auto period =
        std::chrono::duration<double>(1.0 / publish_timer_period_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&GpuImagePublisherNode::on_timer, this));

    RCLCPP_INFO(get_logger(), "Publishing GPU image %ux%u dtype=%s slots=%zu",
                config.width, config.height, dtype_str.c_str(),
                config.slot_count);
  }

 private:
  void on_timer() {
    const uint8_t fill_value = static_cast<uint8_t>(frame_counter_ & 0xFF);
    auto view = helper_->produce(fill_value, frame_id_);
    if (!view.has_value()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Failed to produce GPU frame");
      return;
    }

    view->header.stamp = now();
    view->header.frame_id = frame_id_;
    publisher_->publish(*view);
    ++frame_counter_;
  }

  rclcpp::Publisher<ros2_cuda_ipc_core::ImageView>::SharedPtr publisher_;
  std::unique_ptr<sample_nodes::GpuImagePublisherHelper> helper_;
  rclcpp::TimerBase::SharedPtr timer_;
  double publish_timer_period_ = 30.0;
  uint64_t frame_counter_ = 0;
  std::string frame_id_;
};

}  // namespace sample_nodes

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<sample_nodes::GpuImagePublisherNode>();
    rclcpp::spin(node);
  } catch (const std::exception &ex) {
    RCLCPP_FATAL(rclcpp::get_logger("gpu_image_publisher"), "Exception: %s",
                 ex.what());
  }
  rclcpp::shutdown();
  return 0;
}
