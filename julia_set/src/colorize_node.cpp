#include <cuda_runtime_api.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "julia_set/cuda/julia_kernel.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_cuda_ipc_core/cuda/cuda_util.hpp"
#include "ros2_cuda_ipc_core/cuda/gpu_lease_pool.hpp"
#include "ros2_cuda_ipc_core/image_view.hpp"
#include "ros2_cuda_ipc_core/memory_types.hpp"
#include "ros2_cuda_ipc_core/nvtx_scoped_range.hpp"
#include "ros2_cuda_ipc_core/type_adapters.hpp"

namespace julia_set {

using ros2_cuda_ipc_core::NvtxScopedRange;
using ros2_cuda_ipc_core::cuda::cuda_error_to_string;
using ros2_cuda_ipc_core::cuda::GpuLeasePool;

namespace {

ros2_cuda_ipc_core::MemoryBackendKind parse_memory_backend(
    std::string name, const rclcpp::Logger &logger) {
  std::transform(name.begin(), name.end(), name.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  if (name == "cuda" || name == "cuda_ipc" || name == "ipc") {
    return ros2_cuda_ipc_core::MemoryBackendKind::CUDA_IPC;
  }
  if (name == "vmm_fd" || name == "vmm-fd" || name == "vmm" || name == "fd") {
    return ros2_cuda_ipc_core::MemoryBackendKind::VMM_FD;
  }

  RCLCPP_WARN(logger, "Unknown memory_backend='%s'; defaulting to CUDA IPC",
              name.c_str());
  return ros2_cuda_ipc_core::MemoryBackendKind::CUDA_IPC;
}

}  // namespace

class ColorizeNode : public rclcpp::Node {
 public:
  ColorizeNode()
      : rclcpp::Node("colorize_node",
                     rclcpp::NodeOptions().use_intra_process_comms(false)),
        input_topic_(declare_parameter<std::string>("input_topic_name",
                                                    "julia_set/image")),
        output_topic_(declare_parameter<std::string>("output_topic_name",
                                                     "julia_set/colorized")),
        output_shm_name_(declare_parameter<std::string>(
            "output_shm_name", "/ros2_cuda_ipc_julia_colorized")),
        output_channels_(static_cast<uint32_t>(
            declare_parameter<int>("output_channels", 3))),
        slot_count_(static_cast<std::size_t>(
            declare_parameter<int>("output_slot_count", 4))),
        pending_ttl_(std::chrono::milliseconds(
            declare_parameter<int>("output_pending_ttl_ms", 300))),
        backend_(parse_memory_backend(
            declare_parameter<std::string>("memory_backend", "cuda_ipc"),
            get_logger())),
        pool_({output_shm_name_, slot_count_, pending_ttl_, backend_},
              get_logger()),
        output_encoding_(
            declare_parameter<std::string>("output_encoding", "rgb8")) {
    if (output_channels_ < 3) {
      RCLCPP_WARN(get_logger(),
                  "output_channels=%u is not supported; defaulting to 3",
                  output_channels_);
      output_channels_ = 3;
    }
    if (slot_count_ == 0) {
      RCLCPP_WARN(get_logger(),
                  "output_slot_count was zero; defaulting to 1 slot");
      slot_count_ = 1;
      pool_ =
          GpuLeasePool({output_shm_name_, slot_count_, pending_ttl_, backend_},
                       get_logger());
    }

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
        input_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        std::bind(&ColorizeNode::on_image, this, std::placeholders::_1),
        subscription_options);

    rclcpp::PublisherOptions publisher_options;
    publisher_options.use_intra_process_comm =
        rclcpp::IntraProcessSetting::Disable;
    publisher_ = create_publisher<ros2_cuda_ipc_core::ImageView>(
        output_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        publisher_options);

    RCLCPP_INFO(
        get_logger(), "colorize_node listening on %s, publishing %s via shm=%s",
        input_topic_.c_str(), output_topic_.c_str(), output_shm_name_.c_str());
  }

  ~ColorizeNode() override {
    auto logger = get_logger();
    pool_.reset();
    width_ = 0;
    height_ = 0;
    frame_size_bytes_ = 0;
    device_index_ = -1;
    if (stream_) {
      const cudaError_t err = cudaStreamDestroy(stream_);
      if (err != cudaSuccess) {
        RCLCPP_ERROR(logger, "cudaStreamDestroy failed: %s",
                     cuda_error_to_string(err).c_str());
      }
      stream_ = nullptr;
    }
  }

 private:
  void on_image(const ros2_cuda_ipc_core::ImageView &view) {
    NvtxScopedRange callback_range("ColorizeNode::on_image");
    auto logger = get_logger();
    if (!view.core.valid()) {
      RCLCPP_WARN(logger, "Received invalid GPU image view");
      return;
    }

    if (!view.sanity_check()) {
      RCLCPP_WARN(
          logger,
          "Skipping GPU frame: failed sanity check rows=%u cols=%u channels=%u",
          view.rows(), view.cols(), view.channels());
      return;
    }

    if (view.channels() != 1) {
      RCLCPP_WARN_THROTTLE(
          logger, *get_clock(), 2000,
          "Expected single-channel input but received %u channels",
          view.channels());
      return;
    }

    const std::size_t subscribers = publisher_->get_subscription_count();

    if (!ensure_pool(view)) {
      return;
    }

    pool_.reclaim_stale_pending();

    auto slot_opt = pool_.acquire(subscribers);
    if (!slot_opt.has_value() || *slot_opt == nullptr) {
      RCLCPP_WARN_THROTTLE(logger, *get_clock(), 2000,
                           "No available GPU slots for colorized output");
      return;
    }
    auto &slot = **slot_opt;

    const auto cancel_slot = [&]() { pool_.cancel_pending(slot); };

    cudaError_t err = cudaSetDevice(device_index_);
    if (err != cudaSuccess) {
      RCLCPP_ERROR(logger, "cudaSetDevice failed: %s",
                   cuda_error_to_string(err).c_str());
      cancel_slot();
      return;
    }

    {
      NvtxScopedRange wait_range("ColorizeNode::stream_wait_input");
      err = view.enqueue_ready_event(stream_);
    }
    if (err != cudaSuccess) {
      RCLCPP_ERROR(logger, "cudaStreamWaitEvent failed: %s",
                   cuda_error_to_string(err).c_str());
      cancel_slot();
      return;
    }

    {
      NvtxScopedRange launch_range("ColorizeNode::launch_colorize_kernel");
      err = launch_colorize_kernel(view.core.data<uint8_t>(),
                                   static_cast<uint8_t *>(slot.device_ptr),
                                   width_, height_, output_channels_, stream_);
    }
    if (err != cudaSuccess) {
      RCLCPP_ERROR(logger, "launch_colorize_kernel failed for slot %u: %s",
                   slot.index, cuda_error_to_string(err).c_str());
      cancel_slot();
      return;
    }

    {
      NvtxScopedRange record_range("ColorizeNode::cudaEventRecord");
      err = cudaEventRecord(slot.event, stream_);
    }
    if (err != cudaSuccess) {
      RCLCPP_ERROR(logger, "cudaEventRecord failed: %s",
                   cuda_error_to_string(err).c_str());
      cancel_slot();
      return;
    }

    ros2_cuda_ipc_core::ImageView output = make_output_view(slot, view);
    publisher_->publish(output);
  }

  bool ensure_pool(const ros2_cuda_ipc_core::ImageView &view) {
    const uint32_t width = view.cols();
    const uint32_t height = view.rows();
    const int device_id = view.core.device_id;

    if (pool_.is_initialised() && width == width_ && height == height_ &&
        device_id == device_index_) {
      return true;
    }

    auto logger = get_logger();
    pool_.reset();

    cudaError_t err = cudaSetDevice(device_id);
    if (err != cudaSuccess) {
      RCLCPP_ERROR(logger, "cudaSetDevice failed: %s",
                   cuda_error_to_string(err).c_str());
      width_ = 0;
      height_ = 0;
      device_index_ = -1;
      frame_size_bytes_ = 0;
      return false;
    }

    width_ = width;
    height_ = height;
    device_index_ = device_id;
    frame_size_bytes_ =
        static_cast<uint64_t>(width_) * height_ * output_channels_;

    if (!pool_.initialise(frame_size_bytes_, device_index_)) {
      width_ = 0;
      height_ = 0;
      device_index_ = -1;
      frame_size_bytes_ = 0;
      return false;
    }

    RCLCPP_INFO(
        logger,
        "Initialized colorize pool %ux%u channels=%u slots=%zu device=%d",
        width_, height_, output_channels_, slot_count_, device_index_);
    return true;
  }

  ros2_cuda_ipc_core::ImageView make_output_view(
      const GpuLeasePool::Slot &slot,
      const ros2_cuda_ipc_core::ImageView &input) const {
    ros2_cuda_ipc_core::ImageView view;
    view.header = input.header;
    view.core.dev_ptr = slot.device_ptr;
    view.core.ready_evt = slot.event;
    view.core.device_id = device_index_;
    view.core.byte_size = frame_size_bytes_;
    view.core.slot_id = slot.index;
    view.core.generation = slot.generation;
    view.core.shm_name = output_shm_name_;
    view.core.set_memory_handles(slot.backend, slot.mem_handle.data(),
                                 slot.mem_handle.size(), slot.event_handle);
    view.dtype = ros2_cuda_ipc_core::DType::U8;
    view.shape = {height_, width_, output_channels_};
    view.strides = {static_cast<uint64_t>(width_) * output_channels_,
                    static_cast<uint64_t>(output_channels_), 1};
    view.encoding = output_encoding_;
    return view;
  }

  rclcpp::Subscription<ros2_cuda_ipc_core::ImageView>::SharedPtr subscription_;
  rclcpp::Publisher<ros2_cuda_ipc_core::ImageView>::SharedPtr publisher_;
  cudaStream_t stream_ = nullptr;
  uint64_t frame_size_bytes_ = 0;
  uint32_t width_ = 0;
  uint32_t height_ = 0;
  int device_index_ = -1;
  std::string input_topic_;
  std::string output_topic_;
  std::string output_shm_name_;
  uint32_t output_channels_ = 3;
  std::size_t slot_count_ = 0;
  std::chrono::milliseconds pending_ttl_{300};
  ros2_cuda_ipc_core::MemoryBackendKind backend_ =
      ros2_cuda_ipc_core::MemoryBackendKind::CUDA_IPC;
  GpuLeasePool pool_;
  std::string output_encoding_;
};

}  // namespace julia_set

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<julia_set::ColorizeNode>();
    rclcpp::spin(node);
  } catch (const std::exception &ex) {
    RCLCPP_FATAL(rclcpp::get_logger("colorize_node"), "Exception: %s",
                 ex.what());
  }
  rclcpp::shutdown();
  return 0;
}
