#include <cuda_runtime_api.h>

#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "julia_set/cuda/julia_kernel.hpp"
#include "julia_set/cuda/nvtx_scoped_range.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_cuda_ipc_core/image_view.hpp"
#include "ros2_cuda_ipc_core/type_adapters.hpp"

namespace julia_set {
namespace {

std::string cuda_error_to_string(cudaError_t err) {
  return std::string(cudaGetErrorName(err)) + ": " + cudaGetErrorString(err);
}

using Clock = std::chrono::steady_clock;

bool deadline_reached(const Clock::time_point &deadline,
                      const Clock::time_point &now) {
  return deadline.time_since_epoch().count() != 0 && now >= deadline;
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
    destroy_pool();
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
  struct Slot {
    uint32_t index = 0;
    void *device_ptr = nullptr;
    cudaEvent_t event = nullptr;
    cudaIpcMemHandle_t mem_handle{};
    cudaIpcEventHandle_t event_handle{};
    uint32_t generation = 0;
    Clock::time_point pending_deadline{};
  };

  void on_image(const ros2_cuda_ipc_core::ImageView &view) {
    NvtxScopedRange callback_range("ColorizeNode::on_image");
    if (!view.core.valid()) {
      RCLCPP_WARN(get_logger(), "Received invalid GPU image view");
      return;
    }

    if (!view.sanity_check()) {
      RCLCPP_WARN(
          get_logger(),
          "Skipping GPU frame: failed sanity check rows=%u cols=%u channels=%u",
          view.rows(), view.cols(), view.channels());
      return;
    }

    if (view.channels() != 1) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Expected single-channel input but received %u channels",
          view.channels());
      return;
    }

    const std::size_t subscribers = publisher_->get_subscription_count();

    if (!ensure_pool(view)) {
      return;
    }

    reclaim_stale_pending();

    auto free_slot =
        ros2_cuda_ipc_core::LeaseHandle::choose_empty_slot(output_shm_name_);
    if (!free_slot.has_value()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "No available GPU slots for colorized output");
      return;
    }
    if (free_slot.value() >= slots_.size()) {
      RCLCPP_ERROR(get_logger(), "LeaseHandle returned invalid slot index %u",
                   free_slot.value());
      return;
    }

    Slot &slot = slots_[free_slot.value()];

    auto generation = ros2_cuda_ipc_core::LeaseHandle::bump_generation(
        output_shm_name_, slot.index, static_cast<uint32_t>(subscribers));
    if (!generation.has_value()) {
      RCLCPP_WARN(get_logger(), "Failed to bump generation for slot %u",
                  slot.index);
      return;
    }
    slot.generation = generation.value();

    if (subscribers > 0 && pending_ttl_.count() > 0) {
      slot.pending_deadline = Clock::now() + pending_ttl_;
    } else {
      slot.pending_deadline = {};
    }

    const auto cancel_slot = [&]() {
      slot.pending_deadline = {};
      ros2_cuda_ipc_core::LeaseHandle::force_clear_pending(output_shm_name_,
                                                           slot.index);
    };

    cudaError_t err = cudaSetDevice(device_index_);
    if (err != cudaSuccess) {
      RCLCPP_ERROR(get_logger(), "cudaSetDevice failed: %s",
                   cudaGetErrorString(err));
      cancel_slot();
      return;
    }

    {
      NvtxScopedRange wait_range("ColorizeNode::stream_wait_input");
      err = view.enqueue_ready_event(stream_);
    }
    if (err != cudaSuccess) {
      RCLCPP_ERROR(get_logger(), "cudaStreamWaitEvent failed: %s",
                   cudaGetErrorString(err));
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
      RCLCPP_ERROR(get_logger(),
                   "launch_colorize_kernel failed for slot %u: %s", slot.index,
                   cuda_error_to_string(err).c_str());
      cancel_slot();
      return;
    }

    {
      NvtxScopedRange record_range("ColorizeNode::cudaEventRecord");
      err = cudaEventRecord(slot.event, stream_);
    }
    if (err != cudaSuccess) {
      RCLCPP_ERROR(get_logger(), "cudaEventRecord failed: %s",
                   cudaGetErrorString(err));
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

    if (pool_initialised_ && width == width_ && height == height_ &&
        device_id == device_index_) {
      return true;
    }

    destroy_pool();

    cudaError_t err = cudaSetDevice(device_id);
    if (err != cudaSuccess) {
      RCLCPP_ERROR(get_logger(), "cudaSetDevice failed: %s",
                   cudaGetErrorString(err));
      return false;
    }

    if (!ros2_cuda_ipc_core::LeaseHandle::init(
            output_shm_name_, static_cast<uint32_t>(slot_count_))) {
      RCLCPP_ERROR(get_logger(), "Failed to initialise lease shared memory %s",
                   output_shm_name_.c_str());
      return false;
    }

    width_ = width;
    height_ = height;
    device_index_ = device_id;
    frame_size_bytes_ =
        static_cast<uint64_t>(width_) * height_ * output_channels_;

    slots_.resize(slot_count_);
    for (std::size_t i = 0; i < slots_.size(); ++i) {
      auto &slot = slots_[i];
      slot.index = static_cast<uint32_t>(i);
      slot.pending_deadline = {};

      err = cudaMalloc(&slot.device_ptr, frame_size_bytes_);
      if (err != cudaSuccess) {
        RCLCPP_ERROR(get_logger(), "cudaMalloc failed: %s",
                     cudaGetErrorString(err));
        destroy_pool();
        return false;
      }

      err = cudaEventCreateWithFlags(
          &slot.event, cudaEventDisableTiming | cudaEventInterprocess);
      if (err != cudaSuccess) {
        RCLCPP_ERROR(get_logger(), "cudaEventCreateWithFlags failed: %s",
                     cudaGetErrorString(err));
        destroy_pool();
        return false;
      }

      err = cudaIpcGetMemHandle(&slot.mem_handle, slot.device_ptr);
      if (err != cudaSuccess) {
        RCLCPP_ERROR(get_logger(), "cudaIpcGetMemHandle failed: %s",
                     cudaGetErrorString(err));
        destroy_pool();
        return false;
      }

      err = cudaIpcGetEventHandle(&slot.event_handle, slot.event);
      if (err != cudaSuccess) {
        RCLCPP_ERROR(get_logger(), "cudaIpcGetEventHandle failed: %s",
                     cudaGetErrorString(err));
        destroy_pool();
        return false;
      }
    }

    pool_initialised_ = true;
    RCLCPP_INFO(
        get_logger(),
        "Initialized colorize pool %ux%u channels=%u slots=%zu device=%d",
        width_, height_, output_channels_, slots_.size(), device_index_);
    return true;
  }

  void destroy_pool() noexcept {
    if (device_index_ >= 0) {
      cudaSetDevice(device_index_);
    }

    for (auto &slot : slots_) {
      if (slot.event) {
        cudaEventDestroy(slot.event);
        slot.event = nullptr;
      }
      if (slot.device_ptr) {
        cudaFree(slot.device_ptr);
        slot.device_ptr = nullptr;
      }
      slot.pending_deadline = {};
    }
    slots_.clear();
    frame_size_bytes_ = 0;
    width_ = 0;
    height_ = 0;
    device_index_ = -1;
    pool_initialised_ = false;
  }

  void reclaim_stale_pending() {
    if (!pool_initialised_ || pending_ttl_.count() <= 0) {
      return;
    }

    const auto now = Clock::now();
    for (auto &slot : slots_) {
      if (!deadline_reached(slot.pending_deadline, now)) {
        continue;
      }

      auto pending = ros2_cuda_ipc_core::LeaseHandle::current_pending(
          output_shm_name_, slot.index);
      if (!pending.has_value() || pending.value() == 0) {
        slot.pending_deadline = {};
        continue;
      }

      auto refcnt = ros2_cuda_ipc_core::LeaseHandle::current_refcount(
          output_shm_name_, slot.index);
      if (!refcnt.has_value() || refcnt.value() != 0) {
        continue;
      }

      if (ros2_cuda_ipc_core::LeaseHandle::force_clear_pending(output_shm_name_,
                                                               slot.index)) {
        RCLCPP_WARN(get_logger(),
                    "Force-cleared pending lease slot=%u after %lld ms timeout",
                    slot.index, static_cast<long long>(pending_ttl_.count()));
        slot.pending_deadline = {};
      }
    }
  }

  ros2_cuda_ipc_core::ImageView make_output_view(
      const Slot &slot, const ros2_cuda_ipc_core::ImageView &input) const {
    ros2_cuda_ipc_core::ImageView view;
    view.header = input.header;
    view.core.dev_ptr = slot.device_ptr;
    view.core.ready_evt = slot.event;
    view.core.device_id = device_index_;
    view.core.byte_size = frame_size_bytes_;
    view.core.slot_id = slot.index;
    view.core.generation = slot.generation;
    view.core.shm_name = output_shm_name_;
    view.core.set_ipc_handles(slot.mem_handle, slot.event_handle);
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
  std::vector<Slot> slots_;
  uint64_t frame_size_bytes_ = 0;
  uint32_t width_ = 0;
  uint32_t height_ = 0;
  int device_index_ = -1;
  bool pool_initialised_ = false;
  std::string input_topic_;
  std::string output_topic_;
  std::string output_shm_name_;
  uint32_t output_channels_ = 3;
  std::size_t slot_count_ = 0;
  std::chrono::milliseconds pending_ttl_{300};
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
