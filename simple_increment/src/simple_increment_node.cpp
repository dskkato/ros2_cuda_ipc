// Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
// Copyright 2025 dskkato
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cuda_runtime_api.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "ros2_cuda_ipc_core/image_view.hpp"
#include "ros2_cuda_ipc_core/lease_handle.hpp"
#include "ros2_cuda_ipc_core/type_adapters.hpp"
#include "simple_increment/cuda_functions.hpp"

namespace simple_increment {
namespace detail {

uint32_t dtype_bytes(ros2_cuda_ipc_core::DType dtype) {
  switch (dtype) {
    case ros2_cuda_ipc_core::DType::U8:
      return 1;
    case ros2_cuda_ipc_core::DType::U16:
    case ros2_cuda_ipc_core::DType::F16:
    case ros2_cuda_ipc_core::DType::S16:
      return 2;
    case ros2_cuda_ipc_core::DType::F32:
    case ros2_cuda_ipc_core::DType::S32:
    case ros2_cuda_ipc_core::DType::U32:
      return 4;
    case ros2_cuda_ipc_core::DType::F64:
      return 8;
  }
  return 1;
}

std::string cuda_error_to_string(cudaError_t err) {
  return std::string(cudaGetErrorName(err)) + ": " + cudaGetErrorString(err);
}

struct ImageLayout {
  uint32_t height = 0;
  uint32_t width = 0;
  uint32_t channels = 0;
  ros2_cuda_ipc_core::DType dtype = ros2_cuda_ipc_core::DType::U8;
  std::string encoding;

  bool matches(const ros2_cuda_ipc_core::ImageView &view) const {
    return height == view.shape[0] && width == view.shape[1] &&
           channels == view.shape[2] && dtype == view.dtype &&
           encoding == view.encoding;
  }

  static ImageLayout from_view(const ros2_cuda_ipc_core::ImageView &view) {
    ImageLayout layout;
    layout.height = view.shape[0];
    layout.width = view.shape[1];
    layout.channels = view.shape[2];
    layout.dtype = view.dtype;
    layout.encoding = view.encoding;
    return layout;
  }
};

class ImageSlotPool {
 public:
  struct Config {
    std::string shm_name = "/ros2_cuda_ipc_simple_increment";
    std::size_t slot_count = 4;
    int device_index = 0;
    std::chrono::milliseconds pending_ttl{300};
  };

  struct Reservation {
    ros2_cuda_ipc_core::ImageView view;
    uint8_t *device_ptr = nullptr;
    uint64_t byte_size = 0;
    uint32_t slot_index = 0;
    bool finalised = false;
  };

  explicit ImageSlotPool(const Config &config) : config_(config) {}
  ~ImageSlotPool() { destroy_slots(); }

  ImageSlotPool(const ImageSlotPool &) = delete;
  ImageSlotPool &operator=(const ImageSlotPool &) = delete;

  bool configure(const ImageLayout &layout) {
    if (config_.slot_count == 0) {
      RCLCPP_ERROR(rclcpp::get_logger("simple_increment.ImageSlotPool"),
                   "slot_count must be greater than zero");
      return false;
    }

    if (layout.height == 0 || layout.width == 0 || layout.channels == 0) {
      RCLCPP_ERROR(rclcpp::get_logger("simple_increment.ImageSlotPool"),
                   "Invalid layout: height=%u width=%u channels=%u",
                   layout.height, layout.width, layout.channels);
      return false;
    }

    const auto err = cudaSetDevice(config_.device_index);
    if (err != cudaSuccess) {
      RCLCPP_ERROR(rclcpp::get_logger("simple_increment.ImageSlotPool"),
                   "cudaSetDevice(%d) failed: %s", config_.device_index,
                   cuda_error_to_string(err).c_str());
      return false;
    }

    destroy_slots();

    layout_ = layout;
    frame_size_bytes_ = static_cast<uint64_t>(layout.height) * layout.width *
                        layout.channels * dtype_bytes(layout.dtype);

    if (!ros2_cuda_ipc_core::LeaseHandle::init(
            config_.shm_name, static_cast<uint32_t>(config_.slot_count))) {
      RCLCPP_ERROR(rclcpp::get_logger("simple_increment.ImageSlotPool"),
                   "LeaseHandle::init failed for shm %s",
                   config_.shm_name.c_str());
      layout_ = {};
      frame_size_bytes_ = 0;
      return false;
    }

    slots_.assign(config_.slot_count, Slot{});

    for (std::size_t i = 0; i < slots_.size(); ++i) {
      auto &slot = slots_[i];
      slot.index = static_cast<uint32_t>(i);

      cudaError_t alloc_err = cudaMalloc(&slot.device_ptr, frame_size_bytes_);
      if (alloc_err != cudaSuccess) {
        RCLCPP_ERROR(rclcpp::get_logger("simple_increment.ImageSlotPool"),
                     "cudaMalloc failed for slot %zu: %s", i,
                     cuda_error_to_string(alloc_err).c_str());
        destroy_slots();
        return false;
      }

      alloc_err = cudaEventCreateWithFlags(
          &slot.event, cudaEventDisableTiming | cudaEventInterprocess);
      if (alloc_err != cudaSuccess) {
        RCLCPP_ERROR(rclcpp::get_logger("simple_increment.ImageSlotPool"),
                     "cudaEventCreateWithFlags failed for slot %zu: %s", i,
                     cuda_error_to_string(alloc_err).c_str());
        destroy_slots();
        return false;
      }

      alloc_err = cudaIpcGetMemHandle(&slot.mem_handle, slot.device_ptr);
      if (alloc_err != cudaSuccess) {
        RCLCPP_ERROR(rclcpp::get_logger("simple_increment.ImageSlotPool"),
                     "cudaIpcGetMemHandle failed for slot %zu: %s", i,
                     cuda_error_to_string(alloc_err).c_str());
        destroy_slots();
        return false;
      }

      alloc_err = cudaIpcGetEventHandle(&slot.event_handle, slot.event);
      if (alloc_err != cudaSuccess) {
        RCLCPP_ERROR(rclcpp::get_logger("simple_increment.ImageSlotPool"),
                     "cudaIpcGetEventHandle failed for slot %zu: %s", i,
                     cuda_error_to_string(alloc_err).c_str());
        destroy_slots();
        return false;
      }
    }

    configured_ = true;
    return true;
  }

  std::optional<Reservation> reserve(std::size_t pending_consumers) {
    if (!configured_) {
      return std::nullopt;
    }

    reclaim_stale_pending();

    auto slot_index =
        ros2_cuda_ipc_core::LeaseHandle::choose_empty_slot(config_.shm_name);
    if (!slot_index.has_value()) {
      RCLCPP_WARN(rclcpp::get_logger("simple_increment.ImageSlotPool"),
                  "No available slots in shm %s", config_.shm_name.c_str());
      return std::nullopt;
    }
    if (slot_index.value() >= slots_.size()) {
      RCLCPP_ERROR(rclcpp::get_logger("simple_increment.ImageSlotPool"),
                   "LeaseHandle returned invalid slot index %u",
                   slot_index.value());
      return std::nullopt;
    }

    auto &slot = slots_[slot_index.value()];
    auto generation = ros2_cuda_ipc_core::LeaseHandle::bump_generation(
        config_.shm_name, slot.index, static_cast<uint32_t>(pending_consumers));
    if (!generation.has_value()) {
      RCLCPP_WARN(rclcpp::get_logger("simple_increment.ImageSlotPool"),
                  "Failed to bump generation for slot %u", slot.index);
      return std::nullopt;
    }
    slot.generation = generation.value();

    const auto now = std::chrono::steady_clock::now();
    if (pending_consumers > 0 && config_.pending_ttl.count() > 0) {
      slot.pending_deadline = now + config_.pending_ttl;
    } else {
      slot.pending_deadline = {};
    }

    Reservation reservation;
    reservation.device_ptr = static_cast<uint8_t *>(slot.device_ptr);
    reservation.byte_size = frame_size_bytes_;
    reservation.slot_index = slot.index;
    reservation.view.core.dev_ptr = slot.device_ptr;
    reservation.view.core.ready_evt = slot.event;
    reservation.view.core.device_id = config_.device_index;
    reservation.view.core.byte_size = frame_size_bytes_;
    reservation.view.core.slot_id = slot.index;
    reservation.view.core.generation = slot.generation;
    reservation.view.core.shm_name = config_.shm_name;
    reservation.view.core.set_ipc_handles(slot.mem_handle, slot.event_handle);
    reservation.view.dtype = layout_.dtype;
    reservation.view.shape = {layout_.height, layout_.width, layout_.channels};
    const uint64_t elem = dtype_bytes(layout_.dtype);
    reservation.view.strides = {
        static_cast<uint64_t>(layout_.width) * layout_.channels * elem,
        static_cast<uint64_t>(layout_.channels) * elem, elem};
    reservation.view.encoding = layout_.encoding;
    return reservation;
  }

  bool finalize(Reservation &reservation, cudaStream_t stream) {
    if (!configured_) {
      return false;
    }
    if (reservation.slot_index >= slots_.size()) {
      return false;
    }
    auto &slot = slots_[reservation.slot_index];
    const auto err = cudaEventRecord(slot.event, stream);
    if (err != cudaSuccess) {
      RCLCPP_ERROR(rclcpp::get_logger("simple_increment.ImageSlotPool"),
                   "cudaEventRecord failed for slot %u: %s", slot.index,
                   cuda_error_to_string(err).c_str());
    } else {
      reservation.finalised = true;
    }
    return reservation.finalised;
  }

  void cancel(const Reservation &reservation) {
    if (!configured_) {
      return;
    }
    auto logger = rclcpp::get_logger("simple_increment.ImageSlotPool");
    if (!ros2_cuda_ipc_core::LeaseHandle::force_clear_pending(
            config_.shm_name, reservation.slot_index)) {
      RCLCPP_WARN(
          logger,
          "Failed to force clear pending for slot %u after cancellation",
          reservation.slot_index);
    }
  }

  uint64_t frame_size_bytes() const noexcept { return frame_size_bytes_; }

 private:
  struct Slot {
    uint32_t index = 0;
    void *device_ptr = nullptr;
    cudaEvent_t event = nullptr;
    cudaIpcMemHandle_t mem_handle{};
    cudaIpcEventHandle_t event_handle{};
    uint32_t generation = 0;
    std::chrono::steady_clock::time_point pending_deadline{};
  };

  void destroy_slots() noexcept {
    if (slots_.empty()) {
      layout_ = {};
      frame_size_bytes_ = 0;
      configured_ = false;
      return;
    }

    const auto err = cudaSetDevice(config_.device_index);
    if (err != cudaSuccess) {
      RCLCPP_ERROR(rclcpp::get_logger("simple_increment.ImageSlotPool"),
                   "cudaSetDevice(%d) failed during destroy: %s",
                   config_.device_index, cuda_error_to_string(err).c_str());
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
    layout_ = {};
    frame_size_bytes_ = 0;
    configured_ = false;
  }

  void reclaim_stale_pending() {
    if (config_.pending_ttl.count() <= 0) {
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    auto logger = rclcpp::get_logger("simple_increment.ImageSlotPool");

    for (auto &slot : slots_) {
      if (slot.pending_deadline.time_since_epoch().count() == 0 ||
          now < slot.pending_deadline) {
        continue;
      }

      auto pending = ros2_cuda_ipc_core::LeaseHandle::current_pending(
          config_.shm_name, slot.index);
      if (!pending.has_value()) {
        continue;
      }
      if (pending.value() == 0) {
        slot.pending_deadline = {};
        continue;
      }

      auto refcnt = ros2_cuda_ipc_core::LeaseHandle::current_refcount(
          config_.shm_name, slot.index);
      if (!refcnt.has_value() || refcnt.value() != 0) {
        continue;
      }

      if (ros2_cuda_ipc_core::LeaseHandle::force_clear_pending(config_.shm_name,
                                                               slot.index)) {
        RCLCPP_WARN(logger,
                    "Force-cleared pending slot=%u after TTL expiration",
                    slot.index);
        slot.pending_deadline = {};
      }
    }
  }

  Config config_;
  ImageLayout layout_{};
  std::vector<Slot> slots_;
  uint64_t frame_size_bytes_ = 0;
  bool configured_ = false;
};

}  // namespace detail

class SimpleIncrementNode : public rclcpp::Node {
 public:
  explicit SimpleIncrementNode(const rclcpp::NodeOptions &options)
      : rclcpp::Node("simple_increment", options) {
    proc_count_ = declare_parameter<int>("proc_count", 1);
    inplace_enabled_ = declare_parameter<bool>("inplace_enabled", false);
    type_adaptation_enabled_ =
        declare_parameter<bool>("type_adaptation_enabled", true);
    input_topic_ = declare_parameter<std::string>("input_topic", "image_in");
    output_topic_ = declare_parameter<std::string>("output_topic", "image_out");
    output_shm_name_ = declare_parameter<std::string>(
        "output_shm_name", "/ros2_cuda_ipc_simple_increment");
    const int slot_count_param = declare_parameter<int>("slot_count", 4);
    slot_count_ =
        slot_count_param < 1 ? 1 : static_cast<std::size_t>(slot_count_param);
    device_index_ = declare_parameter<int>("device_index", 0);
    const int pending_ttl_ms = declare_parameter<int>("pending_ttl_ms", 300);
    pool_ =
        std::make_unique<detail::ImageSlotPool>(detail::ImageSlotPool::Config{
            output_shm_name_, slot_count_, device_index_,
            std::chrono::milliseconds{pending_ttl_ms}});

    if (!type_adaptation_enabled_) {
      RCLCPP_WARN(
          get_logger(),
          "type_adaptation_enabled=false is not supported in this port; "
          "defaulting to GPU type adaptation");
      type_adaptation_enabled_ = true;
    }

    auto err = cudaSetDevice(device_index_);
    if (err != cudaSuccess) {
      throw std::runtime_error("cudaSetDevice failed: " +
                               detail::cuda_error_to_string(err));
    }

    err = cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking);
    if (err != cudaSuccess) {
      throw std::runtime_error("cudaStreamCreateWithFlags failed: " +
                               detail::cuda_error_to_string(err));
    }

    rclcpp::PublisherOptions pub_options;
    pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    publisher_ = create_publisher<ros2_cuda_ipc_core::ImageView>(
        output_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        pub_options);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    subscription_ = create_subscription<ros2_cuda_ipc_core::ImageView>(
        input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&SimpleIncrementNode::on_image, this, std::placeholders::_1),
        sub_options);

    RCLCPP_INFO(get_logger(),
                "simple_increment configured: proc_count=%d inplace=%s "
                "slot_count=%zu device=%d",
                proc_count_, inplace_enabled_ ? "true" : "false", slot_count_,
                device_index_);
  }

  ~SimpleIncrementNode() override {
    if (stream_) {
      cudaStreamDestroy(stream_);
      stream_ = nullptr;
    }
  }

 private:
  void on_image(const ros2_cuda_ipc_core::ImageView &view) {
    if (!view.valid()) {
      RCLCPP_WARN(get_logger(), "Received invalid ImageView; dropping frame");
      return;
    }

    if (!layout_.has_value() || !layout_->matches(view)) {
      auto layout = detail::ImageLayout::from_view(view);
      if (!pool_->configure(layout)) {
        RCLCPP_ERROR(get_logger(), "Failed to configure output pool");
        return;
      }
      layout_ = layout;
      RCLCPP_INFO(get_logger(),
                  "Output pool configured for %ux%u x%u dtype=%u encoding=%s",
                  layout.width, layout.height, layout.channels,
                  static_cast<unsigned>(layout.dtype), layout.encoding.c_str());
    }

    auto err = cudaSetDevice(device_index_);
    if (err != cudaSuccess) {
      RCLCPP_ERROR(get_logger(), "cudaSetDevice failed: %s",
                   detail::cuda_error_to_string(err).c_str());
      return;
    }

    err = view.core.enqueue_ready_event(stream_);
    if (err != cudaSuccess) {
      RCLCPP_ERROR(get_logger(), "cudaStreamWaitEvent failed: %s",
                   detail::cuda_error_to_string(err).c_str());
      return;
    }

    auto reservation = pool_->reserve(publisher_->get_subscription_count());
    if (!reservation.has_value()) {
      RCLCPP_WARN(get_logger(), "Failed to reserve output slot");
      return;
    }

    const uint8_t *source = view.core.data<uint8_t>();
    auto &res = reservation.value();
    uint8_t *destination = res.device_ptr;

    if (source == nullptr || destination == nullptr) {
      RCLCPP_ERROR(get_logger(), "Null device pointer in increment pipeline");
      pool_->cancel(res);
      return;
    }

    if (proc_count_ <= 0) {
      err = cudaMemcpyAsync(destination, source, res.byte_size,
                            cudaMemcpyDeviceToDevice, stream_);
      if (err != cudaSuccess) {
        RCLCPP_ERROR(get_logger(), "cudaMemcpyAsync failed: %s",
                     detail::cuda_error_to_string(err).c_str());
        pool_->cancel(res);
        return;
      }
    } else {
      if (inplace_enabled_) {
        err = cudaMemcpyAsync(destination, source, res.byte_size,
                              cudaMemcpyDeviceToDevice, stream_);
        if (err != cudaSuccess) {
          RCLCPP_ERROR(get_logger(), "cudaMemcpyAsync failed: %s",
                       detail::cuda_error_to_string(err).c_str());
          pool_->cancel(res);
          return;
        }
        for (int i = 0; i < proc_count_; ++i) {
          err = compute_increment_inplace(static_cast<int>(res.byte_size),
                                          destination, stream_);
          if (err != cudaSuccess) {
            RCLCPP_ERROR(get_logger(), "compute_increment_inplace failed: %s",
                         detail::cuda_error_to_string(err).c_str());
            pool_->cancel(res);
            return;
          }
        }
      } else {
        err = compute_increment(static_cast<int>(res.byte_size), source,
                                destination, stream_);
        if (err != cudaSuccess) {
          RCLCPP_ERROR(get_logger(), "compute_increment failed: %s",
                       detail::cuda_error_to_string(err).c_str());
          pool_->cancel(res);
          return;
        }
        for (int i = 1; i < proc_count_; ++i) {
          err = compute_increment_inplace(static_cast<int>(res.byte_size),
                                          destination, stream_);
          if (err != cudaSuccess) {
            RCLCPP_ERROR(get_logger(), "compute_increment_inplace failed: %s",
                         detail::cuda_error_to_string(err).c_str());
            pool_->cancel(res);
            return;
          }
        }
      }
    }

    if (!pool_->finalize(res, stream_)) {
      pool_->cancel(res);
      return;
    }

    res.view.header = view.header;
    res.view.header.stamp = now();

    publisher_->publish(res.view);
  }

  int proc_count_ = 1;
  bool inplace_enabled_ = false;
  bool type_adaptation_enabled_ = true;
  std::string input_topic_;
  std::string output_topic_;
  std::string output_shm_name_;
  std::size_t slot_count_ = 4;
  int device_index_ = 0;

  std::unique_ptr<detail::ImageSlotPool> pool_;
  std::optional<detail::ImageLayout> layout_;
  cudaStream_t stream_ = nullptr;

  rclcpp::Publisher<ros2_cuda_ipc_core::ImageView>::SharedPtr publisher_;
  rclcpp::Subscription<ros2_cuda_ipc_core::ImageView>::SharedPtr subscription_;
};

}  // namespace simple_increment

RCLCPP_COMPONENTS_REGISTER_NODE(simple_increment::SimpleIncrementNode)
