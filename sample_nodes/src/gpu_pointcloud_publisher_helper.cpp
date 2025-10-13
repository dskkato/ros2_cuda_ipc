#include "sample_nodes/gpu_pointcloud_publisher_helper.hpp"

#include <chrono>
#include <stdexcept>
#include <vector>

#include "rclcpp/logging.hpp"
#include "ros2_cuda_ipc_core/cuda/cuda_util.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace sample_nodes {
namespace {

using ros2_cuda_ipc_core::cuda::cuda_error_to_string;

}  // namespace

GpuPointCloudPublisherHelper::GpuPointCloudPublisherHelper(const Config &config)
    : config_(config) {
  if (config_.slot_count == 0) {
    throw std::runtime_error("slot_count must be greater than zero");
  }

  cudaError_t err = cudaSetDevice(config_.device_index);
  if (err != cudaSuccess) {
    throw std::runtime_error("cudaSetDevice failed: " +
                             cuda_error_to_string(err));
  }

  point_step_ = sizeof(float) * 3;  // x, y, z
  cloud_size_bytes_ =
      static_cast<uint64_t>(config_.width) * config_.height * point_step_;

  stream_ = nullptr;
  err = cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking);
  if (err != cudaSuccess) {
    throw std::runtime_error("cudaStreamCreateWithFlags failed: " +
                             cuda_error_to_string(err));
  }

  fields_.clear();
  fields_.push_back({"x", 0u, sensor_msgs::msg::PointField::FLOAT32, 1u});
  fields_.push_back({"y", 4u, sensor_msgs::msg::PointField::FLOAT32, 1u});
  fields_.push_back({"z", 8u, sensor_msgs::msg::PointField::FLOAT32, 1u});

  initialise_shm();
  allocate_slots();
}

GpuPointCloudPublisherHelper::~GpuPointCloudPublisherHelper() {
  destroy_slots();
}

void GpuPointCloudPublisherHelper::initialise_shm() {
  if (!ros2_cuda_ipc_core::LeaseHandle::init(
          config_.shm_name, static_cast<uint32_t>(config_.slot_count))) {
    throw std::runtime_error("Failed to initialise lease shared memory: " +
                             config_.shm_name);
  }
}

void GpuPointCloudPublisherHelper::allocate_slots() {
  slots_.resize(config_.slot_count);
  for (std::size_t i = 0; i < slots_.size(); ++i) {
    auto &slot = slots_[i];
    slot.index = static_cast<uint32_t>(i);

    cudaError_t err = cudaMalloc(&slot.device_ptr, cloud_size_bytes_);
    if (err != cudaSuccess) {
      throw std::runtime_error("cudaMalloc failed: " +
                               cuda_error_to_string(err));
    }

    err = cudaEventCreateWithFlags(
        &slot.event, cudaEventDisableTiming | cudaEventInterprocess);
    if (err != cudaSuccess) {
      throw std::runtime_error("cudaEventCreateWithFlags failed: " +
                               cuda_error_to_string(err));
    }

    err = cudaIpcGetMemHandle(&slot.mem_handle, slot.device_ptr);
    if (err != cudaSuccess) {
      throw std::runtime_error("cudaIpcGetMemHandle failed: " +
                               cuda_error_to_string(err));
    }

    err = cudaIpcGetEventHandle(&slot.event_handle, slot.event);
    if (err != cudaSuccess) {
      throw std::runtime_error("cudaIpcGetEventHandle failed: " +
                               cuda_error_to_string(err));
    }
  }
}

void GpuPointCloudPublisherHelper::destroy_slots() noexcept {
  for (auto &slot : slots_) {
    if (slot.event) {
      cudaEventDestroy(slot.event);
      slot.event = nullptr;
    }
    if (slot.device_ptr) {
      cudaFree(slot.device_ptr);
      slot.device_ptr = nullptr;
    }
  }
  if (stream_) {
    cudaStreamDestroy(stream_);
    stream_ = nullptr;
  }
}

std::optional<ros2_cuda_ipc_core::PointCloud2View>
GpuPointCloudPublisherHelper::produce(size_t subscriber_count, float value) {
  if (slots_.empty()) {
    return std::nullopt;
  }

  const uint64_t point_count =
      static_cast<uint64_t>(config_.width) * config_.height;
  std::vector<float> host_cloud(point_count * 3, value);

  reclaim_stale_pending();

  auto free_slot =
      ros2_cuda_ipc_core::LeaseHandle::choose_empty_slot(config_.shm_name);
  if (!free_slot.has_value()) {
    RCLCPP_WARN(rclcpp::get_logger("GpuPointCloudPublisherHelper"),
                "No available GPU slots (all leases in use)");
    return std::nullopt;
  }
  if (free_slot.value() >= slots_.size()) {
    RCLCPP_ERROR(rclcpp::get_logger("GpuPointCloudPublisherHelper"),
                 "LeaseHandle returned invalid slot index %u",
                 free_slot.value());
    return std::nullopt;
  }

  auto &slot = slots_[free_slot.value()];

  auto gen = ros2_cuda_ipc_core::LeaseHandle::bump_generation(
      config_.shm_name, slot.index, subscriber_count);
  if (!gen.has_value()) {
    RCLCPP_WARN(rclcpp::get_logger("GpuPointCloudPublisherHelper"),
                "Failed to bump generation for slot %u", slot.index);
    return std::nullopt;
  }
  slot.generation = gen.value();

  const auto now = std::chrono::steady_clock::now();
  if (subscriber_count > 0 && config_.pending_ttl.count() > 0) {
    slot.pending_deadline = now + config_.pending_ttl;
  } else {
    slot.pending_deadline = {};
  }

  cudaError_t err = cudaMemcpy(slot.device_ptr, host_cloud.data(),
                               cloud_size_bytes_, cudaMemcpyHostToDevice);
  if (err != cudaSuccess) {
    RCLCPP_ERROR(rclcpp::get_logger("GpuPointCloudPublisherHelper"),
                 "cudaMemcpy failed: %s", cuda_error_to_string(err).c_str());
    return std::nullopt;
  }

  err = cudaEventRecord(slot.event, stream_);
  if (err != cudaSuccess) {
    RCLCPP_ERROR(rclcpp::get_logger("GpuPointCloudPublisherHelper"),
                 "cudaEventRecord failed: %s",
                 cuda_error_to_string(err).c_str());
    return std::nullopt;
  }

  ros2_cuda_ipc_core::PointCloud2View view;
  view.core.dev_ptr = slot.device_ptr;
  view.core.ready_evt = slot.event;
  view.core.device_id = config_.device_index;
  view.core.byte_size = cloud_size_bytes_;
  view.core.slot_id = slot.index;
  view.core.generation = slot.generation;
  view.core.shm_name = config_.shm_name;
  view.core.set_ipc_handles(slot.mem_handle, slot.event_handle);
  view.height = config_.height;
  view.width = config_.width;
  view.point_step = point_step_;
  view.row_step = point_step_ * config_.width;
  view.is_dense = config_.is_dense;
  view.fields = fields_;

  return view;
}

void GpuPointCloudPublisherHelper::reclaim_stale_pending() {
  if (config_.pending_ttl.count() <= 0) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  auto logger = rclcpp::get_logger("GpuPointCloudPublisherHelper");

  for (auto &slot : slots_) {
    if (!deadline_reached(slot.pending_deadline, now)) {
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
      RCLCPP_WARN(
          logger, "Force-cleared pending lease slot=%u after %lld ms timeout",
          slot.index, static_cast<long long>(config_.pending_ttl.count()));
      slot.pending_deadline = {};
    }
  }
}

}  // namespace sample_nodes
