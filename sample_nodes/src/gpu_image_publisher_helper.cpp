#include "sample_nodes/gpu_image_publisher_helper.hpp"

#include <chrono>
#include <stdexcept>

#include "rclcpp/logging.hpp"

namespace sample_nodes {
namespace {

std::string cuda_error_to_string(cudaError_t err) {
  return std::string(cudaGetErrorName(err)) + ": " + cudaGetErrorString(err);
}

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

}  // namespace

GpuImagePublisherHelper::GpuImagePublisherHelper(const Config &config)
    : config_(config) {
  if (config_.slot_count == 0) {
    throw std::runtime_error("slot_count must be greater than zero");
  }

  cudaError_t err = cudaSetDevice(config_.device_index);
  if (err != cudaSuccess) {
    throw std::runtime_error("cudaSetDevice failed: " +
                             cuda_error_to_string(err));
  }

  frame_size_bytes_ = static_cast<uint64_t>(config_.width) * config_.height *
                      config_.channels * dtype_bytes(config_.dtype);
  stream_ = nullptr;
  err = cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking);
  if (err != cudaSuccess) {
    throw std::runtime_error("cudaStreamCreateWithFlags failed: " +
                             cuda_error_to_string(err));
  }

  initialise_shm();
  allocate_slots();
}

GpuImagePublisherHelper::~GpuImagePublisherHelper() { destroy_slots(); }

void GpuImagePublisherHelper::initialise_shm() {
  if (!ros2_cuda_ipc_core::LeaseHandle::init(
          config_.shm_name, static_cast<uint32_t>(config_.slot_count))) {
    throw std::runtime_error("Failed to initialise lease shared memory: " +
                             config_.shm_name);
  }
}

void GpuImagePublisherHelper::allocate_slots() {
  slots_.resize(config_.slot_count);
  for (std::size_t i = 0; i < slots_.size(); ++i) {
    auto &slot = slots_[i];
    slot.index = static_cast<uint32_t>(i);

    cudaError_t err = cudaMalloc(&slot.device_ptr, frame_size_bytes_);
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

void GpuImagePublisherHelper::destroy_slots() noexcept {
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
  if (stream_) {
    cudaStreamDestroy(stream_);
    stream_ = nullptr;
  }
}

std::optional<ros2_cuda_ipc_core::ImageView> GpuImagePublisherHelper::produce(
    size_t subscriber_count, uint8_t fill_value) {
  if (slots_.empty()) {
    return std::nullopt;
  }

  reclaim_stale_pending();

  auto free_slot =
      ros2_cuda_ipc_core::LeaseHandle::choose_empty_slot(config_.shm_name);
  if (!free_slot.has_value()) {
    RCLCPP_WARN(rclcpp::get_logger("GpuImagePublisherHelper"),
                "No available GPU slots (all leases in use)");
    return std::nullopt;
  }
  if (free_slot.value() >= slots_.size()) {
    RCLCPP_ERROR(rclcpp::get_logger("GpuImagePublisherHelper"),
                 "LeaseHandle returned invalid slot index %u",
                 free_slot.value());
    return std::nullopt;
  }

  auto &slot = slots_[free_slot.value()];

  auto gen = ros2_cuda_ipc_core::LeaseHandle::bump_generation(
      config_.shm_name, slot.index, subscriber_count);
  if (!gen.has_value()) {
    RCLCPP_WARN(rclcpp::get_logger("GpuImagePublisherHelper"),
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

  cudaError_t err =
      cudaMemsetAsync(slot.device_ptr, fill_value, frame_size_bytes_, stream_);
  if (err != cudaSuccess) {
    RCLCPP_ERROR(rclcpp::get_logger("GpuImagePublisherHelper"),
                 "cudaMemsetAsync failed: %s",
                 cuda_error_to_string(err).c_str());
    return std::nullopt;
  }

  err = cudaEventRecord(slot.event, stream_);
  if (err != cudaSuccess) {
    RCLCPP_ERROR(rclcpp::get_logger("GpuImagePublisherHelper"),
                 "cudaEventRecord failed: %s",
                 cuda_error_to_string(err).c_str());
    return std::nullopt;
  }

  // optional wait to ensure data ready for demo (event ensures consumer waits)
  err = cudaStreamSynchronize(stream_);
  if (err != cudaSuccess) {
    RCLCPP_ERROR(rclcpp::get_logger("GpuImagePublisherHelper"),
                 "cudaStreamSynchronize failed: %s",
                 cuda_error_to_string(err).c_str());
    return std::nullopt;
  }

  ros2_cuda_ipc_core::ImageView view;
  view.core.dev_ptr = slot.device_ptr;
  view.core.ready_evt = slot.event;
  view.core.device_id = config_.device_index;
  view.core.byte_size = frame_size_bytes_;
  view.core.slot_id = slot.index;
  view.core.generation = slot.generation;
  view.core.shm_name = config_.shm_name;
  view.core.set_ipc_handles(slot.mem_handle, slot.event_handle);
  view.dtype = config_.dtype;
  view.shape = {config_.height, config_.width, config_.channels};
  const uint64_t elem_size = dtype_bytes(config_.dtype);
  view.strides = {
      static_cast<uint64_t>(config_.width) * config_.channels * elem_size,
      static_cast<uint64_t>(config_.channels) * elem_size, elem_size};
  if (!config_.encoding.empty()) {
    view.encoding = config_.encoding;
  } else {
    view.encoding.clear();
  }

  return view;
}

void GpuImagePublisherHelper::reclaim_stale_pending() {
  if (config_.pending_ttl.count() <= 0) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  auto logger = rclcpp::get_logger("GpuImagePublisherHelper");

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
