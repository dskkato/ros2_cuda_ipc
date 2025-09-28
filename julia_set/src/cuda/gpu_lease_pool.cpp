#include "julia_set/cuda/gpu_lease_pool.hpp"

#include "julia_set/cuda/cuda_util.hpp"
#include "rclcpp/logging.hpp"
#include "ros2_cuda_ipc_core/lease_handle.hpp"

namespace julia_set {
namespace {
using Clock = std::chrono::steady_clock;

bool deadline_reached(const Clock::time_point &deadline,
                      const Clock::time_point &now) {
  return deadline.time_since_epoch().count() != 0 && now >= deadline;
}

}  // namespace

GpuLeasePool::GpuLeasePool(Config config) : config_(std::move(config)) {}

GpuLeasePool::~GpuLeasePool() {
  destroy_slots(rclcpp::get_logger("GpuLeasePool"));
}

bool GpuLeasePool::initialise(uint64_t frame_size_bytes, int device_index,
                              const rclcpp::Logger &logger) {
  if (config_.slot_count == 0) {
    RCLCPP_ERROR(logger, "GpuLeasePool requires slot_count > 0");
    return false;
  }

  destroy_slots(logger);

  cudaError_t err = cudaSetDevice(device_index);
  if (err != cudaSuccess) {
    RCLCPP_ERROR(logger, "cudaSetDevice failed: %s",
                 cuda_error_to_string(err).c_str());
    return false;
  }

  if (!ros2_cuda_ipc_core::LeaseHandle::init(
          config_.shm_name, static_cast<uint32_t>(config_.slot_count))) {
    RCLCPP_ERROR(logger, "Failed to initialise lease shared memory %s",
                 config_.shm_name.c_str());
    return false;
  }

  frame_size_bytes_ = frame_size_bytes;
  device_index_ = device_index;
  slots_.assign(config_.slot_count, {});
  for (std::size_t i = 0; i < slots_.size(); ++i) {
    slots_[i].index = static_cast<uint32_t>(i);
  }

  if (!allocate_slots(logger)) {
    destroy_slots(logger);
    return false;
  }

  initialised_ = true;
  return true;
}

void GpuLeasePool::reset(const rclcpp::Logger &logger) noexcept {
  destroy_slots(logger);
}

bool GpuLeasePool::matches(uint64_t frame_size_bytes,
                           int device_index) const noexcept {
  return initialised_ && frame_size_bytes_ == frame_size_bytes &&
         device_index_ == device_index;
}

std::optional<GpuLeasePool::Slot *> GpuLeasePool::acquire(
    std::size_t subscriber_count, const rclcpp::Logger &logger) {
  if (!initialised_) {
    return std::nullopt;
  }

  auto free_slot =
      ros2_cuda_ipc_core::LeaseHandle::choose_empty_slot(config_.shm_name);
  if (!free_slot.has_value()) {
    return std::nullopt;
  }
  if (free_slot.value() >= slots_.size()) {
    RCLCPP_ERROR(logger, "LeaseHandle returned invalid slot index %u",
                 free_slot.value());
    return std::nullopt;
  }

  Slot &slot = slots_[free_slot.value()];

  auto generation = ros2_cuda_ipc_core::LeaseHandle::bump_generation(
      config_.shm_name, slot.index, static_cast<uint32_t>(subscriber_count));
  if (!generation.has_value()) {
    RCLCPP_WARN(logger, "Failed to bump generation for slot %u", slot.index);
    return std::nullopt;
  }
  slot.generation = generation.value();

  if (subscriber_count > 0 && config_.pending_ttl.count() > 0) {
    slot.pending_deadline = Clock::now() + config_.pending_ttl;
  } else {
    slot.pending_deadline = {};
  }

  return &slot;
}

void GpuLeasePool::reclaim_stale_pending(const rclcpp::Logger &logger) {
  if (!initialised_ || config_.pending_ttl.count() <= 0) {
    return;
  }

  const auto now = Clock::now();

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

bool GpuLeasePool::cancel_pending(Slot &slot, const rclcpp::Logger &logger) {
  slot.pending_deadline = {};
  if (!initialised_) {
    return false;
  }

  if (ros2_cuda_ipc_core::LeaseHandle::force_clear_pending(config_.shm_name,
                                                           slot.index)) {
    RCLCPP_DEBUG(logger, "Cleared pending lease for slot %u", slot.index);
    return true;
  }
  return false;
}

bool GpuLeasePool::allocate_slots(const rclcpp::Logger &logger) {
  for (auto &slot : slots_) {
    cudaError_t err = cudaMalloc(&slot.device_ptr, frame_size_bytes_);
    if (err != cudaSuccess) {
      RCLCPP_ERROR(logger, "cudaMalloc failed: %s",
                   cuda_error_to_string(err).c_str());
      return false;
    }

    err = cudaEventCreateWithFlags(
        &slot.event, cudaEventDisableTiming | cudaEventInterprocess);
    if (err != cudaSuccess) {
      RCLCPP_ERROR(logger, "cudaEventCreateWithFlags failed: %s",
                   cuda_error_to_string(err).c_str());
      return false;
    }

    err = cudaIpcGetMemHandle(&slot.mem_handle, slot.device_ptr);
    if (err != cudaSuccess) {
      RCLCPP_ERROR(logger, "cudaIpcGetMemHandle failed: %s",
                   cuda_error_to_string(err).c_str());
      return false;
    }

    err = cudaIpcGetEventHandle(&slot.event_handle, slot.event);
    if (err != cudaSuccess) {
      RCLCPP_ERROR(logger, "cudaIpcGetEventHandle failed: %s",
                   cuda_error_to_string(err).c_str());
      return false;
    }
  }

  return true;
}

void GpuLeasePool::destroy_slots(const rclcpp::Logger &logger) noexcept {
  if (device_index_ >= 0) {
    cudaSetDevice(device_index_);
  }

  for (auto &slot : slots_) {
    if (slot.event) {
      const cudaError_t event_err = cudaEventDestroy(slot.event);
      if (event_err != cudaSuccess) {
        RCLCPP_ERROR(logger, "cudaEventDestroy failed for slot %u: %s",
                     slot.index, cuda_error_to_string(event_err).c_str());
      }
      slot.event = nullptr;
    }
    if (slot.device_ptr) {
      const cudaError_t free_err = cudaFree(slot.device_ptr);
      if (free_err != cudaSuccess) {
        RCLCPP_ERROR(logger, "cudaFree failed for slot %u: %s", slot.index,
                     cuda_error_to_string(free_err).c_str());
      }
      slot.device_ptr = nullptr;
    }
    slot.pending_deadline = {};
  }
  slots_.clear();
  frame_size_bytes_ = 0;
  device_index_ = -1;
  initialised_ = false;
}

}  // namespace julia_set
