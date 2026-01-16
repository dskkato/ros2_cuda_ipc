#include "ros2_cuda_ipc_core/cuda/gpu_lease_pool.hpp"

#include <memory>

#include "rclcpp/logging.hpp"
#include "ros2_cuda_ipc_core/cuda/cuda_ipc_memory_backend.hpp"
#include "ros2_cuda_ipc_core/cuda/cuda_util.hpp"
#include "ros2_cuda_ipc_core/cuda/vmm_fd_memory_backend.hpp"
#include "ros2_cuda_ipc_core/lease_handle.hpp"
#include "ros2_cuda_ipc_core/memory_types.hpp"

namespace ros2_cuda_ipc_core::cuda {

namespace {
using Clock = std::chrono::steady_clock;

bool deadline_reached(const Clock::time_point &deadline,
                      const Clock::time_point &now) {
  return deadline.time_since_epoch().count() != 0 && now >= deadline;
}

std::unique_ptr<GpuLeasePool::MemoryBackend> make_backend(
    ros2_cuda_ipc_core::MemoryBackendKind backend) {
  if (backend == ros2_cuda_ipc_core::MemoryBackendKind::VMM_FD) {
    return make_vmm_fd_memory_backend();
  }
  return make_cuda_ipc_memory_backend();
}

}  // namespace

using ros2_cuda_ipc_core::LeaseHandle;

GpuLeasePool::GpuLeasePool(Config config, rclcpp::Logger logger)
    : config_(std::move(config)), logger_(std::move(logger)) {}

GpuLeasePool::~GpuLeasePool() { destroy_slots(); }

bool GpuLeasePool::initialise(uint64_t frame_size_bytes, int device_index) {
  if (config_.slot_count == 0) {
    RCLCPP_ERROR(logger_, "GpuLeasePool requires slot_count > 0");
    return false;
  }

  destroy_slots();

  cudaError_t err = cudaSetDevice(device_index);
  if (err != cudaSuccess) {
    RCLCPP_ERROR(logger_, "cudaSetDevice failed: %s",
                 cuda_error_to_string(err).c_str());
    return false;
  }

  if (!LeaseHandle::init(config_.shm_name,
                         static_cast<uint32_t>(config_.slot_count))) {
    RCLCPP_ERROR(logger_, "Failed to initialise lease shared memory %s",
                 config_.shm_name.c_str());
    return false;
  }

  frame_size_bytes_ = frame_size_bytes;
  device_index_ = device_index;
  slots_.assign(config_.slot_count, {});
  for (std::size_t i = 0; i < slots_.size(); ++i) {
    slots_[i].index = static_cast<uint32_t>(i);
  }

  if (!allocate_slots()) {
    destroy_slots();
    return false;
  }

  initialised_ = true;
  return true;
}

void GpuLeasePool::reset() noexcept { destroy_slots(); }

bool GpuLeasePool::matches(uint64_t frame_size_bytes,
                           int device_index) const noexcept {
  return initialised_ && frame_size_bytes_ == frame_size_bytes &&
         device_index_ == device_index;
}

std::optional<GpuLeasePool::Slot *> GpuLeasePool::acquire(
    std::size_t subscriber_count) {
  if (!initialised_) {
    return std::nullopt;
  }

  auto free_slot = LeaseHandle::choose_empty_slot(config_.shm_name);
  if (!free_slot.has_value()) {
    return std::nullopt;
  }
  if (free_slot.value() >= slots_.size()) {
    RCLCPP_ERROR(logger_, "LeaseHandle returned invalid slot index %u",
                 free_slot.value());
    return std::nullopt;
  }

  Slot &slot = slots_[free_slot.value()];

  auto generation = LeaseHandle::bump_generation(
      config_.shm_name, slot.index, static_cast<uint32_t>(subscriber_count));
  if (!generation.has_value()) {
    RCLCPP_WARN(logger_, "Failed to bump generation for slot %u", slot.index);
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

void GpuLeasePool::reclaim_stale_pending() {
  if (!initialised_ || config_.pending_ttl.count() <= 0) {
    return;
  }

  const auto now = Clock::now();

  for (auto &slot : slots_) {
    if (!deadline_reached(slot.pending_deadline, now)) {
      continue;
    }

    auto pending = LeaseHandle::current_pending(config_.shm_name, slot.index);
    if (!pending.has_value()) {
      continue;
    }
    if (pending.value() == 0) {
      slot.pending_deadline = {};
      continue;
    }

    auto refcnt = LeaseHandle::current_refcount(config_.shm_name, slot.index);
    if (!refcnt.has_value() || refcnt.value() != 0) {
      continue;
    }

    if (LeaseHandle::force_clear_pending(config_.shm_name, slot.index)) {
      RCLCPP_WARN(
          logger_, "Force-cleared pending lease slot=%u after %lld ms timeout",
          slot.index, static_cast<long long>(config_.pending_ttl.count()));
      slot.pending_deadline = {};
    }
  }
}

bool GpuLeasePool::cancel_pending(Slot &slot) {
  slot.pending_deadline = {};
  if (!initialised_) {
    return false;
  }

  if (LeaseHandle::force_clear_pending(config_.shm_name, slot.index)) {
    RCLCPP_DEBUG(logger_, "Cleared pending lease for slot %u", slot.index);
    return true;
  }
  return false;
}

bool GpuLeasePool::allocate_slots() {
  memory_backend_ = make_backend(config_.backend);
  if (!memory_backend_) {
    RCLCPP_ERROR(logger_, "Failed to create memory backend");
    return false;
  }

  if (!memory_backend_->allocate(frame_size_bytes_, device_index_, slots_,
                                 logger_)) {
    memory_backend_.reset();
    return false;
  }

  for (auto &slot : slots_) {
    cudaError_t err = cudaEventCreateWithFlags(
        &slot.event, cudaEventDisableTiming | cudaEventInterprocess);
    if (err != cudaSuccess) {
      RCLCPP_ERROR(logger_, "cudaEventCreateWithFlags failed: %s",
                   cuda_error_to_string(err).c_str());
      return false;
    }

    err = cudaIpcGetEventHandle(&slot.event_handle, slot.event);
    if (err != cudaSuccess) {
      RCLCPP_ERROR(logger_, "cudaIpcGetEventHandle failed: %s",
                   cuda_error_to_string(err).c_str());
      return false;
    }
  }

  return true;
}

void GpuLeasePool::destroy_slots() noexcept {
  if (device_index_ >= 0) {
    cudaSetDevice(device_index_);
  }

  for (auto &slot : slots_) {
    if (slot.event) {
      const cudaError_t event_err = cudaEventDestroy(slot.event);
      if (event_err != cudaSuccess) {
        RCLCPP_ERROR(logger_, "cudaEventDestroy failed for slot %u: %s",
                     slot.index, cuda_error_to_string(event_err).c_str());
      }
      slot.event = nullptr;
    }
    slot.pending_deadline = {};
  }
  if (memory_backend_) {
    memory_backend_->destroy(slots_, logger_);
    memory_backend_.reset();
  }
  slots_.clear();
  frame_size_bytes_ = 0;
  device_index_ = -1;
  initialised_ = false;
}

}  // namespace ros2_cuda_ipc_core::cuda
