// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/lease/lease_handle.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cstdint>
#include <limits>
#include <optional>
#include <rclcpp/logging.hpp>
#include <thread>

namespace ros2_cuda_ipc_core::lease {
namespace {

constexpr uint32_t kShmMagic = 0x4C534531;  // 'LSE1'
constexpr uint32_t kLayoutVersion = 3;
constexpr uint32_t kCancelReservationAttempts = 1024;

struct ShmHeader {
  uint32_t magic;
  uint32_t layout_version;
  uint32_t capacity;
  uint32_t consumer_count;
  PublisherInstanceId publisher_instance_id;
};

inline std::atomic<uint32_t>& as_atomic(uint32_t& value) {
  return reinterpret_cast<std::atomic<uint32_t>&>(value);
}

rclcpp::Logger lease_logger() {
  static rclcpp::Logger logger =
      rclcpp::get_logger("ros2_cuda_ipc_core.LeaseHandle");
  return logger;
}

bool valid_layout(const struct stat& st, const ShmHeader& header,
                  std::size_t* expected_size) {
  if (st.st_size < static_cast<off_t>(sizeof(ShmHeader)) ||
      header.capacity == 0 ||
      header.capacity >
          (std::numeric_limits<std::size_t>::max() - sizeof(ShmHeader)) /
              sizeof(SlotMeta)) {
    return false;
  }
  *expected_size = sizeof(ShmHeader) +
                   static_cast<std::size_t>(header.capacity) * sizeof(SlotMeta);
  return *expected_size <= static_cast<std::size_t>(st.st_size);
}

}  // namespace

LeaseMapping::~LeaseMapping() {
  if (addr_ != nullptr && mapped_size_ != 0) {
    munmap(addr_, mapped_size_);
  }
}

std::shared_ptr<LeaseMapping> LeaseMapping::create(
    const std::string& shm_name, const PublisherInstanceId& instance_id,
    uint32_t capacity) {
  if (capacity == 0 || is_nil(instance_id)) {
    return nullptr;
  }
  if (capacity > (std::numeric_limits<std::size_t>::max() - sizeof(ShmHeader)) /
                     sizeof(SlotMeta)) {
    return nullptr;
  }
  const std::size_t size =
      sizeof(ShmHeader) + static_cast<std::size_t>(capacity) * sizeof(SlotMeta);
  const int fd = shm_open(shm_name.c_str(), O_CREAT | O_EXCL | O_RDWR, 0660);
  if (fd < 0) {
    RCLCPP_ERROR(lease_logger(),
                 "lease:create shm_open failed name=%s errno=%d",
                 shm_name.c_str(), errno);
    return nullptr;
  }
  if (ftruncate(fd, static_cast<off_t>(size)) != 0) {
    RCLCPP_ERROR(lease_logger(),
                 "lease:create ftruncate failed name=%s errno=%d",
                 shm_name.c_str(), errno);
    close(fd);
    shm_unlink(shm_name.c_str());
    return nullptr;
  }
  void* addr = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  if (addr == MAP_FAILED) {
    RCLCPP_ERROR(lease_logger(), "lease:create mmap failed name=%s errno=%d",
                 shm_name.c_str(), errno);
    close(fd);
    shm_unlink(shm_name.c_str());
    return nullptr;
  }
  close(fd);

  auto* header = static_cast<ShmHeader*>(addr);
  header->magic = kShmMagic;
  header->layout_version = kLayoutVersion;
  header->capacity = capacity;
  header->consumer_count = 0;
  header->publisher_instance_id = instance_id;
  auto* slots = reinterpret_cast<SlotMeta*>(header + 1);
  for (uint32_t i = 0; i < capacity; ++i) {
    as_atomic(slots[i].generation).store(0u, std::memory_order_relaxed);
    as_atomic(slots[i].refcnt).store(0u, std::memory_order_relaxed);
    as_atomic(slots[i].pending).store(0u, std::memory_order_relaxed);
    as_atomic(slots[i].reserved).store(0u, std::memory_order_relaxed);
  }

  auto mapping = std::shared_ptr<LeaseMapping>(new LeaseMapping);
  mapping->shm_name_ = shm_name;
  mapping->publisher_instance_id_ = instance_id;
  mapping->capacity_ = capacity;
  mapping->mapped_size_ = size;
  mapping->addr_ = addr;
  mapping->slots_ = slots;
  return mapping;
}

std::shared_ptr<LeaseMapping> LeaseMapping::attach(
    const std::string& shm_name,
    const PublisherInstanceId& expected_instance_id) {
  if (is_nil(expected_instance_id)) {
    return nullptr;
  }
  const int fd = shm_open(shm_name.c_str(), O_RDWR, 0660);
  if (fd < 0) {
    RCLCPP_WARN(lease_logger(), "lease:attach shm_open failed name=%s errno=%d",
                shm_name.c_str(), errno);
    return nullptr;
  }
  struct stat st{};
  if (fstat(fd, &st) != 0) {
    RCLCPP_WARN(lease_logger(), "lease:attach fstat failed name=%s errno=%d",
                shm_name.c_str(), errno);
    close(fd);
    return nullptr;
  }
  if (st.st_size < static_cast<off_t>(sizeof(ShmHeader))) {
    RCLCPP_WARN(lease_logger(), "lease:attach segment too small name=%s",
                shm_name.c_str());
    close(fd);
    return nullptr;
  }
  void* addr =
      mmap(nullptr, st.st_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  if (addr == MAP_FAILED) {
    RCLCPP_WARN(lease_logger(), "lease:attach mmap failed name=%s errno=%d",
                shm_name.c_str(), errno);
    close(fd);
    return nullptr;
  }
  close(fd);

  auto* header = static_cast<ShmHeader*>(addr);
  std::size_t expected_size = 0;
  if (header->magic != kShmMagic || header->layout_version != kLayoutVersion ||
      header->publisher_instance_id != expected_instance_id) {
    RCLCPP_WARN(lease_logger(),
                "lease:attach header or publisher instance mismatch name=%s "
                "magic=%u ver=%u",
                shm_name.c_str(), header->magic, header->layout_version);
    munmap(addr, st.st_size);
    return nullptr;
  }
  if (!valid_layout(st, *header, &expected_size)) {
    RCLCPP_WARN(lease_logger(), "lease:attach invalid layout name=%s",
                shm_name.c_str());
    munmap(addr, st.st_size);
    return nullptr;
  }

  auto mapping = std::shared_ptr<LeaseMapping>(new LeaseMapping);
  mapping->shm_name_ = shm_name;
  mapping->publisher_instance_id_ = header->publisher_instance_id;
  mapping->capacity_ = header->capacity;
  mapping->mapped_size_ = static_cast<std::size_t>(st.st_size);
  mapping->addr_ = addr;
  mapping->slots_ = reinterpret_cast<SlotMeta*>(header + 1);
  return mapping;
}

LeaseHandle::LeaseHandle(std::shared_ptr<LeaseMapping> mapping, SlotMeta* slot,
                         uint32_t slot_id, uint32_t generation)
    : mapping_(std::move(mapping)),
      slot_meta_(slot),
      slot_id_(slot_id),
      generation_(generation) {}

LeaseHandle::LeaseHandle(LeaseHandle&& other) noexcept {
  *this = std::move(other);
}

LeaseHandle& LeaseHandle::operator=(LeaseHandle&& other) noexcept {
  if (this == &other) return *this;
  release();
  mapping_ = std::move(other.mapping_);
  slot_meta_ = other.slot_meta_;
  slot_id_ = other.slot_id_;
  generation_ = other.generation_;
  other.slot_meta_ = nullptr;
  other.slot_id_ = 0;
  other.generation_ = 0;
  return *this;
}

LeaseHandle::~LeaseHandle() { release(); }

void LeaseHandle::release() noexcept {
  if (!slot_meta_) return;
  auto& ref = as_atomic(slot_meta_->refcnt);
  const uint32_t previous = ref.fetch_sub(1, std::memory_order_acq_rel);
  if (previous == 0) {
    RCLCPP_ERROR(lease_logger(), "lease:refcnt_underflow slot=%u", slot_id_);
    ref.store(0, std::memory_order_release);
  }
  slot_meta_ = nullptr;
  slot_id_ = 0;
  generation_ = 0;
  mapping_.reset();
}

std::optional<uint32_t> LeaseHandle::current_generation(
    const std::shared_ptr<LeaseMapping>& mapping, uint32_t slot_id) {
  if (!mapping || slot_id >= mapping->capacity()) return std::nullopt;
  return as_atomic(mapping->slot(slot_id)->generation)
      .load(std::memory_order_acquire);
}

std::optional<uint32_t> LeaseHandle::current_refcount(
    const std::shared_ptr<LeaseMapping>& mapping, uint32_t slot_id) {
  if (!mapping || slot_id >= mapping->capacity()) return std::nullopt;
  return as_atomic(mapping->slot(slot_id)->refcnt)
      .load(std::memory_order_acquire);
}

std::optional<uint32_t> LeaseHandle::current_pending(
    const std::shared_ptr<LeaseMapping>& mapping, uint32_t slot_id) {
  if (!mapping || slot_id >= mapping->capacity()) return std::nullopt;
  return as_atomic(mapping->slot(slot_id)->pending)
      .load(std::memory_order_acquire);
}

std::optional<LeaseHandle::PublisherReservation>
LeaseHandle::reserve_for_publish(const std::shared_ptr<LeaseMapping>& mapping,
                                 uint32_t pending_count) {
  if (!mapping || mapping->capacity() == 0) return std::nullopt;
  const uint32_t capacity = mapping->capacity();
  const uint32_t start =
      mapping->next_slot().fetch_add(1, std::memory_order_relaxed) % capacity;
  for (uint32_t offset = 0; offset < capacity; ++offset) {
    const uint32_t slot_id = (start + offset) % capacity;
    SlotMeta& slot = *mapping->slot(slot_id);
    auto& ref = as_atomic(slot.refcnt);
    auto& pending = as_atomic(slot.pending);
    if (ref.load(std::memory_order_acquire) != 0 ||
        pending.load(std::memory_order_acquire) != 0)
      continue;
    auto& reserved = as_atomic(slot.reserved);
    uint32_t expected = 0;
    if (!reserved.compare_exchange_strong(
            expected, 1, std::memory_order_acq_rel, std::memory_order_acquire))
      continue;
    if (ref.load(std::memory_order_acquire) != 0 ||
        pending.load(std::memory_order_acquire) != 0) {
      reserved.store(0, std::memory_order_release);
      continue;
    }
    auto& generation = as_atomic(slot.generation);
    const uint32_t next = generation.load(std::memory_order_relaxed) + 1;
    generation.store(next, std::memory_order_release);
    pending.store(pending_count, std::memory_order_release);
    reserved.store(0, std::memory_order_release);
    mapping->next_slot().store((slot_id + 1) % capacity,
                               std::memory_order_relaxed);
    return PublisherReservation{mapping, slot_id, next};
  }
  return std::nullopt;
}

bool LeaseHandle::force_clear_pending(
    const std::shared_ptr<LeaseMapping>& mapping, uint32_t slot_id) {
  if (!mapping || slot_id >= mapping->capacity()) return false;
  SlotMeta* slot = mapping->slot(slot_id);
  auto& reserved = as_atomic(slot->reserved);
  uint32_t expected = 0;
  if (!reserved.compare_exchange_strong(expected, 1, std::memory_order_acq_rel,
                                        std::memory_order_acquire))
    return false;
  auto& pending = as_atomic(slot->pending);
  if (pending.load(std::memory_order_acquire) == 0) {
    reserved.store(0, std::memory_order_release);
    return true;
  }
  if (as_atomic(slot->refcnt).load(std::memory_order_acquire) != 0) {
    reserved.store(0, std::memory_order_release);
    return false;
  }
  pending.store(0, std::memory_order_release);
  reserved.store(0, std::memory_order_release);
  return true;
}

bool LeaseHandle::cancel_pending(const std::shared_ptr<LeaseMapping>& mapping,
                                 uint32_t slot_id, uint32_t generation) {
  if (!mapping || slot_id >= mapping->capacity()) return false;
  SlotMeta& slot = *mapping->slot(slot_id);
  auto& reserved = as_atomic(slot.reserved);
  bool acquired = false;
  for (uint32_t attempt = 0; attempt < kCancelReservationAttempts; ++attempt) {
    uint32_t expected = 0;
    if (reserved.compare_exchange_strong(expected, 1, std::memory_order_acq_rel,
                                         std::memory_order_acquire)) {
      acquired = true;
      break;
    }
    std::this_thread::yield();
  }
  if (!acquired) {
    RCLCPP_ERROR(lease_logger(),
                 "lease:cancel_reservation_contention slot=%u gen=%u", slot_id,
                 generation);
    return false;
  }
  if (as_atomic(slot.generation).load(std::memory_order_acquire) !=
          generation ||
      as_atomic(slot.refcnt).load(std::memory_order_acquire) != 0) {
    reserved.store(0, std::memory_order_release);
    return false;
  }
  as_atomic(slot.pending).store(0, std::memory_order_release);
  reserved.store(0, std::memory_order_release);
  return true;
}

LeaseHandle LeaseHandle::acquire(const std::shared_ptr<LeaseMapping>& mapping,
                                 uint32_t slot_id, uint32_t generation) {
  if (!mapping || slot_id >= mapping->capacity()) return LeaseHandle{};
  SlotMeta* slot = mapping->slot(slot_id);
  auto& gen = as_atomic(slot->generation);
  auto& ref = as_atomic(slot->refcnt);
  auto& pending = as_atomic(slot->pending);
  auto& reserved = as_atomic(slot->reserved);
  if (reserved.load(std::memory_order_acquire) != 0 ||
      gen.load(std::memory_order_acquire) != generation)
    return LeaseHandle{};

  uint32_t observed_ref = ref.load(std::memory_order_acquire);
  while (true) {
    if (observed_ref == UINT32_MAX) {
      RCLCPP_ERROR(lease_logger(), "lease:ref_overflow slot=%u", slot_id);
      return LeaseHandle{};
    }
    if (ref.compare_exchange_weak(observed_ref, observed_ref + 1,
                                  std::memory_order_acq_rel,
                                  std::memory_order_acquire))
      break;
  }
  const uint32_t recheck_gen = gen.load(std::memory_order_acquire);
  if (reserved.load(std::memory_order_acquire) != 0 ||
      recheck_gen != generation) {
    ref.fetch_sub(1, std::memory_order_acq_rel);
    return LeaseHandle{};
  }
  uint32_t observed_pending = pending.load(std::memory_order_acquire);
  while (observed_pending != 0) {
    if (pending.compare_exchange_weak(observed_pending, observed_pending - 1,
                                      std::memory_order_acq_rel,
                                      std::memory_order_acquire))
      break;
  }
  return LeaseHandle(mapping, slot, slot_id, generation);
}

}  // namespace ros2_cuda_ipc_core::lease
