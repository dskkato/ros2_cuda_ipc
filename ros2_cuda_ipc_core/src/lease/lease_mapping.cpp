// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/lease/lease_mapping.hpp"

#include <fcntl.h>
#include <rcutils/logging_macros.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cstdint>
#include <limits>
#include <new>

namespace ros2_cuda_ipc_core::lease {
namespace {

constexpr uint32_t kShmMagic = 0x4C534531;  // 'LSE1'
constexpr uint32_t kLayoutVersion = 5;

struct ShmHeader {
  uint32_t magic;
  uint32_t layout_version;
  uint32_t capacity;
  uint32_t consumer_count;
  PublisherInstanceId publisher_instance_id;
};

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
    RCUTILS_LOG_ERROR_NAMED("ros2_cuda_ipc_core.lease_mapping",
                            "lease:create shm_open failed name=%s errno=%d",
                            shm_name.c_str(), errno);
    return nullptr;
  }
  if (ftruncate(fd, static_cast<off_t>(size)) != 0) {
    RCUTILS_LOG_ERROR_NAMED("ros2_cuda_ipc_core.lease_mapping",
                            "lease:create ftruncate failed name=%s errno=%d",
                            shm_name.c_str(), errno);
    close(fd);
    shm_unlink(shm_name.c_str());
    return nullptr;
  }
  void* addr = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  if (addr == MAP_FAILED) {
    RCUTILS_LOG_ERROR_NAMED("ros2_cuda_ipc_core.lease_mapping",
                            "lease:create mmap failed name=%s errno=%d",
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
  auto* slot_storage = static_cast<std::byte*>(addr) + sizeof(ShmHeader);
  for (uint32_t i = 0; i < capacity; ++i) {
    ::new (static_cast<void*>(slot_storage + i * sizeof(SlotMeta))) SlotMeta{};
  }
  auto* slots = reinterpret_cast<SlotMeta*>(slot_storage);

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
    RCUTILS_LOG_WARN_NAMED("ros2_cuda_ipc_core.lease_mapping",
                           "lease:attach shm_open failed name=%s errno=%d",
                           shm_name.c_str(), errno);
    return nullptr;
  }
  struct stat st{};
  if (fstat(fd, &st) != 0) {
    RCUTILS_LOG_WARN_NAMED("ros2_cuda_ipc_core.lease_mapping",
                           "lease:attach fstat failed name=%s errno=%d",
                           shm_name.c_str(), errno);
    close(fd);
    return nullptr;
  }
  if (st.st_size < static_cast<off_t>(sizeof(ShmHeader))) {
    RCUTILS_LOG_WARN_NAMED("ros2_cuda_ipc_core.lease_mapping",
                           "lease:attach segment too small name=%s",
                           shm_name.c_str());
    close(fd);
    return nullptr;
  }
  void* addr =
      mmap(nullptr, st.st_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  if (addr == MAP_FAILED) {
    RCUTILS_LOG_WARN_NAMED("ros2_cuda_ipc_core.lease_mapping",
                           "lease:attach mmap failed name=%s errno=%d",
                           shm_name.c_str(), errno);
    close(fd);
    return nullptr;
  }
  close(fd);

  auto* header = static_cast<ShmHeader*>(addr);
  std::size_t expected_size = 0;
  if (header->magic != kShmMagic || header->layout_version != kLayoutVersion ||
      header->publisher_instance_id != expected_instance_id) {
    RCUTILS_LOG_WARN_NAMED(
        "ros2_cuda_ipc_core.lease_mapping",
        "lease:attach header or publisher instance mismatch name=%s "
        "magic=%u ver=%u",
        shm_name.c_str(), header->magic, header->layout_version);
    munmap(addr, st.st_size);
    return nullptr;
  }
  if (!valid_layout(st, *header, &expected_size)) {
    RCUTILS_LOG_WARN_NAMED("ros2_cuda_ipc_core.lease_mapping",
                           "lease:attach invalid layout name=%s",
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

}  // namespace ros2_cuda_ipc_core::lease
