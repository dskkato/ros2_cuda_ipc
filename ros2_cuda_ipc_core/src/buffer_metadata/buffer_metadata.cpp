// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/buffer_metadata/buffer_metadata.hpp"

#include <fcntl.h>
#include <rcutils/logging_macros.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <uuid/uuid.h>

#include <cerrno>
#include <cstring>
#include <limits>
#include <new>

namespace ros2_cuda_ipc_core::buffer_metadata {
namespace {

constexpr std::size_t kMetadataSize = sizeof(BlockMetadata);

uint64_t initial_uid() {
  uuid_t uuid;
  uuid_generate_random(uuid);
  uint64_t value = 0;
  std::memcpy(&value, uuid, sizeof(value));
  // A nonzero random base distinguishes a recreated block even when a PID is
  // reused. Zero is retained as the invalid/unpublished sentinel only.
  return value == 0 || value == std::numeric_limits<uint64_t>::max() ? 1
                                                                     : value;
}

}  // namespace

std::shared_ptr<BufferMetadata> BufferMetadata::make_mapping(
    const std::string& shm_name, void* addr, std::size_t mapped_size) {
  auto mapping = std::shared_ptr<BufferMetadata>(new BufferMetadata);
  mapping->shm_name_ = shm_name;
  mapping->mapped_size_ = mapped_size;
  mapping->addr_ = addr;
  mapping->metadata_ = static_cast<BlockMetadata*>(addr);
  return mapping;
}

BufferMetadata::~BufferMetadata() {
  if (addr_ != nullptr && mapped_size_ != 0) {
    munmap(addr_, mapped_size_);
  }
}

std::shared_ptr<BufferMetadata> BufferMetadata::create(
    const std::string& shm_name) {
  const int fd = shm_open(shm_name.c_str(), O_CREAT | O_EXCL | O_RDWR, 0660);
  if (fd < 0) {
    RCUTILS_LOG_ERROR_NAMED(
        "ros2_cuda_ipc_core.buffer_metadata",
        "buffer_metadata:create shm_open failed name=%s errno=%d",
        shm_name.c_str(), errno);
    return nullptr;
  }
  if (ftruncate(fd, static_cast<off_t>(kMetadataSize)) != 0) {
    RCUTILS_LOG_ERROR_NAMED(
        "ros2_cuda_ipc_core.buffer_metadata",
        "buffer_metadata:create ftruncate failed name=%s errno=%d",
        shm_name.c_str(), errno);
    close(fd);
    shm_unlink(shm_name.c_str());
    return nullptr;
  }
  void* addr =
      mmap(nullptr, kMetadataSize, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  if (addr == MAP_FAILED) {
    RCUTILS_LOG_ERROR_NAMED(
        "ros2_cuda_ipc_core.buffer_metadata",
        "buffer_metadata:create mmap failed name=%s errno=%d", shm_name.c_str(),
        errno);
    close(fd);
    shm_unlink(shm_name.c_str());
    return nullptr;
  }
  close(fd);
  auto* metadata = ::new (addr) BlockMetadata{};
  metadata->uid.store(initial_uid(), std::memory_order_relaxed);
  return BufferMetadata::make_mapping(shm_name, addr, kMetadataSize);
}

std::shared_ptr<BufferMetadata> BufferMetadata::attach(
    const std::string& shm_name) {
  const int fd = shm_open(shm_name.c_str(), O_RDWR, 0660);
  if (fd < 0) {
    RCUTILS_LOG_WARN_NAMED(
        "ros2_cuda_ipc_core.buffer_metadata",
        "buffer_metadata:attach shm_open failed name=%s errno=%d",
        shm_name.c_str(), errno);
    return nullptr;
  }
  struct stat st{};
  if (fstat(fd, &st) != 0 || st.st_size != static_cast<off_t>(kMetadataSize)) {
    RCUTILS_LOG_WARN_NAMED(
        "ros2_cuda_ipc_core.buffer_metadata",
        "buffer_metadata:attach invalid object size name=%s size=%lld",
        shm_name.c_str(), static_cast<long long>(st.st_size));
    close(fd);
    return nullptr;
  }
  void* addr =
      mmap(nullptr, kMetadataSize, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  close(fd);
  if (addr == MAP_FAILED) {
    RCUTILS_LOG_WARN_NAMED(
        "ros2_cuda_ipc_core.buffer_metadata",
        "buffer_metadata:attach mmap failed name=%s errno=%d", shm_name.c_str(),
        errno);
    return nullptr;
  }
  return BufferMetadata::make_mapping(shm_name, addr, kMetadataSize);
}

}  // namespace ros2_cuda_ipc_core::buffer_metadata
