// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/buffer_metadata/buffer_metadata.hpp"

#include <fcntl.h>
#include <rcutils/logging_macros.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <limits>
#include <new>
#include <sstream>

namespace ros2_cuda_ipc_core::buffer_metadata {
namespace {

bool valid_name(const std::string& name) {
  return name.size() > 1 && name.front() == '/' &&
         name.find('/', 1) == std::string::npos;
}

std::shared_ptr<BufferMetadata> map_created(const std::string& shm_name,
                                            uint64_t initial_uid) {
  if (!valid_name(shm_name) || initial_uid == 0) return nullptr;
  const int fd = shm_open(shm_name.c_str(), O_CREAT | O_EXCL | O_RDWR, 0660);
  if (fd < 0) {
    RCUTILS_LOG_ERROR_NAMED("ros2_cuda_ipc_core.buffer_metadata",
                            "create shm_open failed name=%s errno=%d",
                            shm_name.c_str(), errno);
    return nullptr;
  }
  constexpr std::size_t size = sizeof(BlockMetadata);
  if (ftruncate(fd, static_cast<off_t>(size)) != 0) {
    close(fd);
    shm_unlink(shm_name.c_str());
    return nullptr;
  }
  void* addr = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  close(fd);
  if (addr == MAP_FAILED) {
    shm_unlink(shm_name.c_str());
    return nullptr;
  }
  auto* block = ::new (addr) BlockMetadata{};
  block->uid.store(initial_uid, std::memory_order_relaxed);
  auto mapping = std::shared_ptr<BufferMetadata>(new BufferMetadata);
  mapping->shm_name_ = shm_name;
  mapping->mapped_size_ = size;
  mapping->addr_ = addr;
  mapping->block_ = block;
  return mapping;
}

std::shared_ptr<BufferMetadata> map_attached(const std::string& shm_name) {
  if (!valid_name(shm_name)) return nullptr;
  const int fd = shm_open(shm_name.c_str(), O_RDWR, 0660);
  if (fd < 0) return nullptr;
  struct stat st{};
  if (fstat(fd, &st) != 0 || st.st_size != sizeof(BlockMetadata)) {
    close(fd);
    return nullptr;
  }
  void* addr = mmap(nullptr, sizeof(BlockMetadata), PROT_READ | PROT_WRITE,
                    MAP_SHARED, fd, 0);
  close(fd);
  if (addr == MAP_FAILED) return nullptr;
  auto* block = static_cast<BlockMetadata*>(addr);
  if (block->uid.load(std::memory_order_acquire) == 0) {
    munmap(addr, sizeof(BlockMetadata));
    return nullptr;
  }
  auto mapping = std::shared_ptr<BufferMetadata>(new BufferMetadata);
  mapping->shm_name_ = shm_name;
  mapping->mapped_size_ = sizeof(BlockMetadata);
  mapping->addr_ = addr;
  mapping->block_ = block;
  return mapping;
}

}  // namespace

std::string block_metadata_shm_name(uint32_t publisher_pid, uint32_t block_id) {
  std::ostringstream out;
  out << "/ros2_cuda_ipc_" << publisher_pid << "_" << block_id;
  return out.str();
}

BufferMetadata::~BufferMetadata() {
  if (addr_ != nullptr && mapped_size_ != 0) {
    munmap(addr_, mapped_size_);
  }
}

std::shared_ptr<BufferMetadata> BufferMetadata::create(
    const std::string& shm_name, uint64_t initial_uid) {
  return map_created(shm_name, initial_uid);
}

std::shared_ptr<BufferMetadata> BufferMetadata::attach(
    const std::string& shm_name) {
  return map_attached(shm_name);
}

}  // namespace ros2_cuda_ipc_core::buffer_metadata
