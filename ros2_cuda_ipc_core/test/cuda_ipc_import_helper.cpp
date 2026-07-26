// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <unistd.h>

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "cuda_ipc_memory_test_protocol.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_cuda_ipc_core/backend/cuda_ipc/memory_importer.hpp"
#include "ros2_cuda_ipc_core/backend/memory_importer.hpp"
#include "ros2_cuda_ipc_msgs/msg/buffer_core.hpp"

namespace {

bool read_all(int fd, void* data, std::size_t size) {
  auto* bytes = static_cast<unsigned char*>(data);
  std::size_t offset = 0;
  while (offset < size) {
    const ssize_t received = ::read(fd, bytes + offset, size - offset);
    if (received > 0) {
      offset += static_cast<std::size_t>(received);
      continue;
    }
    if (received < 0 && errno == EINTR) {
      continue;
    }
    return false;
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    return 2;
  }
  char* end = nullptr;
  const long parsed_fd = std::strtol(argv[1], &end, 10);
  if (end == argv[1] || *end != '\0' || parsed_fd < 0) {
    return 3;
  }
  const int read_fd = static_cast<int>(parsed_fd);

  ros2_cuda_ipc_core::test::CudaIpcMemoryTestPayload payload;
  const bool received = read_all(read_fd, &payload, sizeof(payload));
  ::close(read_fd);
  if (!received || payload.byte_size == 0) {
    return 4;
  }

  ros2_cuda_ipc_msgs::msg::BufferCore msg;
  msg.device_id = payload.device_id;
  msg.byte_size = payload.byte_size;
  msg.backend = ros2_cuda_ipc_msgs::msg::BufferCore::CUDA_IPC;
  msg.mem_handle = payload.memory_handle;
  msg.event_handle = payload.event_handle;

  CUipcEventHandle event_handle{};
  static_assert(sizeof(event_handle) == sizeof(payload.event_handle));
  std::memcpy(&event_handle, payload.event_handle.data(), sizeof(event_handle));

  ros2_cuda_ipc_core::backend::cuda_ipc::MemoryImporter importer;
  auto imported = importer.import(msg, event_handle);
  if (!imported.has_value()) {
    return 5;
  }

  {
    auto guard_result = imported->context->push_current();
    if (!guard_result) {
      ros2_cuda_ipc_core::backend::release_imported_resources(*imported);
      return 6;
    }
    auto guard = std::move(guard_result).value();
    if (cuEventSynchronize(imported->event) != CUDA_SUCCESS) {
      ros2_cuda_ipc_core::backend::release_imported_resources(*imported);
      return 7;
    }
    std::vector<uint8_t> host(payload.byte_size);
    const auto device_ptr = static_cast<CUdeviceptr>(
        reinterpret_cast<uintptr_t>(imported->dev_ptr));
    if (cuMemcpyDtoH(host.data(), device_ptr, host.size()) != CUDA_SUCCESS) {
      ros2_cuda_ipc_core::backend::release_imported_resources(*imported);
      return 8;
    }
    for (const uint8_t value : host) {
      if (value != payload.expected_value) {
        ros2_cuda_ipc_core::backend::release_imported_resources(*imported);
        return 9;
      }
    }
  }

  return ros2_cuda_ipc_core::backend::release_imported_resources(*imported)
             ? 0
             : 10;
}
