// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/backend/vmm_fd/memory_importer.hpp"

#include <fcntl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <optional>
#include <string>

#include "rclcpp/logging.hpp"
#include "ros2_cuda_ipc_core/backend/vmm_fd/payload.hpp"
#include "ros2_cuda_ipc_core/detail/cuda_driver_context.hpp"
#include "ros2_cuda_ipc_core/detail/cuda_util.hpp"
#include "ros2_cuda_ipc_core/detail/posix_error.hpp"
#include "ros2_cuda_ipc_core/transport/memory_types.hpp"

namespace ros2_cuda_ipc_core::backend::vmm_fd {

namespace {

std::size_t align_up_size(std::size_t value, std::size_t alignment) {
  if (alignment == 0) {
    return value;
  }
  const std::size_t remainder = value % alignment;
  if (remainder == 0) {
    return value;
  }
  return value + alignment - remainder;
}

std::optional<std::string> parse_vmm_payload(
    const transport::MemoryHandlePayload& payload,
    const rclcpp::Logger& logger) {
  auto uuid = decode_uuid_payload(payload);
  if (!uuid.has_value()) {
    RCLCPP_WARN(logger, "Received invalid VMM_FD payload");
    return std::nullopt;
  }
  return uuid;
}

std::optional<int> request_fd_from_publisher(const std::string& path,
                                             const rclcpp::Logger& logger) {
  int sock = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
  if (sock < 0) {
    if (errno == EINVAL || errno == EPROTOTYPE) {
      sock = ::socket(AF_UNIX, SOCK_STREAM, 0);
      if (sock >= 0) {
        fcntl(sock, F_SETFD, FD_CLOEXEC);
      }
    }
  }
  if (sock < 0) {
    RCLCPP_WARN(logger, "socket(AF_UNIX) failed: %s",
                ros2_cuda_ipc_core::detail::errno_to_string().c_str());
    return std::nullopt;
  }

  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  if (path.size() >= sizeof(addr.sun_path)) {
    RCLCPP_WARN(logger, "Socket path %s is too long", path.c_str());
    ::close(sock);
    return std::nullopt;
  }
  std::strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path) - 1);
  if (::connect(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    RCLCPP_WARN(logger, "connect(%s) failed: %s", path.c_str(),
                ros2_cuda_ipc_core::detail::errno_to_string().c_str());
    ::close(sock);
    return std::nullopt;
  }

  char buf = 0;
  struct iovec iov{&buf, 1};
  alignas(struct cmsghdr) char cmsg_buf[CMSG_SPACE(sizeof(int))];
  struct msghdr msg{};
  msg.msg_iov = &iov;
  msg.msg_iovlen = 1;
  msg.msg_control = cmsg_buf;
  msg.msg_controllen = sizeof(cmsg_buf);
  const ssize_t received = ::recvmsg(sock, &msg, 0);
  if (received <= 0) {
    RCLCPP_WARN(logger, "recvmsg on %s failed: %s", path.c_str(),
                ros2_cuda_ipc_core::detail::errno_to_string().c_str());
    ::close(sock);
    return std::nullopt;
  }

  cmsghdr* cmsg = CMSG_FIRSTHDR(&msg);
  if (!cmsg || cmsg->cmsg_level != SOL_SOCKET ||
      cmsg->cmsg_type != SCM_RIGHTS || cmsg->cmsg_len < CMSG_LEN(sizeof(int))) {
    RCLCPP_WARN(logger, "recvmsg on %s missing SCM_RIGHTS payload",
                path.c_str());
    ::close(sock);
    return std::nullopt;
  }

  int fd = -1;
  std::memcpy(&fd, CMSG_DATA(cmsg), sizeof(int));
  ::close(sock);
  if (fd < 0) {
    RCLCPP_WARN(logger, "recvmsg returned invalid fd for %s", path.c_str());
    return std::nullopt;
  }
  return fd;
}

}  // namespace

std::optional<ImportedMemory> MemoryImporter::import(
    const ros2_cuda_ipc_msgs::msg::BufferCore& msg,
    const cudaIpcEventHandle_t& event_handle,
    const rclcpp::Logger& logger) const {
  const auto meta = parse_vmm_payload(msg.mem_handle, logger);
  if (!meta.has_value()) {
    return std::nullopt;
  }

  const std::string socket_path = build_socket_path(*meta);
  if (socket_path.size() >= sizeof(sockaddr_un::sun_path)) {
    RCLCPP_WARN(logger, "UUID %s is too long for AF_UNIX path",
                socket_path.c_str());
    return std::nullopt;
  }

  const auto fd_opt = request_fd_from_publisher(socket_path, logger);
  if (!fd_opt.has_value()) {
    return std::nullopt;
  }

  ImportedMemory imported;
  imported.device_id = static_cast<int>(msg.device_id);
  detail::ScopedPrimaryContext context(imported.device_id);
  if (!context.ok()) {
    RCLCPP_WARN(logger, "CUDA Driver context setup failed: %s",
                detail::cu_result_to_string(context.status()).c_str());
    ::close(fd_opt.value());
    return std::nullopt;
  }

  void* os_handle =
      reinterpret_cast<void*>(static_cast<intptr_t>(fd_opt.value()));
  CUresult cu_res =
      cuMemImportFromShareableHandle(&imported.vmm_allocation, os_handle,
                                     CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR);
  ::close(fd_opt.value());
  if (cu_res != CUDA_SUCCESS) {
    RCLCPP_WARN(
        logger, "cuMemImportFromShareableHandle failed: %s",
        ros2_cuda_ipc_core::detail::cu_result_to_string(cu_res).c_str());
    return std::nullopt;
  }

  CUmemAllocationProp prop{};
  prop.type = CU_MEM_ALLOCATION_TYPE_PINNED;
  prop.location.type = CU_MEM_LOCATION_TYPE_DEVICE;
  prop.location.id = static_cast<int>(msg.device_id);
  prop.requestedHandleTypes = CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR;
  std::size_t granularity = 0;
  cu_res = cuMemGetAllocationGranularity(&granularity, &prop,
                                         CU_MEM_ALLOC_GRANULARITY_MINIMUM);
  if (cu_res != CUDA_SUCCESS) {
    RCLCPP_WARN(
        logger, "cuMemGetAllocationGranularity failed: %s",
        ros2_cuda_ipc_core::detail::cu_result_to_string(cu_res).c_str());
    cuMemRelease(imported.vmm_allocation);
    return std::nullopt;
  }

  imported.allocation_size =
      align_up_size(static_cast<std::size_t>(msg.byte_size), granularity);
  if (imported.allocation_size == 0) {
    imported.allocation_size = granularity;
  }

  cu_res = cuMemAddressReserve(&imported.vmm_address, imported.allocation_size,
                               0, 0, 0);
  if (cu_res != CUDA_SUCCESS) {
    RCLCPP_WARN(
        logger, "cuMemAddressReserve failed: %s",
        ros2_cuda_ipc_core::detail::cu_result_to_string(cu_res).c_str());
    cuMemRelease(imported.vmm_allocation);
    return std::nullopt;
  }

  cu_res = cuMemMap(imported.vmm_address, imported.allocation_size, 0,
                    imported.vmm_allocation, 0);
  if (cu_res != CUDA_SUCCESS) {
    RCLCPP_WARN(
        logger, "cuMemMap failed: %s",
        ros2_cuda_ipc_core::detail::cu_result_to_string(cu_res).c_str());
    cuMemAddressFree(imported.vmm_address, imported.allocation_size);
    cuMemRelease(imported.vmm_allocation);
    return std::nullopt;
  }

  CUmemAccessDesc access_desc{};
  access_desc.location = prop.location;
  access_desc.flags = CU_MEM_ACCESS_FLAGS_PROT_READWRITE;
  cu_res = cuMemSetAccess(imported.vmm_address, imported.allocation_size,
                          &access_desc, 1);
  if (cu_res != CUDA_SUCCESS) {
    RCLCPP_WARN(
        logger, "cuMemSetAccess failed: %s",
        ros2_cuda_ipc_core::detail::cu_result_to_string(cu_res).c_str());
    cuMemUnmap(imported.vmm_address, imported.allocation_size);
    cuMemAddressFree(imported.vmm_address, imported.allocation_size);
    cuMemRelease(imported.vmm_allocation);
    return std::nullopt;
  }

  CUipcEventHandle driver_event_handle{};
  std::memcpy(&driver_event_handle, &event_handle, sizeof(driver_event_handle));
  CUevent event = nullptr;
  cu_res = cuIpcOpenEventHandle(&event, driver_event_handle);
  if (cu_res != CUDA_SUCCESS) {
    RCLCPP_WARN(logger, "cuIpcOpenEventHandle failed: %s",
                detail::cu_result_to_string(cu_res).c_str());
    cuMemUnmap(imported.vmm_address, imported.allocation_size);
    cuMemAddressFree(imported.vmm_address, imported.allocation_size);
    cuMemRelease(imported.vmm_allocation);
    return std::nullopt;
  }

  imported.event = reinterpret_cast<cudaEvent_t>(event);
  imported.driver_owned = true;
  imported.dev_ptr = reinterpret_cast<void*>(imported.vmm_address);
  return imported;
}

}  // namespace ros2_cuda_ipc_core::backend::vmm_fd
