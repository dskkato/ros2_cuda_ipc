#pragma once

#include <cuda.h>
#include <cuda_runtime_api.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/logging.hpp>
#include <rclcpp/type_adapter.hpp>
#include <string>
#include <string_view>
#include <unordered_map>

#include "ros2_cuda_ipc_core/buffer_view.hpp"
#include "ros2_cuda_ipc_core/image_view.hpp"
#include "ros2_cuda_ipc_core/lease_handle.hpp"
#include "ros2_cuda_ipc_core/memory_types.hpp"
#include "ros2_cuda_ipc_core/pointcloud2_view.hpp"
#include "ros2_cuda_ipc_msgs/msg/buffer_core.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_image.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/header.hpp"

namespace ros2_cuda_ipc_core {

namespace detail {

struct IpcHandleKey {
  uint8_t backend = 0;
  MemoryHandlePayload mem{};
  std::array<uint8_t, sizeof(cudaIpcEventHandle_t)> evt{};

  bool operator==(const IpcHandleKey &other) const noexcept {
    return backend == other.backend && mem == other.mem && evt == other.evt;
  }
};

struct IpcHandleKeyHash {
  std::size_t operator()(const IpcHandleKey &key) const noexcept {
    std::size_t hash = key.backend;
    for (uint8_t byte : key.mem) {
      hash = hash * 131U + byte;
    }
    for (uint8_t byte : key.evt) {
      hash = hash * 131U + byte;
    }
    return hash;
  }
};

struct CachedIpcHandles {
  void *dev_ptr = nullptr;
  cudaEvent_t event = nullptr;
  CUdeviceptr vmm_address = 0;
  CUmemGenericAllocationHandle vmm_allocation = 0;
  std::size_t allocation_size = 0;
};

inline std::unordered_map<IpcHandleKey, CachedIpcHandles, IpcHandleKeyHash> &
ipc_handle_cache() {
  // Known limitation: cached IPC handles live for the subscriber process
  // lifetime. Subscribers cannot safely track publisher lifecycles, so
  // handles persist until shutdown.
  static std::unordered_map<IpcHandleKey, CachedIpcHandles, IpcHandleKeyHash>
      cache;
  return cache;
}

inline std::mutex &ipc_handle_cache_mutex() {
  static std::mutex mutex;
  return mutex;
}

inline std::string cu_result_to_string(CUresult result) {
  const char *name = nullptr;
  const char *desc = nullptr;
  cuGetErrorName(result, &name);
  cuGetErrorString(result, &desc);
  if (!name) {
    name = "UNKNOWN";
  }
  if (!desc) {
    desc = "unknown";
  }
  return std::string(name) + ": " + desc;
}

inline bool ensure_cuda_driver_initialised(const rclcpp::Logger &logger) {
  static std::once_flag once;
  static CUresult init_status = CUDA_SUCCESS;
  std::call_once(once, [&]() { init_status = cuInit(0); });
  if (init_status != CUDA_SUCCESS) {
    RCLCPP_ERROR(logger, "cuInit failed: %s",
                 cu_result_to_string(init_status).c_str());
    return false;
  }
  return true;
}

inline uint64_t load_u64_le(const uint8_t *data) {
  uint64_t value = 0;
  for (int i = 7; i >= 0; --i) {
    value = (value << 8) | data[i];
  }
  return value;
}

inline void store_u64_le(uint8_t *dest, uint64_t value) {
  for (int i = 0; i < 8; ++i) {
    dest[i] = static_cast<uint8_t>(value & 0xFF);
    value >>= 8;
  }
}

struct VmmPayload {
  std::string uuid;
  std::size_t allocation_size = 0;
};

inline std::optional<VmmPayload> parse_vmm_payload(
    const MemoryHandlePayload &payload, const rclcpp::Logger &logger) {
  std::size_t length = 0;
  while (length < payload.size()) {
    const uint8_t byte = payload[length];
    if (byte == 0) {
      break;
    }
    ++length;
  }
  if (length == 0) {
    RCLCPP_WARN(logger,
                "Received VMM_FD backend payload without UUID bytes set");
    return std::nullopt;
  }
  if (length > payload.size()) {
    length = payload.size() - 1;
  }
  VmmPayload result;
  result.uuid.assign(reinterpret_cast<const char *>(payload.data()), length);
  if (result.uuid.size() >= sizeof(sockaddr_un::sun_path)) {
    RCLCPP_WARN(logger, "UUID %s is too long for AF_UNIX path",
                result.uuid.c_str());
    return std::nullopt;
  }
  if (payload.size() >= 64) {
    result.allocation_size =
        static_cast<std::size_t>(load_u64_le(payload.data() + 56));
  }
  return result;
}

inline std::optional<int> request_fd_from_publisher(
    const std::string &path, const rclcpp::Logger &logger) {
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
    RCLCPP_WARN(logger, "socket(AF_UNIX) failed: %s", strerror(errno));
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
  if (::connect(sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    RCLCPP_WARN(logger, "connect(%s) failed: %s", path.c_str(),
                strerror(errno));
    ::close(sock);
    return std::nullopt;
  }

  char buf = 0;
  struct iovec iov {
    &buf, 1
  };
  alignas(struct cmsghdr) char cmsg_buf[CMSG_SPACE(sizeof(int))];
  struct msghdr msg {};
  msg.msg_iov = &iov;
  msg.msg_iovlen = 1;
  msg.msg_control = cmsg_buf;
  msg.msg_controllen = sizeof(cmsg_buf);
  const ssize_t received = ::recvmsg(sock, &msg, 0);
  if (received <= 0) {
    RCLCPP_WARN(logger, "recvmsg on %s failed: %s", path.c_str(),
                strerror(errno));
    ::close(sock);
    return std::nullopt;
  }

  cmsghdr *cmsg = CMSG_FIRSTHDR(&msg);
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

struct VmmOpenResult {
  void *dev_ptr = nullptr;
  CUdeviceptr address = 0;
  CUmemGenericAllocationHandle allocation = 0;
  std::size_t allocation_size = 0;
};

inline std::optional<VmmOpenResult> open_vmm_allocation(
    const MemoryHandlePayload &payload, uint32_t device_id,
    std::size_t fallback_size, const rclcpp::Logger &logger) {
  auto meta = parse_vmm_payload(payload, logger);
  if (!meta.has_value()) {
    return std::nullopt;
  }
  if (!ensure_cuda_driver_initialised(logger)) {
    return std::nullopt;
  }
  const auto socket_path = build_memory_socket_path(meta->uuid);
  auto fd_opt = request_fd_from_publisher(socket_path, logger);
  if (!fd_opt.has_value()) {
    return std::nullopt;
  }
  const int fd = fd_opt.value();
  CUmemGenericAllocationHandle allocation = 0;
  void *os_handle = reinterpret_cast<void *>(static_cast<intptr_t>(fd));
  CUresult cu_res = cuMemImportFromShareableHandle(
      &allocation, os_handle, CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR);
  ::close(fd);
  if (cu_res != CUDA_SUCCESS) {
    RCLCPP_WARN(logger, "cuMemImportFromShareableHandle failed: %s",
                cu_result_to_string(cu_res).c_str());
    return std::nullopt;
  }

  std::size_t allocation_size =
      meta->allocation_size > 0 ? meta->allocation_size : fallback_size;
  if (allocation_size == 0) {
    RCLCPP_WARN(logger, "VMM payload for uuid %s lacked allocation size",
                meta->uuid.c_str());
    cuMemRelease(allocation);
    return std::nullopt;
  }

  CUdeviceptr address = 0;
  cu_res = cuMemAddressReserve(&address, allocation_size, 0, 0, 0);
  if (cu_res != CUDA_SUCCESS) {
    RCLCPP_WARN(logger, "cuMemAddressReserve failed: %s",
                cu_result_to_string(cu_res).c_str());
    cuMemRelease(allocation);
    return std::nullopt;
  }

  cu_res = cuMemMap(address, allocation_size, 0, allocation, 0);
  if (cu_res != CUDA_SUCCESS) {
    RCLCPP_WARN(logger, "cuMemMap failed: %s",
                cu_result_to_string(cu_res).c_str());
    cuMemAddressFree(address, allocation_size);
    cuMemRelease(allocation);
    return std::nullopt;
  }

  CUmemAccessDesc access_desc{};
  access_desc.location.type = CU_MEM_LOCATION_TYPE_DEVICE;
  access_desc.location.id = static_cast<int>(device_id);
  access_desc.flags = CU_MEM_ACCESS_FLAGS_PROT_READWRITE;
  cu_res = cuMemSetAccess(address, allocation_size, &access_desc, 1);
  if (cu_res != CUDA_SUCCESS) {
    RCLCPP_WARN(logger, "cuMemSetAccess failed: %s",
                cu_result_to_string(cu_res).c_str());
    cuMemUnmap(address, allocation_size);
    cuMemAddressFree(address, allocation_size);
    cuMemRelease(allocation);
    return std::nullopt;
  }

  VmmOpenResult result;
  result.dev_ptr = reinterpret_cast<void *>(address);
  result.address = address;
  result.allocation = allocation;
  result.allocation_size = allocation_size;
  return result;
}

}  // namespace detail

inline cudaIpcMemHandle_t to_cuda_mem_handle(
    const ros2_cuda_ipc_msgs::msg::BufferCore &msg) {
  cudaIpcMemHandle_t handle{};
  std::memcpy(&handle, msg.mem_handle.data(), sizeof(handle));
  return handle;
}

inline cudaIpcEventHandle_t to_cuda_event_handle(
    const ros2_cuda_ipc_msgs::msg::BufferCore &msg) {
  cudaIpcEventHandle_t handle{};
  std::memcpy(&handle, msg.event_handle.data(), sizeof(handle));
  return handle;
}

inline void fill_buffer_core_message(const BufferView &view,
                                     ros2_cuda_ipc_msgs::msg::BufferCore &msg) {
  msg.shm_name = view.shm_name;
  msg.device_id = static_cast<uint32_t>(view.device_id);
  msg.slot_id = view.slot_id;
  msg.generation = view.generation;
  msg.byte_size = view.byte_size;
  msg.backend = ros2_cuda_ipc_core::to_backend_byte(view.backend());
  if (view.handles_ready()) {
    const auto &payload = view.mem_payload();
    std::memcpy(msg.mem_handle.data(), payload.data(), payload.size());
    std::memcpy(msg.event_handle.data(), &view.event_handle(),
                sizeof(cudaIpcEventHandle_t));
  } else {
    std::memset(msg.mem_handle.data(), 0, msg.mem_handle.size());
    std::memset(msg.event_handle.data(), 0, msg.event_handle.size());
  }
}

inline BufferView make_invalid_buffer_view() {
  BufferView view;
  view.reset();
  return view;
}

}  // namespace ros2_cuda_ipc_core

namespace rclcpp {

template <>
struct TypeAdapter<ros2_cuda_ipc_core::BufferView,
                   ros2_cuda_ipc_msgs::msg::BufferCore> {
  using is_specialized = std::true_type;
  using custom_type = ros2_cuda_ipc_core::BufferView;
  using ros_message_type = ros2_cuda_ipc_msgs::msg::BufferCore;

  static void convert_to_custom(const ros_message_type &msg,
                                custom_type &view) {
    view.reset();

    const auto backend = ros2_cuda_ipc_core::backend_from_byte(
        static_cast<uint8_t>(msg.backend));

    auto lease = ros2_cuda_ipc_core::LeaseHandle::acquire(
        msg.shm_name, msg.slot_id, msg.generation);
    if (!lease.valid()) {
      RCLCPP_WARN(rclcpp::get_logger("ros2_cuda_ipc_core.BufferView"),
                  "Failed to acquire lease shm=%s slot=%u gen=%u",
                  msg.shm_name.c_str(), msg.slot_id, msg.generation);
      view = ros2_cuda_ipc_core::BufferView{};
      return;
    }

    auto lease_ptr =
        std::make_shared<ros2_cuda_ipc_core::LeaseHandle>(std::move(lease));

    cudaIpcEventHandle_t event_handle =
        ros2_cuda_ipc_core::to_cuda_event_handle(msg);

    ros2_cuda_ipc_core::detail::IpcHandleKey key{};
    key.backend = static_cast<uint8_t>(msg.backend);
    key.mem = msg.mem_handle;
    std::memcpy(key.evt.data(), &event_handle, sizeof(event_handle));

    void *dev_ptr = nullptr;
    cudaEvent_t evt = nullptr;

    {
      std::lock_guard<std::mutex> lock(
          ros2_cuda_ipc_core::detail::ipc_handle_cache_mutex());
      auto &cache = ros2_cuda_ipc_core::detail::ipc_handle_cache();
      auto it = cache.find(key);
      if (it != cache.end()) {
        dev_ptr = it->second.dev_ptr;
        evt = it->second.event;
      }
    }

    if (!dev_ptr || !evt) {
      if (backend == ros2_cuda_ipc_core::MemoryBackendKind::CUDA_IPC) {
        cudaIpcMemHandle_t mem_handle =
            ros2_cuda_ipc_core::to_cuda_mem_handle(msg);
        auto err = cudaIpcOpenMemHandle(&dev_ptr, mem_handle,
                                        cudaIpcMemLazyEnablePeerAccess);
        if (err != cudaSuccess) {
          RCLCPP_WARN(rclcpp::get_logger("ros2_cuda_ipc_core.BufferView"),
                      "cudaIpcOpenMemHandle failed: %s",
                      cudaGetErrorString(err));
          view = ros2_cuda_ipc_core::BufferView{};
          return;
        }
        err = cudaIpcOpenEventHandle(&evt, event_handle);
        if (err != cudaSuccess) {
          cudaIpcCloseMemHandle(dev_ptr);
          RCLCPP_WARN(rclcpp::get_logger("ros2_cuda_ipc_core.BufferView"),
                      "cudaIpcOpenEventHandle failed: %s",
                      cudaGetErrorString(err));
          view = ros2_cuda_ipc_core::BufferView{};
          return;
        }
        {
          std::lock_guard<std::mutex> lock(
              ros2_cuda_ipc_core::detail::ipc_handle_cache_mutex());
          auto &cache = ros2_cuda_ipc_core::detail::ipc_handle_cache();
          auto [it, inserted] = cache.emplace(
              key, ros2_cuda_ipc_core::detail::CachedIpcHandles{dev_ptr, evt});
          if (!inserted) {
            cudaIpcCloseMemHandle(dev_ptr);
            cudaEventDestroy(evt);
            dev_ptr = it->second.dev_ptr;
            evt = it->second.event;
          }
        }
      } else if (backend == ros2_cuda_ipc_core::MemoryBackendKind::VMM_FD) {
        auto logger = rclcpp::get_logger("ros2_cuda_ipc_core.BufferView");
        auto vmm_result = ros2_cuda_ipc_core::detail::open_vmm_allocation(
            msg.mem_handle, msg.device_id,
            static_cast<std::size_t>(msg.byte_size), logger);
        if (!vmm_result.has_value()) {
          view = ros2_cuda_ipc_core::BufferView{};
          return;
        }
        dev_ptr = vmm_result->dev_ptr;
        auto err = cudaIpcOpenEventHandle(&evt, event_handle);
        if (err != cudaSuccess) {
          RCLCPP_WARN(logger,
                      "cudaIpcOpenEventHandle failed for VMM backend: %s",
                      cudaGetErrorString(err));
          ros2_cuda_ipc_core::detail::CachedIpcHandles cleanup{
              dev_ptr, nullptr, vmm_result->address, vmm_result->allocation,
              vmm_result->allocation_size};
          if (cleanup.vmm_address) {
            cuMemUnmap(cleanup.vmm_address, cleanup.allocation_size);
            cuMemAddressFree(cleanup.vmm_address, cleanup.allocation_size);
          }
          if (cleanup.vmm_allocation) {
            cuMemRelease(cleanup.vmm_allocation);
          }
          view = ros2_cuda_ipc_core::BufferView{};
          return;
        }

        {
          std::lock_guard<std::mutex> lock(
              ros2_cuda_ipc_core::detail::ipc_handle_cache_mutex());
          auto &cache = ros2_cuda_ipc_core::detail::ipc_handle_cache();
          auto [it, inserted] = cache.emplace(
              key, ros2_cuda_ipc_core::detail::CachedIpcHandles{
                       dev_ptr, evt, vmm_result->address,
                       vmm_result->allocation, vmm_result->allocation_size});
          if (!inserted) {
            // Another thread populated the cache while we were opening.
            cuMemUnmap(vmm_result->address, vmm_result->allocation_size);
            cuMemAddressFree(vmm_result->address, vmm_result->allocation_size);
            cuMemRelease(vmm_result->allocation);
            cudaEventDestroy(evt);
            dev_ptr = it->second.dev_ptr;
            evt = it->second.event;
          }
        }
      }
    }

    custom_type opened;
    opened.dev_ptr = dev_ptr;
    opened.ready_evt = evt;
    opened.device_id = static_cast<int>(msg.device_id);
    opened.byte_size = msg.byte_size;
    opened.slot_id = msg.slot_id;
    opened.generation = msg.generation;
    opened.shm_name = msg.shm_name;
    opened.lease = std::move(lease_ptr);
    opened.set_memory_handles(backend, msg.mem_handle.data(),
                              msg.mem_handle.size(), event_handle);
    view = std::move(opened);
  }

  static void convert_to_ros_message(const custom_type &view,
                                     ros_message_type &msg) {
    fill_buffer_core_message(view, msg);
  }
};

template <>
struct TypeAdapter<ros2_cuda_ipc_core::ImageView,
                   ros2_cuda_ipc_msgs::msg::GpuImage> {
  using is_specialized = std::true_type;
  using custom_type = ros2_cuda_ipc_core::ImageView;
  using ros_message_type = ros2_cuda_ipc_msgs::msg::GpuImage;

  static void convert_to_custom(const ros_message_type &msg,
                                custom_type &view) {
    ros2_cuda_ipc_core::BufferView core;
    TypeAdapter<
        ros2_cuda_ipc_core::BufferView,
        ros2_cuda_ipc_msgs::msg::BufferCore>::convert_to_custom(msg.core, core);
    if (!core.valid()) {
      view = ros2_cuda_ipc_core::ImageView();
      return;
    }

    custom_type converted;
    converted.core = std::move(core);
    converted.dtype = static_cast<ros2_cuda_ipc_core::DType>(msg.dtype);
    std::copy(msg.shape.begin(), msg.shape.end(), converted.shape.begin());
    std::copy(msg.strides.begin(), msg.strides.end(),
              converted.strides.begin());
    converted.encoding = msg.encoding;
    converted.header = msg.header;

    view = std::move(converted);
  }

  static void convert_to_ros_message(const custom_type &view,
                                     ros_message_type &msg) {
    msg.dtype = static_cast<uint8_t>(view.dtype);
    for (size_t i = 0; i < view.shape.size(); ++i) {
      msg.shape[i] = view.shape[i];
      msg.strides[i] = view.strides[i];
    }
    msg.encoding = view.encoding;
    msg.header = view.header;
    TypeAdapter<
        ros2_cuda_ipc_core::BufferView,
        ros2_cuda_ipc_msgs::msg::BufferCore>::convert_to_ros_message(view.core,
                                                                     msg.core);
  }
};

template <>
struct TypeAdapter<ros2_cuda_ipc_core::PointCloud2View,
                   ros2_cuda_ipc_msgs::msg::GpuPointCloud2> {
  using is_specialized = std::true_type;
  using custom_type = ros2_cuda_ipc_core::PointCloud2View;
  using ros_message_type = ros2_cuda_ipc_msgs::msg::GpuPointCloud2;

  static void convert_to_custom(const ros_message_type &msg,
                                custom_type &view) {
    ros2_cuda_ipc_core::BufferView core;
    TypeAdapter<
        ros2_cuda_ipc_core::BufferView,
        ros2_cuda_ipc_msgs::msg::BufferCore>::convert_to_custom(msg.core, core);

    custom_type converted;
    converted.header = msg.header;

    if (!core.valid()) {
      view = std::move(converted);
      return;
    }

    converted.core = std::move(core);
    converted.height = msg.height;
    converted.width = msg.width;
    converted.point_step = msg.point_step;
    converted.row_step = msg.row_step;
    converted.is_dense = msg.is_dense;
    converted.fields.reserve(msg.fields.size());
    for (const auto &field : msg.fields) {
      custom_type::Field f;
      f.name = field.name;
      f.offset = field.offset;
      f.datatype = field.datatype;
      f.count = field.count;
      converted.fields.emplace_back(std::move(f));
    }

    view = std::move(converted);
  }

  static void convert_to_ros_message(const custom_type &view,
                                     ros_message_type &msg) {
    msg.header = view.header;
    msg.height = view.height;
    msg.width = view.width;
    msg.point_step = view.point_step;
    msg.row_step = view.row_step;
    msg.is_dense = view.is_dense;
    msg.fields.clear();
    msg.fields.reserve(view.fields.size());
    for (const auto &field : view.fields) {
      sensor_msgs::msg::PointField f;
      f.name = field.name;
      f.offset = field.offset;
      f.datatype = field.datatype;
      f.count = field.count;
      msg.fields.emplace_back(std::move(f));
    }

    TypeAdapter<
        ros2_cuda_ipc_core::BufferView,
        ros2_cuda_ipc_msgs::msg::BufferCore>::convert_to_ros_message(view.core,
                                                                     msg.core);
  }
};

}  // namespace rclcpp

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
    ros2_cuda_ipc_core::BufferView, ros2_cuda_ipc_msgs::msg::BufferCore);
RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(ros2_cuda_ipc_core::ImageView,
                                             ros2_cuda_ipc_msgs::msg::GpuImage);
RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
    ros2_cuda_ipc_core::PointCloud2View,
    ros2_cuda_ipc_msgs::msg::GpuPointCloud2);
