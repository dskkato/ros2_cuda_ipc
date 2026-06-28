#pragma once

#include <cuda.h>
#include <cuda_runtime_api.h>

#include <algorithm>
#include <cstring>
#include <memory>
#include <mutex>
#include <rclcpp/logging.hpp>
#include <rclcpp/type_adapter.hpp>
#include <string>

#include "ros2_cuda_ipc_core/buffer_view.hpp"
#include "ros2_cuda_ipc_core/detail/type_adapters.hpp"
#include "ros2_cuda_ipc_core/image_view.hpp"
#include "ros2_cuda_ipc_core/lease_handle.hpp"
#include "ros2_cuda_ipc_core/pointcloud2_view.hpp"
#include "ros2_cuda_ipc_msgs/msg/buffer_core.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_image.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/header.hpp"

namespace ros2_cuda_ipc_core {

inline cudaIpcMemHandle_t to_cuda_mem_handle(
    const ros2_cuda_ipc_msgs::msg::BufferCore& msg) {
  cudaIpcMemHandle_t handle{};
  std::memcpy(&handle, msg.mem_handle.data(), sizeof(handle));
  return handle;
}

inline cudaIpcEventHandle_t to_cuda_event_handle(
    const ros2_cuda_ipc_msgs::msg::BufferCore& msg) {
  cudaIpcEventHandle_t handle{};
  std::memcpy(&handle, msg.event_handle.data(), sizeof(handle));
  return handle;
}

inline void fill_buffer_core_message(const BufferView& view,
                                     ros2_cuda_ipc_msgs::msg::BufferCore& msg) {
  msg.shm_name = view.shm_name;
  msg.device_id = static_cast<uint32_t>(view.device_id);
  msg.slot_id = view.slot_id;
  msg.generation = view.generation;
  msg.byte_size = view.byte_size;
  msg.backend = ros2_cuda_ipc_core::to_backend_byte(view.backend());
  if (view.handles_ready()) {
    const auto& payload = view.mem_payload();
    std::memcpy(msg.mem_handle.data(), payload.data(), payload.size());
    std::memcpy(msg.event_handle.data(), &view.event_handle(),
                sizeof(cudaIpcEventHandle_t));
  } else {
    std::memset(msg.mem_handle.data(), 0, msg.mem_handle.size());
    std::memset(msg.event_handle.data(), 0, msg.event_handle.size());
  }
}

}  // namespace ros2_cuda_ipc_core

namespace rclcpp {

template <>
struct TypeAdapter<ros2_cuda_ipc_core::BufferView,
                   ros2_cuda_ipc_msgs::msg::BufferCore> {
  using is_specialized = std::true_type;
  using custom_type = ros2_cuda_ipc_core::BufferView;
  using ros_message_type = ros2_cuda_ipc_msgs::msg::BufferCore;

  static void convert_to_custom(const ros_message_type& msg,
                                custom_type& view) {
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

    void* dev_ptr = nullptr;
    cudaEvent_t evt = nullptr;

    auto close_opened_event = [&evt]() {
      if (evt) {
        cudaEventDestroy(evt);
        evt = nullptr;
      }
    };

    {
      std::lock_guard<std::mutex> lock(
          ros2_cuda_ipc_core::detail::ipc_handle_cache_mutex());
      auto& cache = ros2_cuda_ipc_core::detail::ipc_handle_cache();
      auto it = cache.find(key);
      if (it != cache.end()) {
        dev_ptr = it->second.dev_ptr;
        evt = it->second.event;
      }
    }

    if (!dev_ptr || !evt) {
      // Need to open IPC handles. EventHandle is supported for both backends.
      {
        auto err = cudaIpcOpenEventHandle(&evt, event_handle);
        if (err != cudaSuccess) {
          RCLCPP_WARN(rclcpp::get_logger("ros2_cuda_ipc_core.BufferView"),
                      "cudaIpcOpenEventHandle failed: %s",
                      cudaGetErrorString(err));
          view = ros2_cuda_ipc_core::BufferView{};
          return;
        }
      }

      if (backend == ros2_cuda_ipc_core::MemoryBackendKind::CUDA_IPC) {
        cudaIpcMemHandle_t mem_handle =
            ros2_cuda_ipc_core::to_cuda_mem_handle(msg);
        auto err = cudaIpcOpenMemHandle(&dev_ptr, mem_handle,
                                        cudaIpcMemLazyEnablePeerAccess);
        if (err != cudaSuccess) {
          RCLCPP_WARN(rclcpp::get_logger("ros2_cuda_ipc_core.BufferView"),
                      "cudaIpcOpenMemHandle failed: %s",
                      cudaGetErrorString(err));
          close_opened_event();
          view = ros2_cuda_ipc_core::BufferView{};
          return;
        }
        {
          std::lock_guard<std::mutex> lock(
              ros2_cuda_ipc_core::detail::ipc_handle_cache_mutex());
          auto& cache = ros2_cuda_ipc_core::detail::ipc_handle_cache();
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
          close_opened_event();
          view = ros2_cuda_ipc_core::BufferView{};
          return;
        }

        dev_ptr = vmm_result->dev_ptr;
        {
          std::lock_guard<std::mutex> lock(
              ros2_cuda_ipc_core::detail::ipc_handle_cache_mutex());
          auto& cache = ros2_cuda_ipc_core::detail::ipc_handle_cache();
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
    opened.set_ipc_handles(backend, msg.mem_handle.data(),
                           msg.mem_handle.size(), event_handle);
    view = std::move(opened);
  }

  static void convert_to_ros_message(const custom_type& view,
                                     ros_message_type& msg) {
    fill_buffer_core_message(view, msg);
  }
};

template <>
struct TypeAdapter<ros2_cuda_ipc_core::ImageView,
                   ros2_cuda_ipc_msgs::msg::GpuImage> {
  using is_specialized = std::true_type;
  using custom_type = ros2_cuda_ipc_core::ImageView;
  using ros_message_type = ros2_cuda_ipc_msgs::msg::GpuImage;

  static void convert_to_custom(const ros_message_type& msg,
                                custom_type& view) {
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

  static void convert_to_ros_message(const custom_type& view,
                                     ros_message_type& msg) {
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

  static void convert_to_custom(const ros_message_type& msg,
                                custom_type& view) {
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
    for (const auto& field : msg.fields) {
      custom_type::Field f;
      f.name = field.name;
      f.offset = field.offset;
      f.datatype = field.datatype;
      f.count = field.count;
      converted.fields.emplace_back(std::move(f));
    }

    view = std::move(converted);
  }

  static void convert_to_ros_message(const custom_type& view,
                                     ros_message_type& msg) {
    msg.header = view.header;
    msg.height = view.height;
    msg.width = view.width;
    msg.point_step = view.point_step;
    msg.row_step = view.row_step;
    msg.is_dense = view.is_dense;
    msg.fields.clear();
    msg.fields.reserve(view.fields.size());
    for (const auto& field : view.fields) {
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
