// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "gpu_image_transport/gpu_image_transport_base.hpp"

#include <functional>
#include <stdexcept>

namespace gpu_image_transport {

using ros2_cuda_ipc_core::NvtxScopedRange;

GpuImageTransportNodeBase::GpuImageTransportNodeBase(
    const std::string& node_name, const rclcpp::NodeOptions& options)
    : rclcpp::Node(node_name, options),
      input_topic_(
          declare_parameter<std::string>("input_topic_name", "image_gpu")),
      output_topic_(declare_parameter<std::string>("cpu_topic_name", "image")) {
  const cudaError_t err =
      cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking);
  if (err != cudaSuccess) {
    throw std::runtime_error("cudaStreamCreateWithFlags failed: " +
                             cuda_error_to_string(err));
  }

  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.use_intra_process_comm =
      rclcpp::IntraProcessSetting::Disable;

  subscription_ = create_subscription<ros2_cuda_ipc_core::ImageView>(
      input_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      std::bind(&GpuImageTransportNodeBase::on_image, this,
                std::placeholders::_1),
      subscription_options);
}

GpuImageTransportNodeBase::~GpuImageTransportNodeBase() {
  release_pinned_host();
  if (stream_) {
    const cudaError_t err = cudaStreamDestroy(stream_);
    if (err != cudaSuccess) {
      RCLCPP_ERROR(get_logger(), "cudaStreamDestroy failed: %s",
                   cuda_error_to_string(err).c_str());
    }
    stream_ = nullptr;
  }
}

void GpuImageTransportNodeBase::on_image(
    const ros2_cuda_ipc_core::ImageView& view) {
  NvtxScopedRange callback_range("GpuImageTransportNodeBase::on_image");
  if (!view.core.valid()) {
    RCLCPP_WARN(get_logger(), "Received invalid GPU image view");
    return;
  }

  if (!view.sanity_check()) {
    RCLCPP_WARN(
        get_logger(),
        "Skipping frame: failed sanity check rows=%u cols=%u channels=%u",
        view.rows(), view.cols(), view.channels());
    return;
  }

  cudaError_t err = cudaSuccess;
  {
    NvtxScopedRange wait_range("GpuImageTransportNodeBase::stream_wait_event");
    err = view.enqueue_ready_event(stream_);
  }
  if (err != cudaSuccess) {
    RCLCPP_ERROR(get_logger(), "cudaStreamWaitEvent failed: %s",
                 cuda_error_to_string(err).c_str());
    return;
  }

  const std::uint64_t bytes_to_copy = view.core.byte_size;
  if (bytes_to_copy == 0) {
    return;
  }

  {
    NvtxScopedRange ensure_range(
        "GpuImageTransportNodeBase::ensure_pinned_host");
    const cudaError_t ensure_err = ensure_pinned_capacity(bytes_to_copy);
    if (ensure_err != cudaSuccess) {
      RCLCPP_ERROR(get_logger(), "cudaMallocHost failed: %s",
                   cuda_error_to_string(ensure_err).c_str());
      return;
    }
  }

  {
    NvtxScopedRange memcpy_range("GpuImageTransportNodeBase::cudaMemcpyAsync");
    err = cudaMemcpyAsync(pinned_host_buffer_, view.core.data<uint8_t>(),
                          bytes_to_copy, cudaMemcpyDeviceToHost, stream_);
  }
  if (err != cudaSuccess) {
    RCLCPP_ERROR(get_logger(), "cudaMemcpyAsync failed: %s",
                 cuda_error_to_string(err).c_str());
    return;
  }

  {
    NvtxScopedRange sync_range(
        "GpuImageTransportNodeBase::cudaStreamSynchronize");
    err = cudaStreamSynchronize(stream_);
  }
  if (err != cudaSuccess) {
    RCLCPP_ERROR(get_logger(), "cudaStreamSynchronize failed: %s",
                 cuda_error_to_string(err).c_str());
    return;
  }

  publish_frame(view, bytes_to_copy);
}

cudaError_t GpuImageTransportNodeBase::ensure_pinned_capacity(
    std::size_t bytes) {
  if (bytes == 0) {
    return cudaSuccess;
  }
  if (bytes <= pinned_host_capacity_) {
    return cudaSuccess;
  }
  release_pinned_host();
  const cudaError_t err =
      cudaMallocHost(reinterpret_cast<void**>(&pinned_host_buffer_), bytes);
  if (err == cudaSuccess) {
    pinned_host_capacity_ = bytes;
  }
  return err;
}

void GpuImageTransportNodeBase::release_pinned_host() {
  if (pinned_host_buffer_) {
    const cudaError_t err = cudaFreeHost(pinned_host_buffer_);
    if (err != cudaSuccess) {
      RCLCPP_ERROR(get_logger(), "cudaFreeHost failed: %s",
                   cuda_error_to_string(err).c_str());
    }
    pinned_host_buffer_ = nullptr;
    pinned_host_capacity_ = 0;
  }
}

std::string GpuImageTransportNodeBase::cuda_error_to_string(cudaError_t err) {
  const char* error_string = cudaGetErrorString(err);
  return error_string ? std::string(error_string) : std::string{};
}

}  // namespace gpu_image_transport
