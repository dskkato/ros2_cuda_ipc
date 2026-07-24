// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda_runtime_api.h>

#include <rclcpp/logging.hpp>
#include <stdexcept>
#include <string>

namespace multi_process_image_fanout {

inline std::string cuda_error_to_string(cudaError_t err) {
  return std::string(cudaGetErrorName(err)) + ": " + cudaGetErrorString(err);
}

inline bool log_cuda_error(const rclcpp::Logger& logger, const char* operation,
                           cudaError_t err) {
  if (err == cudaSuccess) {
    return true;
  }

  RCLCPP_ERROR(logger, "%s failed: %s", operation,
               cuda_error_to_string(err).c_str());
  return false;
}

inline void throw_on_cuda_error(cudaError_t err, const char* what) {
  if (err == cudaSuccess) {
    return;
  }

  throw std::runtime_error(std::string(what) + ": " +
                           cuda_error_to_string(err));
}

}  // namespace multi_process_image_fanout
