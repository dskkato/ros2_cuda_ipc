// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda.h>

#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>

namespace ros2_cuda_ipc_core::detail {

/**
 * @brief A Driver API error with CUDA-provided name and description formatting.
 *
 * This type deliberately uses cuGetErrorName() and cuGetErrorString() rather
 * than their Runtime API equivalents so it can be used by Driver-only code.
 */
class CudaDriverError {
 public:
  explicit CudaDriverError(CUresult result) noexcept : result_(result) {}

  CUresult code() const noexcept { return result_; }

  std::string name() const;
  std::string description() const;
  std::string to_string() const;

 private:
  CUresult result_;
};

/**
 * @brief Small C++17 result type used by the internal Driver API foundation.
 */
template <typename T>
class CudaResult {
 public:
  static CudaResult success(T value) {
    CudaResult result;
    result.value_.emplace(std::move(value));
    return result;
  }

  static CudaResult failure(CudaDriverError error) {
    CudaResult result;
    result.error_.emplace(std::move(error));
    return result;
  }

  explicit operator bool() const noexcept { return value_.has_value(); }

  T& value() & { return value_.value(); }
  const T& value() const& { return value_.value(); }
  T&& value() && { return std::move(value_.value()); }

  CudaDriverError& error() & { return error_.value(); }
  const CudaDriverError& error() const& { return error_.value(); }

 private:
  CudaResult() = default;

  std::optional<T> value_;
  std::optional<CudaDriverError> error_;
};

template <>
class CudaResult<void> {
 public:
  static CudaResult success() { return CudaResult(true, {}); }

  static CudaResult failure(CudaDriverError error) {
    return CudaResult(false, std::move(error));
  }

  explicit operator bool() const noexcept { return success_; }

  const CudaDriverError& error() const& { return error_.value(); }

 private:
  CudaResult(bool success, std::optional<CudaDriverError> error)
      : success_(success), error_(std::move(error)) {}

  bool success_;
  std::optional<CudaDriverError> error_;
};

/**
 * @brief Process-wide, thread-safe CUDA Driver API initialization.
 */
class CudaDriver {
 public:
  static CudaResult<void> initialize();
};

class CudaContextGuard;

/**
 * @brief Ownership of one retained device primary-context reference.
 *
 * The primary context is shared with CUDA Runtime API users. This class owns
 * only its retain reference and never resets the primary context.
 */
class CudaDeviceContext {
 public:
  static CudaResult<std::shared_ptr<CudaDeviceContext>> retain_primary(
      int device_id);

  CudaResult<CudaContextGuard> push_current() const;

  int device_id() const noexcept { return device_id_; }
  CUdevice device() const noexcept { return device_; }
  CUcontext context() const noexcept { return context_; }

  ~CudaDeviceContext() noexcept;

  CudaDeviceContext(const CudaDeviceContext&) = delete;
  CudaDeviceContext& operator=(const CudaDeviceContext&) = delete;
  CudaDeviceContext(CudaDeviceContext&&) = delete;
  CudaDeviceContext& operator=(CudaDeviceContext&&) = delete;

 private:
  CudaDeviceContext(int device_id, CUdevice device, CUcontext context)
      : device_id_(device_id), device_(device), context_(context) {}

  int device_id_;
  CUdevice device_;
  CUcontext context_;
};

/**
 * @brief Scoped activation of a CUDA context on one CPU thread.
 *
 * A guard pushes a context in its constructor and pops it in its destructor.
 * It is intentionally move-only and should be destroyed on the same thread
 * on which it was created because CUDA context stacks are thread-local.
 */
class CudaContextGuard {
 public:
  static CudaResult<CudaContextGuard> push(CUcontext context);

  ~CudaContextGuard() noexcept;

  CudaContextGuard(CudaContextGuard&& other) noexcept;
  CudaContextGuard& operator=(CudaContextGuard&& other) noexcept;

  CudaContextGuard(const CudaContextGuard&) = delete;
  CudaContextGuard& operator=(const CudaContextGuard&) = delete;

 private:
  explicit CudaContextGuard(CUcontext context)
      : context_(context),
        owner_thread_(std::this_thread::get_id()),
        active_(true) {}

  void pop_noexcept() noexcept;

  CUcontext context_{nullptr};
  std::thread::id owner_thread_{};
  bool active_{false};
};

}  // namespace ros2_cuda_ipc_core::detail
