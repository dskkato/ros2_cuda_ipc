// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "ros2_cuda_ipc_core/detail/cuda_driver_context.hpp"

namespace ros2_cuda_ipc_core::detail {
namespace {

bool driver_available(int* device_count = nullptr) {
  auto result = CudaDriver::initialize();
  if (!result) {
    return false;
  }
  int count = 0;
  if (cuDeviceGetCount(&count) != CUDA_SUCCESS) {
    return false;
  }
  if (device_count != nullptr) {
    *device_count = count;
  }
  return count > 0;
}

std::shared_ptr<CudaDeviceContext> primary_context_or_skip(int device_id) {
  auto result = CudaDeviceContext::retain_primary(device_id);
  if (!result) {
    ADD_FAILURE() << "Unable to retain CUDA primary context: "
                  << result.error().to_string();
    return nullptr;
  }
  return std::move(result).value();
}

TEST(CudaDriverContext, ErrorFormattingUsesDriverApi) {
  const CudaDriverError error(CUDA_ERROR_INVALID_CONTEXT);
  EXPECT_EQ(error.code(), CUDA_ERROR_INVALID_CONTEXT);
  EXPECT_EQ(error.name(), "CUDA_ERROR_INVALID_CONTEXT");
  EXPECT_FALSE(error.description().empty());
  EXPECT_NE(error.to_string().find("CUDA_ERROR_INVALID_CONTEXT"),
            std::string::npos);

  const auto unknown_code = static_cast<CUresult>(999999);
  const CudaDriverError unknown_error(unknown_code);
  const auto numeric_code = std::to_string(static_cast<int>(unknown_code));
  EXPECT_NE(unknown_error.name().find(numeric_code), std::string::npos);
  EXPECT_NE(unknown_error.description().find(numeric_code), std::string::npos);
}

TEST(CudaDriverContext, InitializationIsSafeForConcurrentCallers) {
  constexpr int kThreadCount = 8;
  std::atomic<bool> start{false};
  std::vector<std::atomic<bool>> succeeded(kThreadCount);
  std::vector<std::string> errors(kThreadCount);
  std::vector<std::thread> threads;
  threads.reserve(kThreadCount);

  for (int i = 0; i < kThreadCount; ++i) {
    threads.emplace_back([&, i]() {
      while (!start.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
      auto result = CudaDriver::initialize();
      succeeded[i].store(static_cast<bool>(result), std::memory_order_release);
      if (!result) {
        errors[i] = result.error().to_string();
      }
    });
  }
  start.store(true, std::memory_order_release);
  for (auto& thread : threads) {
    thread.join();
  }

  for (int i = 1; i < kThreadCount; ++i) {
    const bool first_succeeded = succeeded[0].load(std::memory_order_acquire);
    const bool current_succeeded = succeeded[i].load(std::memory_order_acquire);
    EXPECT_EQ(current_succeeded, first_succeeded);
    if (!first_succeeded) {
      EXPECT_EQ(errors[i], errors[0]);
    }
  }
}

TEST(CudaDriverContext, RetainsSharedPrimaryContext) {
  if (!driver_available()) {
    GTEST_SKIP() << "No CUDA-capable device is available";
  }

  auto first = primary_context_or_skip(0);
  ASSERT_NE(first, nullptr);
  auto second = primary_context_or_skip(0);
  ASSERT_NE(second, nullptr);
  EXPECT_EQ(first->device_id(), 0);
  EXPECT_EQ(first->device(), second->device());
  EXPECT_EQ(first->context(), second->context());

  first.reset();
  auto guard_result = second->push_current();
  ASSERT_TRUE(guard_result) << guard_result.error().to_string();
  auto guard = std::move(guard_result).value();
  CUcontext current = nullptr;
  ASSERT_EQ(cuCtxGetCurrent(&current), CUDA_SUCCESS);
  EXPECT_EQ(current, second->context());
}

TEST(CudaDriverContext, InvalidDeviceOrdinalReturnsControlledError) {
  if (!driver_available()) {
    GTEST_SKIP() << "No CUDA-capable device is available";
  }

  int device_count = 0;
  ASSERT_EQ(cuDeviceGetCount(&device_count), CUDA_SUCCESS);
  auto result = CudaDeviceContext::retain_primary(device_count);
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code(), CUDA_ERROR_INVALID_DEVICE);
}

TEST(CudaDriverContext, NullContextGuardReturnsControlledError) {
  if (!driver_available()) {
    GTEST_SKIP() << "No CUDA-capable device is available";
  }
  auto result = CudaContextGuard::push(nullptr);
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code(), CUDA_ERROR_INVALID_CONTEXT);
}

TEST(CudaDriverContext, RestoresPreviousContextAndSupportsNestedGuards) {
  if (!driver_available()) {
    GTEST_SKIP() << "No CUDA-capable device is available";
  }

  auto context = primary_context_or_skip(0);
  ASSERT_NE(context, nullptr);

  CUcontext original = nullptr;
  ASSERT_EQ(cuCtxGetCurrent(&original), CUDA_SUCCESS);
  ASSERT_EQ(cuCtxSetCurrent(nullptr), CUDA_SUCCESS);

  {
    auto outer_result = context->push_current();
    ASSERT_TRUE(outer_result) << outer_result.error().to_string();
    auto outer = std::move(outer_result).value();

    CUcontext current = nullptr;
    ASSERT_EQ(cuCtxGetCurrent(&current), CUDA_SUCCESS);
    EXPECT_EQ(current, context->context());

    {
      auto inner_result = context->push_current();
      ASSERT_TRUE(inner_result) << inner_result.error().to_string();
      auto inner = std::move(inner_result).value();
      ASSERT_EQ(cuCtxGetCurrent(&current), CUDA_SUCCESS);
      EXPECT_EQ(current, context->context());
    }

    ASSERT_EQ(cuCtxGetCurrent(&current), CUDA_SUCCESS);
    EXPECT_EQ(current, context->context());
  }

  CUcontext restored = nullptr;
  ASSERT_EQ(cuCtxGetCurrent(&restored), CUDA_SUCCESS);
  EXPECT_EQ(restored, original);
  ASSERT_EQ(cuCtxSetCurrent(original), CUDA_SUCCESS);
}

TEST(CudaDriverContext, MoveConstructionDoesNotDoublePop) {
  if (!driver_available()) {
    GTEST_SKIP() << "No CUDA-capable device is available";
  }

  auto context = primary_context_or_skip(0);
  ASSERT_NE(context, nullptr);
  CUcontext original = nullptr;
  ASSERT_EQ(cuCtxGetCurrent(&original), CUDA_SUCCESS);
  ASSERT_EQ(cuCtxSetCurrent(nullptr), CUDA_SUCCESS);

  {
    auto result = context->push_current();
    ASSERT_TRUE(result) << result.error().to_string();
    auto first = std::move(result).value();
    auto second = std::move(first);
    CUcontext current = nullptr;
    ASSERT_EQ(cuCtxGetCurrent(&current), CUDA_SUCCESS);
    EXPECT_EQ(current, context->context());
    (void)second;
  }

  CUcontext restored = nullptr;
  ASSERT_EQ(cuCtxGetCurrent(&restored), CUDA_SUCCESS);
  EXPECT_EQ(restored, original);
  ASSERT_EQ(cuCtxSetCurrent(original), CUDA_SUCCESS);
}

TEST(CudaDriverContext, PushPopIsThreadLocal) {
  if (!driver_available()) {
    GTEST_SKIP() << "No CUDA-capable device is available";
  }

  auto context = primary_context_or_skip(0);
  ASSERT_NE(context, nullptr);

  constexpr int kThreadCount = 4;
  std::atomic<bool> start{false};
  std::mutex mutex;
  std::vector<std::string> failures;
  std::vector<std::thread> threads;
  threads.reserve(kThreadCount);
  for (int i = 0; i < kThreadCount; ++i) {
    threads.emplace_back([&]() {
      CUcontext original = nullptr;
      if (cuCtxGetCurrent(&original) != CUDA_SUCCESS) {
        std::lock_guard<std::mutex> lock(mutex);
        failures.emplace_back("cuCtxGetCurrent before guard failed");
        return;
      }
      while (!start.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
      auto result = context->push_current();
      if (!result) {
        std::lock_guard<std::mutex> lock(mutex);
        failures.emplace_back(result.error().to_string());
        return;
      }
      auto guard = std::move(result).value();
      CUcontext current = nullptr;
      if (cuCtxGetCurrent(&current) != CUDA_SUCCESS ||
          current != context->context()) {
        std::lock_guard<std::mutex> lock(mutex);
        failures.emplace_back("target context was not current");
      }
      (void)guard;
      // The guard is destroyed on this same thread before checking restore.
      // Its scope ends at the end of this block.
      {
        auto empty_scope = std::move(guard);
        (void)empty_scope;
      }
      if (cuCtxGetCurrent(&current) != CUDA_SUCCESS || current != original) {
        std::lock_guard<std::mutex> lock(mutex);
        failures.emplace_back("previous context was not restored");
      }
    });
  }
  start.store(true, std::memory_order_release);
  for (auto& thread : threads) {
    thread.join();
  }
  EXPECT_TRUE(failures.empty()) << (failures.empty() ? "" : failures.front());
}

TEST(CudaDriverContext, MultipleDevicesHaveIndependentPrimaryContexts) {
  int device_count = 0;
  if (!driver_available(&device_count)) {
    GTEST_SKIP() << "No CUDA-capable device is available";
  }
  if (device_count < 2) {
    GTEST_SKIP() << "Fewer than two CUDA devices are available";
  }

  auto first = primary_context_or_skip(0);
  auto second = primary_context_or_skip(1);
  ASSERT_NE(first, nullptr);
  ASSERT_NE(second, nullptr);
  EXPECT_NE(first->context(), second->context());

  CUcontext original = nullptr;
  ASSERT_EQ(cuCtxGetCurrent(&original), CUDA_SUCCESS);
  ASSERT_EQ(cuCtxSetCurrent(nullptr), CUDA_SUCCESS);
  {
    auto first_guard_result = first->push_current();
    ASSERT_TRUE(first_guard_result) << first_guard_result.error().to_string();
    auto first_guard = std::move(first_guard_result).value();
    CUdevice current_device = 0;
    ASSERT_EQ(cuCtxGetDevice(&current_device), CUDA_SUCCESS);
    EXPECT_EQ(current_device, first->device());

    {
      auto second_guard_result = second->push_current();
      ASSERT_TRUE(second_guard_result)
          << second_guard_result.error().to_string();
      auto second_guard = std::move(second_guard_result).value();
      ASSERT_EQ(cuCtxGetDevice(&current_device), CUDA_SUCCESS);
      EXPECT_EQ(current_device, second->device());
    }

    ASSERT_EQ(cuCtxGetDevice(&current_device), CUDA_SUCCESS);
    EXPECT_EQ(current_device, first->device());
  }
  CUcontext restored = nullptr;
  ASSERT_EQ(cuCtxGetCurrent(&restored), CUDA_SUCCESS);
  EXPECT_EQ(restored, original);
  ASSERT_EQ(cuCtxSetCurrent(original), CUDA_SUCCESS);
}

TEST(CudaDriverContext, RuntimeFirstUsesTheSamePrimaryContext) {
  if (!driver_available()) {
    GTEST_SKIP() << "No CUDA-capable device is available";
  }

  ASSERT_EQ(cudaSetDevice(0), cudaSuccess);
  ASSERT_EQ(cudaFree(nullptr), cudaSuccess);
  CUcontext runtime_context = nullptr;
  ASSERT_EQ(cuCtxGetCurrent(&runtime_context), CUDA_SUCCESS);

  auto context = primary_context_or_skip(0);
  ASSERT_NE(context, nullptr);
  EXPECT_EQ(context->context(), runtime_context);
  {
    auto result = context->push_current();
    ASSERT_TRUE(result) << result.error().to_string();
    auto guard = std::move(result).value();
    CUcontext current = nullptr;
    ASSERT_EQ(cuCtxGetCurrent(&current), CUDA_SUCCESS);
    EXPECT_EQ(current, runtime_context);
  }

  int actual_device = -1;
  EXPECT_EQ(cudaGetDevice(&actual_device), cudaSuccess);
  EXPECT_EQ(actual_device, 0);
}

TEST(CudaDriverContext, DriverFirstCoexistsWithRuntimeApi) {
  if (!driver_available()) {
    GTEST_SKIP() << "No CUDA-capable device is available";
  }

  auto context = primary_context_or_skip(0);
  ASSERT_NE(context, nullptr);
  {
    auto result = context->push_current();
    ASSERT_TRUE(result) << result.error().to_string();
    auto guard = std::move(result).value();
    int actual_device = -1;
    EXPECT_EQ(cudaGetDevice(&actual_device), cudaSuccess);
    EXPECT_EQ(actual_device, 0);
  }

  int actual_device = -1;
  EXPECT_EQ(cudaGetDevice(&actual_device), cudaSuccess);
  EXPECT_EQ(actual_device, 0);
}

}  // namespace
}  // namespace ros2_cuda_ipc_core::detail
