// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <cuda.h>
#include <gtest/gtest.h>

#include "ros2_cuda_ipc_core/detail/cuda_driver_context.hpp"
#include "ros2_cuda_ipc_core/detail/cuda_util.hpp"
#include "ros2_cuda_ipc_core/subscriber/buffer_view.hpp"

namespace ros2_cuda_ipc_core {

TEST(DriverApiTest, FormatsDriverErrorsWithoutRuntimeApi) {
  const auto text = detail::cu_result_to_string(CUDA_ERROR_INVALID_VALUE);
  EXPECT_NE(text.find("CUDA_ERROR_INVALID_VALUE"), std::string::npos);
}

TEST(DriverApiTest, PrimaryContextGuardRestoresCurrentContext) {
  const CUresult init = detail::ensure_cuda_driver_initialized();
  if (init != CUDA_SUCCESS) {
    GTEST_SKIP() << "CUDA driver unavailable: "
                 << detail::cu_result_to_string(init);
  }

  CUcontext before = nullptr;
  ASSERT_EQ(cuCtxGetCurrent(&before), CUDA_SUCCESS);
  {
    detail::ScopedPrimaryContext guard(0);
    if (!guard.ok()) {
      GTEST_SKIP() << "CUDA device unavailable: "
                   << detail::cu_result_to_string(guard.status());
    }
    CUcontext inside = nullptr;
    ASSERT_EQ(cuCtxGetCurrent(&inside), CUDA_SUCCESS);
    EXPECT_NE(inside, nullptr);
  }
  CUcontext after = nullptr;
  ASSERT_EQ(cuCtxGetCurrent(&after), CUDA_SUCCESS);
  EXPECT_EQ(after, before);
}

TEST(DriverApiTest, EmptyViewDoesNotNeedRuntimeStreamInterop) {
  subscriber::BufferView view;
  EXPECT_EQ(view.enqueue_ready_event(nullptr), cudaSuccess);
}

}  // namespace ros2_cuda_ipc_core
