// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include "ros2_cuda_ipc_core/publisher/gpu_buffer_pool.hpp"

TEST(GpuBufferPoolTest, RejectsZeroBlockCount) {
  ros2_cuda_ipc_core::publisher::GpuBufferPool pool(0);
  EXPECT_FALSE(pool.initialise(1024, 0));
}
