#include <gtest/gtest.h>

#include "ros2_cuda_ipc_core/cuda_support.hpp"

using ros2_cuda_ipc_core::cuda_is_available;

TEST(CudaSupport, Availability) {
  // Exercise the function; skip if no device present
  if (!cuda_is_available()) {
    GTEST_SKIP() << "CUDA device not available";
  }
  EXPECT_TRUE(cuda_is_available());
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
