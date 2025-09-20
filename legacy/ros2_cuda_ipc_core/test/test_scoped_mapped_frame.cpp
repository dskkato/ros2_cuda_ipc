#include <gtest/gtest.h>

#include <string>

#include "ros2_cuda_ipc_core/gpu_buffer_mapper.hpp"
#include "ros2_cuda_ipc_core/scoped_mapped_frame.hpp"
#include "ros2_cuda_ipc_core/shm_release.hpp"

using namespace ros2_cuda_ipc_core;

TEST(ScopedMappedFrameTest, DestructorDecrementsShm) {
  GpuBufferMapper mapper;
  const uint32_t slot = 7;
  const uint64_t seq = 99;

  // Prepare SHM with refcnt=1
  std::string name = make_slot_shm_name_with_owner("scoped_test", slot);
  auto created = shm_create_init(name, slot, seq, /*refcnt=*/1);
  ASSERT_TRUE(created.has_value());

  // Construct frame without actual IPC handles or stream; only SHM info.
  {
    ScopedMappedFrame frame(mapper, slot, /*mem_handle=*/nullptr,
                            /*evt_handle=*/nullptr, /*stream=*/nullptr, name,
                            seq, /*sync_on_dtor=*/false);
    // device_ptr() will be nullptr since no mem_handle provided
    EXPECT_EQ(frame.device_ptr(), nullptr);
  }  // dtor should decrement refcnt

  auto ref = shm_read_refcnt(name, slot, seq);
  ASSERT_TRUE(ref.has_value());
  EXPECT_EQ(*ref, 0);

  // Cleanup
  EXPECT_TRUE(shm_unlink(name));
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
