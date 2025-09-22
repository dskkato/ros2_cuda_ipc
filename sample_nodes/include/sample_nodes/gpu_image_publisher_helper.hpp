#pragma once

#include <cuda_runtime_api.h>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "ros2_cuda_ipc_core/image_view.hpp"

namespace sample_nodes {

class GpuImagePublisherHelper {
 public:
  struct Config {
    std::string shm_name = "/ros2_cuda_ipc_demo";
    uint32_t width = 640;
    uint32_t height = 480;
    uint32_t channels = 3;
    ros2_cuda_ipc_core::DType dtype = ros2_cuda_ipc_core::DType::U8;
    std::size_t slot_count = 4;
    int device_index = 0;
  };

  explicit GpuImagePublisherHelper(const Config &config);
  ~GpuImagePublisherHelper();

  GpuImagePublisherHelper(const GpuImagePublisherHelper &) = delete;
  GpuImagePublisherHelper &operator=(const GpuImagePublisherHelper &) = delete;
  GpuImagePublisherHelper(GpuImagePublisherHelper &&) = delete;
  GpuImagePublisherHelper &operator=(GpuImagePublisherHelper &&) = delete;

  std::optional<ros2_cuda_ipc_core::ImageView> produce(
      uint8_t fill_value, const std::string &frame_id);
  uint64_t frame_size_bytes() const noexcept { return frame_size_bytes_; }

 private:
  struct Slot {
    uint32_t index = 0;
    void *device_ptr = nullptr;
    cudaEvent_t event = nullptr;
    cudaIpcMemHandle_t mem_handle{};
    cudaIpcEventHandle_t event_handle{};
    uint32_t generation = 0;
  };

  Config config_;
  std::vector<Slot> slots_;
  cudaStream_t stream_ = nullptr;
  uint64_t frame_size_bytes_ = 0;

  void initialise_shm();
  void allocate_slots();
  void destroy_slots() noexcept;
};

}  // namespace sample_nodes
