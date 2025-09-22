#pragma once

#include <cuda_runtime_api.h>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "ros2_cuda_ipc_core/pointcloud2_view.hpp"

namespace sample_nodes {

class GpuPointCloudPublisherHelper {
 public:
  struct Config {
    std::string shm_name = "/ros2_cuda_ipc_demo_pc";
    uint32_t width = 1024;
    uint32_t height = 1;
    std::size_t slot_count = 4;
    int device_index = 0;
    bool is_dense = true;
  };

  explicit GpuPointCloudPublisherHelper(const Config &config);
  ~GpuPointCloudPublisherHelper();

  GpuPointCloudPublisherHelper(const GpuPointCloudPublisherHelper &) = delete;
  GpuPointCloudPublisherHelper &operator=(
      const GpuPointCloudPublisherHelper &) = delete;
  GpuPointCloudPublisherHelper(GpuPointCloudPublisherHelper &&) = delete;
  GpuPointCloudPublisherHelper &operator=(GpuPointCloudPublisherHelper &&) =
      delete;

  std::optional<ros2_cuda_ipc_core::PointCloud2View> produce(
      float value, const std::string &frame_id);
  uint64_t cloud_size_bytes() const noexcept { return cloud_size_bytes_; }

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
  uint64_t cloud_size_bytes_ = 0;
  uint32_t next_slot_ = 0;
  uint32_t point_step_ = 0;
  std::vector<ros2_cuda_ipc_core::PointCloud2View::Field> fields_;

  void initialise_shm();
  void allocate_slots();
  void destroy_slots() noexcept;
};

}  // namespace sample_nodes
