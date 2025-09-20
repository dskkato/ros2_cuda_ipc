#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "ros2_cuda_ipc_core/buffer_view.hpp"

namespace ros2_cuda_ipc_core {

struct PointCloud2View {
  struct Field {
    std::string name;
    uint32_t offset = 0;
    uint8_t datatype = 0;
    uint32_t count = 0;
  };

  struct DeviceField {
    uint32_t offset;
    uint8_t datatype;
    uint32_t count;
  };

  struct DeviceView {
    uint8_t *data;
    int width;
    int height;
    size_t point_step;
    size_t row_step;
    bool is_dense;
    const DeviceField *fields;
    int num_fields;
  };

  BufferView core;
  uint32_t height = 1;
  uint32_t width = 0;
  uint32_t point_step = 0;
  uint32_t row_step = 0;
  bool is_dense = true;
  std::vector<Field> fields;

  int x_off = -1;
  int y_off = -1;
  int z_off = -1;
  int intensity_off = -1;
  int rgb_off = -1;

  PointCloud2View() = default;
  ~PointCloud2View() = default;
  PointCloud2View(PointCloud2View &&) noexcept = default;
  PointCloud2View &operator=(PointCloud2View &&) noexcept = default;
  PointCloud2View(const PointCloud2View &) = delete;
  PointCloud2View &operator=(const PointCloud2View &) = delete;

  size_t num_points() const noexcept {
    return static_cast<size_t>(width) * height;
  }
  cudaError_t wait(cudaStream_t stream) const noexcept {
    return core.wait(stream);
  }
  bool valid() const noexcept { return core.valid() && point_step > 0; }

  DeviceView as_device_view(const DeviceField *device_fields,
                            int n) const noexcept {
    return DeviceView{core.data<uint8_t>(),
                      static_cast<int>(width),
                      static_cast<int>(height),
                      point_step,
                      row_step,
                      is_dense,
                      device_fields,
                      n};
  }
};

}  // namespace ros2_cuda_ipc_core
