#pragma once

#include "ros2_cuda_ipc_core/buffer_view_mapper.hpp"
#include "ros2_cuda_ipc_core/image_view.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_image.hpp"

namespace ros2_cuda_ipc_core {

class ImageViewMapper {
 public:
  explicit ImageViewMapper(BufferViewMapper buffer_mapper = BufferViewMapper{});

  ImageView map(const ros2_cuda_ipc_msgs::msg::GpuImage& msg) const;

 private:
  BufferViewMapper buffer_mapper_;
};

ImageView map_image_view(const ros2_cuda_ipc_msgs::msg::GpuImage& msg);

void fill_gpu_image_message(const ImageView& view,
                            ros2_cuda_ipc_msgs::msg::GpuImage& msg);

}  // namespace ros2_cuda_ipc_core
