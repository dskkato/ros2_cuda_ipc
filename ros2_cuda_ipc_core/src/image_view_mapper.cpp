#include "ros2_cuda_ipc_core/image_view_mapper.hpp"

namespace ros2_cuda_ipc_core {

namespace {

ImageViewMapper& default_image_view_mapper() {
  static ImageViewMapper mapper;
  return mapper;
}

}  // namespace

ImageViewMapper::ImageViewMapper(BufferViewMapper buffer_mapper)
    : buffer_mapper_(std::move(buffer_mapper)) {}

ImageView ImageViewMapper::map(
    const ros2_cuda_ipc_msgs::msg::GpuImage& msg) const {
  BufferView core = buffer_mapper_.map(msg.core);
  if (!core.valid()) {
    return ImageView{};
  }

  ImageView view;
  view.core = std::move(core);
  view.dtype = static_cast<DType>(msg.dtype);
  view.shape = msg.shape;
  view.strides = msg.strides;
  view.encoding = msg.encoding;
  view.header = msg.header;
  return view;
}

ImageView map_image_view(const ros2_cuda_ipc_msgs::msg::GpuImage& msg) {
  return default_image_view_mapper().map(msg);
}

void fill_gpu_image_message(const ImageView& view,
                            ros2_cuda_ipc_msgs::msg::GpuImage& msg) {
  msg.dtype = static_cast<uint8_t>(view.dtype);
  msg.shape = view.shape;
  msg.strides = view.strides;
  msg.encoding = view.encoding;
  msg.header = view.header;
  fill_buffer_core_message(view.core, msg.core);
}

}  // namespace ros2_cuda_ipc_core
