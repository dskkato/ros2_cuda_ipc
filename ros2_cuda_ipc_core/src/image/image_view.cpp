// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/image/image_view.hpp"

#include <limits>

#include "ros2_cuda_ipc_core/detail/image_view_dlpack.hpp"
#include "ros2_cuda_ipc_core/detail/mapped_publication.hpp"
#include "ros2_cuda_ipc_core/detail/read_handle_factory.hpp"

namespace ros2_cuda_ipc_core::image {

ImageView::ImageView() noexcept = default;

ImageView::~ImageView() noexcept = default;

ImageView::ImageView(ImageView&&) noexcept = default;

ImageView& ImageView::operator=(ImageView&&) noexcept = default;

bool ImageView::valid() const noexcept {
  const bool has_mapping =
      core.valid() || (publication_ && publication_->valid());
  return has_mapping && rows() > 0 && cols() > 0;
}

uint32_t ImageView::elem_size_bytes() const noexcept {
  switch (dtype) {
    case DType::U8:
      return 1;
    case DType::U16:
    case DType::F16:
    case DType::S16:
      return 2;
    case DType::F32:
    case DType::S32:
    case DType::U32:
      return 4;
    case DType::F64:
      return 8;
  }
  return 1;
}

bool ImageView::sanity_check() const noexcept {
  if (!valid() || channels() == 0) {
    return false;
  }

  using WideUnsigned = unsigned __int128;
  const WideUnsigned needed =
      (static_cast<WideUnsigned>(rows() - 1) * strideH()) +
      (static_cast<WideUnsigned>(cols() - 1) * strideW()) +
      (static_cast<WideUnsigned>(channels() - 1) * strideC()) +
      elem_size_bytes();
  return needed <= std::numeric_limits<uint64_t>::max() &&
         needed <= detail::DLPackImageView::byte_size(*this);
}

namespace detail {

bool DLPackImageView::bind(ImageView& view, CUstream consumer_stream) {
  if (view.core.valid() || !view.publication_) {
    return false;
  }

  auto read = subscriber::detail::ReadHandleFactory::make_bound(
      *view.publication_, consumer_stream);
  if (!read) {
    return false;
  }

  // make_bound commits the publication only after all fallible setup and the
  // producer wait have succeeded.  Assigning the resulting operation cannot
  // fail, so the ImageView now has the same bound-state invariant as a normal
  // mapper result.
  view.core = std::move(*read);
  return true;
}

void* DLPackImageView::device_ptr(const ImageView& view) noexcept {
  if (view.core.valid()) {
    return view.core.device_ptr();
  }
  return view.publication_ ? view.publication_->device_ptr() : nullptr;
}

std::size_t DLPackImageView::byte_size(const ImageView& view) noexcept {
  if (view.core.valid()) {
    return view.core.byte_size();
  }
  return view.publication_ ? view.publication_->byte_size() : 0;
}

int DLPackImageView::device_id(const ImageView& view) noexcept {
  if (view.core.valid()) {
    return view.core.device_id();
  }
  return view.publication_ ? view.publication_->device_id() : -1;
}

}  // namespace detail

}  // namespace ros2_cuda_ipc_core::image
