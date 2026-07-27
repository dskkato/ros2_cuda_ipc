// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <cuda.h>
#include <gtest/gtest.h>

#include <type_traits>

#include "ros2_cuda_ipc_core/publisher/gpu_buffer_manager.hpp"
#include "ros2_cuda_ipc_core/subscriber/buffer_view.hpp"

namespace {

using ros2_cuda_ipc_core::detail::CudaResult;
using ros2_cuda_ipc_core::publisher::PublishSlot;
using ros2_cuda_ipc_core::subscriber::BufferView;
using BufferDescriptor = ros2_cuda_ipc_core::transport::BufferDescriptor;

using PublishRecordReady = CudaResult<void> (PublishSlot::*)(CUstream) noexcept;
using PreparePublish =
    CudaResult<BufferDescriptor> (PublishSlot::*)(CUstream) noexcept;
using EnqueueReadyEvent =
    CudaResult<void> (BufferView::*)(CUstream) const noexcept;

static_assert(
    std::is_same_v<decltype(&PublishSlot::prepare_publish), PreparePublish>);
static_assert(
    std::is_same_v<decltype(&PublishSlot::record_ready), PublishRecordReady>);
static_assert(std::is_same_v<decltype(&BufferView::enqueue_ready_event),
                             EnqueueReadyEvent>);

TEST(PublicStreamApiDriverTest, HeadersExposeDriverStreamOnly) { SUCCEED(); }

}  // namespace
