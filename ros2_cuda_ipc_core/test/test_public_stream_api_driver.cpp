// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <cuda.h>
#include <gtest/gtest.h>

#include <type_traits>
#include <utility>

#include "ros2_cuda_ipc_core/publisher/gpu_buffer_manager.hpp"
#include "ros2_cuda_ipc_core/subscriber/buffer_view.hpp"

namespace {

using ros2_cuda_ipc_core::detail::CudaResult;
using ros2_cuda_ipc_core::publisher::PublishSlot;
using ros2_cuda_ipc_core::subscriber::BufferView;
using BufferDescriptor = ros2_cuda_ipc_core::transport::BufferDescriptor;

using PreparePublish =
    CudaResult<BufferDescriptor> (PublishSlot::*)(CUstream) noexcept;
using EnqueueReadyEvent =
    CudaResult<void> (BufferView::*)(CUstream) const noexcept;

template <typename T, typename = void>
struct HasRecordReady : std::false_type {};

template <typename T>
struct HasRecordReady<
    T, std::void_t<decltype(std::declval<T&>().record_ready(nullptr))>>
    : std::true_type {};

template <typename T, typename = void>
struct HasDescriptor : std::false_type {};

template <typename T>
struct HasDescriptor<T, std::void_t<decltype(std::declval<T&>().descriptor())>>
    : std::true_type {};

template <typename T, typename = void>
struct HasCommitPublish : std::false_type {};

template <typename T>
struct HasCommitPublish<
    T, std::void_t<decltype(std::declval<T&>().commit_publish())>>
    : std::true_type {};

static_assert(
    std::is_same_v<decltype(&PublishSlot::prepare_publish), PreparePublish>);
static_assert(std::is_same_v<decltype(&BufferView::enqueue_ready_event),
                             EnqueueReadyEvent>);
static_assert(!HasRecordReady<PublishSlot>::value);
static_assert(!HasDescriptor<PublishSlot>::value);
static_assert(!HasCommitPublish<PublishSlot>::value);

TEST(PublicStreamApiDriverTest, HeadersExposeDriverStreamOnly) { SUCCEED(); }

}  // namespace
