// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda_runtime_api.h>

#include <cstdint>

namespace multi_process_image_fanout {

constexpr uint32_t kDefaultChannels = 4;
constexpr const char* kDefaultEncoding = "rgba8";

cudaError_t launch_generate_rgba_pattern_kernel(uint8_t* output, int width,
                                                int height,
                                                uint64_t stride_bytes,
                                                uint64_t frame_index,
                                                cudaStream_t stream);

}  // namespace multi_process_image_fanout
