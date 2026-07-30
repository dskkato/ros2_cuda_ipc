// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda_runtime_api.h>

#include <cstddef>
#include <cstdint>

namespace multi_process_image_fanout {

struct InferenceStats {
  float mean = 0.0f;
  float min = 0.0f;
  float max = 0.0f;
  uint64_t checksum = 0;
};

constexpr uint32_t kDefaultChannels = 4;
constexpr const char* kDefaultEncoding = "rgba8";

// gpu_image_publisher: fills the referenced GPU slot with the animated RGBA
// test pattern consumed by all downstream example nodes.
cudaError_t launch_generate_rgba_pattern_kernel(uint8_t* output, int width,
                                                int height,
                                                uint64_t stride_bytes,
                                                uint64_t frame_index,
                                                cudaStream_t stream);

// encoder_like_node: converts the incoming RGBA image to a half-resolution luma
// plane before the encoder-like checksum step.
cudaError_t launch_rgba_to_luma_downscale2_kernel(
    const uint8_t* input_rgba, uint8_t* output_luma, int input_width,
    int input_height, uint64_t input_stride_bytes, cudaStream_t stream);

// encoder_like_node: reduces the luma plane to a small GPU checksum so the node
// only copies a scalar result back to the CPU.
cudaError_t launch_checksum_u8_kernel(const uint8_t* input,
                                      size_t element_count,
                                      uint64_t* device_checksum,
                                      cudaStream_t stream);

// inference_like_node: converts RGBA8 pixels to normalized grayscale floats as
// a lightweight stand-in for DNN preprocessing.
cudaError_t launch_rgba_to_normalized_gray_kernel(const uint8_t* input_rgba,
                                                  float* output_gray, int width,
                                                  int height,
                                                  uint64_t input_stride_bytes,
                                                  cudaStream_t stream);

// inference_like_node: computes compact feature/stat values on the GPU so only
// the InferenceStats struct is copied to host memory.
cudaError_t launch_inference_stats_kernel(const float* input,
                                          size_t element_count,
                                          InferenceStats* device_stats,
                                          cudaStream_t stream);

}  // namespace multi_process_image_fanout
