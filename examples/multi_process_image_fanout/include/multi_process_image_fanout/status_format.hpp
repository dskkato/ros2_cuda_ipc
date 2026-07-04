// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

namespace multi_process_image_fanout {

inline std::string quote_json_string(const std::string& value) {
  std::ostringstream stream;
  stream << '"';
  for (const char ch : value) {
    if (ch == '"' || ch == '\\') {
      stream << '\\';
    }
    stream << ch;
  }
  stream << '"';
  return stream.str();
}

inline std::string format_encoder_status(uint64_t received_count,
                                         int64_t stamp_ns, int output_width,
                                         int output_height, uint64_t checksum,
                                         float kernel_ms) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(4);
  stream << '{' << "\"node\":" << quote_json_string("encoder_like") << ','
         << "\"received\":" << received_count << ','
         << "\"stamp_ns\":" << stamp_ns << ','
         << "\"output_width\":" << output_width << ','
         << "\"output_height\":" << output_height << ','
         << "\"checksum\":" << checksum << ',' << "\"kernel_ms\":" << kernel_ms
         << '}';
  return stream.str();
}

inline std::string format_inference_status(uint64_t received_count,
                                           int64_t stamp_ns, float mean,
                                           float min, float max,
                                           uint64_t checksum, float kernel_ms) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(4);
  stream << '{' << "\"node\":" << quote_json_string("inference_like") << ','
         << "\"received\":" << received_count << ','
         << "\"stamp_ns\":" << stamp_ns << ',' << "\"mean\":" << mean << ','
         << "\"min\":" << min << ',' << "\"max\":" << max << ','
         << "\"checksum\":" << checksum << ',' << "\"kernel_ms\":" << kernel_ms
         << '}';
  return stream.str();
}

}  // namespace multi_process_image_fanout
