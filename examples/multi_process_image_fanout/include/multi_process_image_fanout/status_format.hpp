// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

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

}  // namespace multi_process_image_fanout
