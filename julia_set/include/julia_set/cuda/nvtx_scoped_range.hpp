#pragma once

#include <nvtx3/nvToolsExt.h>

namespace julia_set {

class NvtxScopedRange {
 public:
  explicit NvtxScopedRange(const char *name) : id_(nvtxRangeStartA(name)) {}

  NvtxScopedRange(const NvtxScopedRange &) = delete;
  NvtxScopedRange &operator=(const NvtxScopedRange &) = delete;
  NvtxScopedRange(NvtxScopedRange &&) = delete;
  NvtxScopedRange &operator=(NvtxScopedRange &&) = delete;

  ~NvtxScopedRange() { nvtxRangeEnd(id_); }

 private:
  nvtxRangeId_t id_{};
};

}  // namespace julia_set
