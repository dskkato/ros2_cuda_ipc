#include "sample_nodes/sample_cuda_utils.hpp"

namespace sample_nodes {

// This stub is used when CUDA is not available.
// The function is a no-op but returns true so callers treat it as success.
bool cuda_simulate_work_ms(int /*ms*/, void* /*stream*/) { return true; }

}  // namespace sample_nodes
