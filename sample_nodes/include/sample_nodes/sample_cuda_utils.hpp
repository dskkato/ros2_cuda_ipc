// Simple CUDA demo helpers for sample_nodes (optional CUDA)
#ifndef SAMPLE_NODES_SAMPLE_CUDA_UTILS_HPP_
#define SAMPLE_NODES_SAMPLE_CUDA_UTILS_HPP_

namespace sample_nodes {

// Launch a small GPU workload to simulate processing time on a given stream.
// `stream` is an opaque CUDA stream pointer (nullptr for default stream).
// Returns true if launched successfully; no-op when built without CUDA.
bool cuda_simulate_work_ms(int ms, void* stream);

}  // namespace sample_nodes

#endif  // SAMPLE_NODES_SAMPLE_CUDA_UTILS_HPP_
