#include "julia_set/cuda/cuda_util.hpp"

namespace julia_set {

std::string cuda_error_to_string(cudaError_t err) {
  return std::string(cudaGetErrorName(err)) + ": " + cudaGetErrorString(err);
}

}  // namespace julia_set
