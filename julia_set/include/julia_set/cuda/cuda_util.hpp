#pragma once

#include <cuda_runtime_api.h>

#include <string>

namespace julia_set {

std::string cuda_error_to_string(cudaError_t err);

}  // namespace julia_set
