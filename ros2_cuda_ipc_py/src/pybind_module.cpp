#include <pybind11/pybind11.h>

namespace py = pybind11;

// forward declaration of bindings defined in bindings.cpp
void init_bindings(py::module_ &m);
// forward declaration of dlpack utilities
void init_dlpack_utils(py::module_ &m);

PYBIND11_MODULE(_cuda_ipc, m) {
  init_bindings(m);
  init_dlpack_utils(m);
}
