#include <pybind11/pybind11.h>

namespace py = pybind11;

// forward declaration of bindings defined in bindings.cpp
void init_bindings(py::module_ &m);

PYBIND11_MODULE(_cuda_ipc, m) { init_bindings(m); }
