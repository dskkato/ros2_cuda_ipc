cmake_minimum_required(VERSION 3.5)
project(ros2_cuda_ipc_py)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(pybind11 REQUIRED)

ament_python_install_package(${PROJECT_NAME})

pybind11_add_module(_cuda_ipc src/pybind_module.cpp)
install(TARGETS _cuda_ipc DESTINATION lib/${PROJECT_NAME})

ament_package()
