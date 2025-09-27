cmake_minimum_required(VERSION 3.16)
project(simple_increment)

enable_language(CUDA)

if(POLICY CMP0104)
  cmake_policy(SET CMP0104 OLD)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ros2_cuda_ipc_core REQUIRED)
find_package(ros2_cuda_ipc_msgs REQUIRED)
find_package(CUDAToolkit REQUIRED)

add_library(simple_increment_kernels
  src/cuda/cuda_functions.cu
)

set_target_properties(simple_increment_kernels PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
  POSITION_INDEPENDENT_CODE ON
)

target_include_directories(simple_increment_kernels
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(simple_increment_kernels
  CUDA::cudart
)

ament_target_dependencies(simple_increment_kernels
  rclcpp
  ros2_cuda_ipc_core
  ros2_cuda_ipc_msgs
)

add_library(simple_increment_component SHARED
  src/simple_increment_node.cpp
)

set_target_properties(simple_increment_component PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

target_include_directories(simple_increment_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(simple_increment_component
  simple_increment_kernels
)

ament_target_dependencies(simple_increment_component
  rclcpp
  rclcpp_components
  ros2_cuda_ipc_core
  ros2_cuda_ipc_msgs
)

rclcpp_components_register_nodes(simple_increment_component
  "simple_increment::SimpleIncrementNode")

install(TARGETS
  simple_increment_kernels
  simple_increment_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_dependencies(rclcpp rclcpp_components ros2_cuda_ipc_core ros2_cuda_ipc_msgs)

ament_package()
