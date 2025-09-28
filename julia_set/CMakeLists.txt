cmake_minimum_required(VERSION 3.16)
project(julia_set)

enable_language(CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros2_cuda_ipc_core REQUIRED)
find_package(ros2_cuda_ipc_msgs REQUIRED)
find_package(CUDAToolkit REQUIRED)

# julia set kernels
add_library(julia_set_helpers
  src/cuda/julia_set.cu
)

set_target_properties(julia_set_helpers PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
  POSITION_INDEPENDENT_CODE ON
)

target_include_directories(julia_set_helpers
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${CUDAToolkit_INCLUDE_DIRS}
)

target_link_libraries(julia_set_helpers
  CUDA::cudart
)

# executables
## Julia Set Publisher
add_executable(julia_set_publisher
  src/julia_set_publisher.cpp
  src/julia_publisher_helper.cpp
)
target_link_libraries(julia_set_publisher
  julia_set_helpers
)

## Colorize Node
add_executable(colorize_node src/colorize_node.cpp)
target_link_libraries(colorize_node
  julia_set_helpers
)

## GPU Image Transport
add_executable(gpu_image_transport src/gpu_image_transport.cpp)
target_link_libraries(gpu_image_transport
  julia_set_helpers
)

ament_target_dependencies(julia_set_publisher
  rclcpp
  sensor_msgs
  ros2_cuda_ipc_core
  ros2_cuda_ipc_msgs
)

ament_target_dependencies(colorize_node
  rclcpp
  ros2_cuda_ipc_core
  ros2_cuda_ipc_msgs
)

ament_target_dependencies(gpu_image_transport
  rclcpp
  sensor_msgs
  ros2_cuda_ipc_core
  ros2_cuda_ipc_msgs
)

install(TARGETS
  julia_set_helpers
  julia_set_publisher
  colorize_node
  gpu_image_transport
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_dependencies(rclcpp sensor_msgs ros2_cuda_ipc_core ros2_cuda_ipc_msgs)

ament_package()
