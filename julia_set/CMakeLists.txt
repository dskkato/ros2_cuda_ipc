cmake_minimum_required(VERSION 3.16)
project(julia_set)

enable_language(CUDA)

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
find_package(sensor_msgs REQUIRED)
find_package(ros2_cuda_ipc_core REQUIRED)
find_package(ros2_cuda_ipc_msgs REQUIRED)
find_package(CUDAToolkit REQUIRED)

add_library(julia_set_helpers
  src/julia_publisher_helper.cu
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

ament_target_dependencies(julia_set_helpers
  rclcpp
  sensor_msgs
  ros2_cuda_ipc_core
  ros2_cuda_ipc_msgs
)

add_executable(julia_set_publisher src/julia_set_publisher.cpp)
target_link_libraries(julia_set_publisher
  julia_set_helpers
)

add_executable(julia_set_subscriber src/julia_set_subscriber.cpp)
target_link_libraries(julia_set_subscriber
  julia_set_helpers
)

ament_target_dependencies(julia_set_publisher
  rclcpp
  sensor_msgs
  ros2_cuda_ipc_core
  ros2_cuda_ipc_msgs
)

ament_target_dependencies(julia_set_subscriber
  rclcpp
  sensor_msgs
  ros2_cuda_ipc_core
  ros2_cuda_ipc_msgs
)

install(TARGETS
  julia_set_helpers
  julia_set_publisher
  julia_set_subscriber
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
