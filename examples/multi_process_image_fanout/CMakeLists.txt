cmake_minimum_required(VERSION 3.16)
project(multi_process_image_fanout)

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
find_package(std_msgs REQUIRED)
find_package(ros2_cuda_ipc_core REQUIRED)
find_package(ros2_cuda_ipc_msgs REQUIRED)
find_package(CUDAToolkit REQUIRED)

add_library(${PROJECT_NAME}_helpers
  src/image_publisher_helper.cpp
  src/cuda/kernels.cu
)

set_target_properties(${PROJECT_NAME}_helpers PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
  POSITION_INDEPENDENT_CODE ON
)

target_include_directories(${PROJECT_NAME}_helpers
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_helpers
  PUBLIC
    rclcpp::rclcpp
    ros2_cuda_ipc_core::ros2_cuda_ipc_core
    ${ros2_cuda_ipc_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    CUDA::cudart
)

add_executable(gpu_image_publisher
  src/gpu_image_publisher.cpp
)

target_link_libraries(gpu_image_publisher
  PRIVATE
    ${PROJECT_NAME}_helpers
    ${ros2_cuda_ipc_msgs_TARGETS}
)

add_executable(preview_node
  src/preview_node.cpp
)

target_link_libraries(preview_node
  PRIVATE
    ${PROJECT_NAME}_helpers
    ${ros2_cuda_ipc_msgs_TARGETS}
)

add_executable(encoder_like_node
  src/encoder_like_node.cpp
)

target_link_libraries(encoder_like_node
  PRIVATE
    ${PROJECT_NAME}_helpers
    ${ros2_cuda_ipc_msgs_TARGETS}
)

install(TARGETS
  ${PROJECT_NAME}_helpers
  gpu_image_publisher
  preview_node
  encoder_like_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY doc/
  DESTINATION share/${PROJECT_NAME}/doc
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_kernels test/test_kernels.cpp)
  target_link_libraries(test_kernels
    ${PROJECT_NAME}_helpers
    CUDA::cudart
  )
endif()

ament_export_dependencies(
  rclcpp
  sensor_msgs
  std_msgs
  ros2_cuda_ipc_core
  ros2_cuda_ipc_msgs
)

ament_package()
