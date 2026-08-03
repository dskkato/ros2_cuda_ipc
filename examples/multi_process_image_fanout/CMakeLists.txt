cmake_minimum_required(VERSION 3.16)
project(multi_process_image_fanout)

# Build for the host GPU by default so local binaries contain native cubins and
# do not depend on runtime PTX JIT. CI and package builds should provide an
# explicit CMAKE_CUDA_ARCHITECTURES value because they may not have a GPU.
if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
  if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.24")
    set(CMAKE_CUDA_ARCHITECTURES native)
  else()
    message(WARNING
      "CMake ${CMAKE_VERSION} cannot detect native CUDA architectures; "
      "set CMAKE_CUDA_ARCHITECTURES explicitly for a portable build.")
  endif()
endif()

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
find_package(ros2_cuda_ipc_image REQUIRED)
find_package(ros2_cuda_ipc_msgs REQUIRED)
find_package(CUDAToolkit REQUIRED)

add_library(${PROJECT_NAME}_kernels
  src/cuda/kernels.cu
)

set_target_properties(${PROJECT_NAME}_kernels PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
  POSITION_INDEPENDENT_CODE ON
)

target_include_directories(${PROJECT_NAME}_kernels
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_kernels
  PUBLIC
    rclcpp::rclcpp
    ros2_cuda_ipc_core::ros2_cuda_ipc_core
    ros2_cuda_ipc_image::ros2_cuda_ipc_image
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
    ${PROJECT_NAME}_kernels
    ${ros2_cuda_ipc_msgs_TARGETS}
)

add_executable(preview_node
  src/preview_node.cpp
)

target_link_libraries(preview_node
  PRIVATE
    ${PROJECT_NAME}_kernels
    ${ros2_cuda_ipc_msgs_TARGETS}
)

add_executable(encoder_like_node
  src/encoder_like_node.cpp
)

target_link_libraries(encoder_like_node
  PRIVATE
    ${PROJECT_NAME}_kernels
    ${ros2_cuda_ipc_msgs_TARGETS}
)

add_executable(inference_like_node
  src/inference_like_node.cpp
)

target_link_libraries(inference_like_node
  PRIVATE
    ${PROJECT_NAME}_kernels
    ${ros2_cuda_ipc_msgs_TARGETS}
)

install(TARGETS ${PROJECT_NAME}_kernels
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  gpu_image_publisher
  preview_node
  encoder_like_node
  inference_like_node
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
    ${PROJECT_NAME}_kernels
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
