cmake_minimum_required(VERSION 3.5)
project(ros2_cuda_ipc_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_cuda_ipc_msgs REQUIRED)

# Optionally enable CUDA support
option(ROS2_CUDA_IPC_ENABLE_CUDA "Build with CUDA support" OFF)

set(SOURCES
  src/gpu_buffer_pool.cpp
)

if(ROS2_CUDA_IPC_ENABLE_CUDA)
  message(STATUS "ros2_cuda_ipc_core: CUDA support ENABLED")
  enable_language(CUDA)
  # Prefer modern CMake CUDA language instead of FindCUDA
  set(CMAKE_CUDA_STANDARD 14)
  set(CMAKE_CUDA_STANDARD_REQUIRED ON)
  list(APPEND SOURCES src/cuda_support.cu)
  find_package(CUDAToolkit QUIET)
else()
  message(STATUS "ros2_cuda_ipc_core: CUDA support DISABLED")
  list(APPEND SOURCES src/cuda_support_stub.cpp)
endif()

add_library(${PROJECT_NAME} ${SOURCES})
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} rclcpp ros2_cuda_ipc_msgs)

if(ROS2_CUDA_IPC_ENABLE_CUDA)
  target_compile_definitions(${PROJECT_NAME} PUBLIC ROS2_CUDA_IPC_WITH_CUDA=1)
  if(CUDAToolkit_FOUND AND TARGET CUDA::cudart)
    target_link_libraries(${PROJECT_NAME} CUDA::cudart)
    # Ensure consumers also find CUDAToolkit when importing this target
    set(_ros2_cuda_ipc_export_cuda TRUE)
  else()
    set(_ros2_cuda_ipc_export_cuda FALSE)
  endif()
endif()

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gpu_buffer_pool test/test_gpu_buffer_pool.cpp)
  if(TARGET test_gpu_buffer_pool)
    target_link_libraries(test_gpu_buffer_pool ${PROJECT_NAME})
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp ros2_cuda_ipc_msgs)
if(DEFINED _ros2_cuda_ipc_export_cuda AND _ros2_cuda_ipc_export_cuda)
  ament_export_dependencies(CUDAToolkit)
endif()
ament_package()
