cmake_minimum_required(VERSION 3.16)

# Enable CUDA language support
# Try to find CUDA compiler before declaring project
if(NOT DEFINED CMAKE_CUDA_COMPILER)
  if(EXISTS "/usr/local/cuda/bin/nvcc")
    set(CMAKE_CUDA_COMPILER "/usr/local/cuda/bin/nvcc")
  endif()
endif()

project(ros2_cuda_ipc_test LANGUAGES CXX CUDA)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# CUDA standard
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

# Set CUDA architectures policy to OLD to avoid requiring explicit architectures
# This allows nvcc to use its default architecture selection
cmake_policy(SET CMP0104 OLD)

# Find CUDA
find_package(CUDAToolkit REQUIRED)

# Check if we're building as a ROS2 package or standalone
find_package(ament_cmake QUIET)

if(ament_cmake_FOUND)
  message(STATUS "Building ${PROJECT_NAME} as ROS2 package")
  set(IS_ROS2_BUILD TRUE)
else()
  message(STATUS "Building ${PROJECT_NAME} as standalone CMake project")
  set(IS_ROS2_BUILD FALSE)
endif()

# Define executables for CUDA IPC test (using Runtime API)
add_executable(cuda_ipc_producer src/cuda_ipc_producer.cu)
target_link_libraries(cuda_ipc_producer CUDA::cudart)

add_executable(cuda_ipc_consumer src/cuda_ipc_consumer.cu)
target_link_libraries(cuda_ipc_consumer CUDA::cudart)

# Define executables for VMM-FD test (using Driver API + Runtime API)
add_executable(vmm_fd_producer src/vmm_fd_producer.cu)
target_link_libraries(vmm_fd_producer CUDA::cuda_driver CUDA::cudart)

add_executable(vmm_fd_consumer src/vmm_fd_consumer.cu)
target_link_libraries(vmm_fd_consumer CUDA::cuda_driver CUDA::cudart)

# Installation
if(IS_ROS2_BUILD)
  # ROS2 installation
  install(TARGETS
    cuda_ipc_producer
    cuda_ipc_consumer
    vmm_fd_producer
    vmm_fd_consumer
    DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )

  ament_package()
else()
  # Standalone installation
  install(TARGETS
    cuda_ipc_producer
    cuda_ipc_consumer
    vmm_fd_producer
    vmm_fd_consumer
    DESTINATION bin
  )
endif()
