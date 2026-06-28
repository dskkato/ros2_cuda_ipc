cmake_minimum_required(VERSION 3.16)
project(ros2_cuda_ipc_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(ros2_cuda_ipc_msgs REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_library(UUID_LIB uuid)

if(NOT UUID_LIB)
  message(FATAL_ERROR "Failed to find libuuid (uuid)")
endif()

add_library(${PROJECT_NAME}
  src/cuda/cuda_util.cpp
  src/cuda/gpu_lease_pool.cpp
  src/cuda/memory_backend_cuda_ipc.cpp
  src/cuda/memory_backend_vmm_fd.cpp
  src/buffer_view.cpp
  src/buffer_view_mapper.cpp
  src/image_view.cpp
  src/image_view_mapper.cpp
  src/ipc_handle_cache.cpp
  src/lease_handle.cpp
  src/backend/backend_importer.cpp
  src/backend/cuda_ipc_importer.cpp
  src/backend/vmm_fd_importer.cpp
  src/pointcloud2_view.cpp
  src/pointcloud2_view_mapper.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rcpputils
  ros2_cuda_ipc_msgs
  sensor_msgs
  std_msgs
)

target_link_libraries(${PROJECT_NAME} CUDA::cudart CUDA::cuda_driver ${UUID_LIB})

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_lease_handle test/test_lease_handle.cpp)
  target_link_libraries(test_lease_handle ${PROJECT_NAME})
  ament_add_gtest(test_buffer_view_mapper test/test_buffer_view_mapper.cpp)
  target_link_libraries(test_buffer_view_mapper ${PROJECT_NAME})
  ament_add_gtest(test_image_view_mapper test/test_image_view_mapper.cpp)
  target_link_libraries(test_image_view_mapper ${PROJECT_NAME})
  ament_add_gtest(test_ipc_handle_cache test/test_ipc_handle_cache.cpp)
  target_link_libraries(test_ipc_handle_cache ${PROJECT_NAME})
  ament_add_gtest(test_pointcloud2_view_mapper
                  test/test_pointcloud2_view_mapper.cpp)
  target_link_libraries(test_pointcloud2_view_mapper ${PROJECT_NAME})
  ament_add_gtest(test_type_adapters test/test_type_adapters.cpp)
  target_link_libraries(test_type_adapters ${PROJECT_NAME})
  ament_add_gtest(test_gpu_lease_pool test/test_gpu_lease_pool.cpp)
  target_link_libraries(test_gpu_lease_pool ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp rcpputils ros2_cuda_ipc_msgs rosidl_default_runtime sensor_msgs std_msgs)
ament_package()
