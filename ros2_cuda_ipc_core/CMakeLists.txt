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
  src/detail/cuda_util.cpp
  src/detail/cuda_context.cpp
  src/publisher/gpu_buffer_manager.cpp
  src/publisher/gpu_buffer_pool.cpp
  src/backend/memory_importer.cpp
  src/backend/cuda_ipc/memory_backend.cpp
  src/backend/cuda_ipc/memory_importer.cpp
  src/backend/vmm_fd/memory_backend.cpp
  src/backend/vmm_fd/memory_importer.cpp
  src/subscriber/buffer_view.cpp
  src/image/image_view.cpp
  src/pointcloud2/pointcloud2_view.cpp
  src/subscriber/ipc_handle_cache.cpp
  src/lease/lease_mapping.cpp
  src/lease/lease_handle.cpp
  src/publisher/lease_manager.cpp
  src/subscriber/buffer_view_mapper.cpp
  src/image/image_view_mapper.cpp
  src/pointcloud2/pointcloud2_view_mapper.cpp
  src/transport/message_utils.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    rclcpp::rclcpp
    rcpputils::rcpputils
    ${ros2_cuda_ipc_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    CUDA::cudart
    CUDA::cuda_driver
    ${UUID_LIB}
)

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
  ament_add_gtest(test_ipc_handle_cache test/test_ipc_handle_cache.cpp)
  target_link_libraries(test_ipc_handle_cache ${PROJECT_NAME})
  # mapper tests
  ament_add_gtest(test_buffer_view_mapper test/mapper/test_buffer_view_mapper.cpp)
  target_link_libraries(test_buffer_view_mapper ${PROJECT_NAME})
  ament_add_gtest(test_image_view_mapper test/mapper/test_image_view_mapper.cpp)
  target_link_libraries(test_image_view_mapper ${PROJECT_NAME})
  ament_add_gtest(test_pointcloud2_view_mapper
                  test/mapper/test_pointcloud2_view_mapper.cpp)
  target_link_libraries(test_pointcloud2_view_mapper ${PROJECT_NAME})
  ament_add_gtest(test_publisher_messages test/test_publisher_messages.cpp)
  target_link_libraries(test_publisher_messages ${PROJECT_NAME})
  ament_add_gtest(test_gpu_buffer_manager
                  test/test_gpu_buffer_manager.cpp)
  target_link_libraries(test_gpu_buffer_manager ${PROJECT_NAME})
  ament_add_gtest(test_gpu_buffer_pool test/test_gpu_buffer_pool.cpp)
  target_link_libraries(test_gpu_buffer_pool ${PROJECT_NAME})
  ament_add_gtest(test_lease_manager test/test_lease_manager.cpp)
  target_link_libraries(test_lease_manager ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(CUDAToolkit rclcpp rcpputils ros2_cuda_ipc_msgs rosidl_default_runtime sensor_msgs std_msgs)
ament_package()
