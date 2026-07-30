cmake_minimum_required(VERSION 3.16)
project(ros2_cuda_ipc_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rcutils REQUIRED)
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
  src/detail/cuda_driver_context.cpp
  src/detail/interprocess_event.cpp
  src/publisher/gpu_buffer_manager.cpp
  src/publisher/gpu_buffer_pool.cpp
  src/backend/memory_importer.cpp
  src/backend/cuda_ipc/memory_backend.cpp
  src/backend/cuda_ipc/memory_importer.cpp
  src/backend/vmm_fd/memory_backend.cpp
  src/backend/vmm_fd/memory_importer.cpp
  src/subscriber/read_handle.cpp
  src/subscriber/buffer_metadata_cache.cpp
  src/image/image_view.cpp
  src/pointcloud2/pointcloud2_view.cpp
  src/subscriber/ipc_handle_cache.cpp
  src/buffer_metadata/buffer_metadata.cpp
  src/buffer_metadata/buffer_ref.cpp
  src/publisher/buffer_metadata_manager.cpp
  src/subscriber/buffer_mapper.cpp
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
    rcutils::rcutils
    rclcpp::rclcpp
    rcpputils::rcpputils
    ${ros2_cuda_ipc_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
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
  FILES_MATCHING PATTERN "*.hpp"
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_buffer_ref test/test_buffer_ref.cpp)
  target_link_libraries(test_buffer_ref ${PROJECT_NAME})
  ament_add_gtest(test_ipc_handle_cache test/test_ipc_handle_cache.cpp)
  target_link_libraries(test_ipc_handle_cache ${PROJECT_NAME})
  ament_add_gtest(test_buffer_metadata_cache test/test_buffer_metadata_cache.cpp)
  target_link_libraries(test_buffer_metadata_cache ${PROJECT_NAME})
  # mapper tests
  ament_add_gtest(test_buffer_mapper test/mapper/test_buffer_mapper.cpp)
  target_include_directories(test_buffer_mapper PRIVATE test)
  target_link_libraries(test_buffer_mapper ${PROJECT_NAME})
  ament_add_gtest(test_image_view_mapper test/mapper/test_image_view_mapper.cpp)
  target_include_directories(test_image_view_mapper PRIVATE test)
  target_link_libraries(test_image_view_mapper ${PROJECT_NAME})
  ament_add_gtest(test_pointcloud2_view_mapper
                  test/mapper/test_pointcloud2_view_mapper.cpp)
  target_include_directories(test_pointcloud2_view_mapper PRIVATE test)
  target_link_libraries(test_pointcloud2_view_mapper ${PROJECT_NAME})
  ament_add_gtest(test_publisher_messages test/test_publisher_messages.cpp)
  target_link_libraries(test_publisher_messages ${PROJECT_NAME})
  ament_add_gtest(test_gpu_buffer_manager
                  test/test_gpu_buffer_manager.cpp)
  target_link_libraries(test_gpu_buffer_manager ${PROJECT_NAME} CUDA::cudart)
  ament_add_gtest(test_gpu_buffer_pool test/test_gpu_buffer_pool.cpp)
  target_link_libraries(test_gpu_buffer_pool ${PROJECT_NAME} CUDA::cudart)
  ament_add_gtest(test_read_handle test/test_read_handle.cpp)
  target_link_libraries(test_read_handle ${PROJECT_NAME} CUDA::cudart)
  ament_add_gtest(test_buffer_metadata_manager test/test_buffer_metadata_manager.cpp)
  target_link_libraries(test_buffer_metadata_manager ${PROJECT_NAME})
  ament_add_gtest(test_cuda_driver_context test/test_cuda_driver_context.cpp)
  target_link_libraries(test_cuda_driver_context ${PROJECT_NAME} CUDA::cudart)
  add_executable(cuda_ipc_import_helper
                 test/cuda_ipc_import_helper.cpp)
  target_include_directories(cuda_ipc_import_helper PRIVATE test)
  target_link_libraries(cuda_ipc_import_helper ${PROJECT_NAME})
  ament_add_gtest(test_cuda_ipc_memory_driver
                  test/test_cuda_ipc_memory_driver.cpp)
  target_include_directories(test_cuda_ipc_memory_driver PRIVATE test)
  target_compile_definitions(
    test_cuda_ipc_memory_driver
    PRIVATE
      CUDA_IPC_IMPORT_HELPER_PATH="$<TARGET_FILE:cuda_ipc_import_helper>")
  target_link_libraries(test_cuda_ipc_memory_driver ${PROJECT_NAME})
  add_dependencies(test_cuda_ipc_memory_driver cuda_ipc_import_helper)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(CUDAToolkit rcutils rclcpp rcpputils ros2_cuda_ipc_msgs rosidl_default_runtime sensor_msgs std_msgs)
ament_package()
