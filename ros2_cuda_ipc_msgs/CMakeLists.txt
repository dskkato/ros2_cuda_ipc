cmake_minimum_required(VERSION 3.16)
project(ros2_cuda_ipc_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(interface_files
  "msg/BufferCore.msg"
  "msg/GpuImage.msg"
  "msg/GpuPointCloud2.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${interface_files}
  DEPENDENCIES std_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
