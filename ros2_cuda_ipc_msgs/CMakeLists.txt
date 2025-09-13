cmake_minimum_required(VERSION 3.5)
project(ros2_cuda_ipc_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GpuPlane.msg"
  "msg/GpuBuffer.msg"
  "srv/GpuBufferRelease.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime builtin_interfaces)
ament_package()
