from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    publish_rate = LaunchConfiguration("publish_rate_hz")
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    channels = LaunchConfiguration("channels")
    slot_count = LaunchConfiguration("slot_count")
    max_iterations = LaunchConfiguration("max_iterations")
    zoom = LaunchConfiguration("zoom")
    offset_x = LaunchConfiguration("offset_x")
    offset_y = LaunchConfiguration("offset_y")
    constant_real = LaunchConfiguration("constant_real")
    constant_imag = LaunchConfiguration("constant_imag")
    animate = LaunchConfiguration("animate")
    animation_speed = LaunchConfiguration("animation_speed")
    frame_id = LaunchConfiguration("frame_id")
    shm_name = LaunchConfiguration("shm_name")
    device_index = LaunchConfiguration("device_index")
    topic_name = LaunchConfiguration("topic_name")
    sample_bytes = LaunchConfiguration("sample_bytes")
    log_full_copy = LaunchConfiguration("log_full_copy")

    arguments = [
        DeclareLaunchArgument("publish_rate_hz", default_value="30.0"),
        DeclareLaunchArgument("width", default_value="1280"),
        DeclareLaunchArgument("height", default_value="720"),
        DeclareLaunchArgument("channels", default_value="3"),
        DeclareLaunchArgument("slot_count", default_value="4"),
        DeclareLaunchArgument("max_iterations", default_value="300"),
        DeclareLaunchArgument("zoom", default_value="1.5"),
        DeclareLaunchArgument("offset_x", default_value="0.0"),
        DeclareLaunchArgument("offset_y", default_value="0.0"),
        DeclareLaunchArgument("constant_real", default_value="-0.8"),
        DeclareLaunchArgument("constant_imag", default_value="0.156"),
        DeclareLaunchArgument("animate", default_value="true"),
        DeclareLaunchArgument("animation_speed", default_value="1.0"),
        DeclareLaunchArgument("frame_id", default_value="julia_frame"),
        DeclareLaunchArgument("shm_name", default_value="/ros2_cuda_ipc_julia"),
        DeclareLaunchArgument("device_index", default_value="0"),
        DeclareLaunchArgument("topic_name", default_value="julia_set/image"),
        DeclareLaunchArgument("sample_bytes", default_value="64"),
        DeclareLaunchArgument("log_full_copy", default_value="false"),
    ]

    publisher = Node(
        package="julia_set",
        executable="julia_set_publisher",
        name="julia_set_publisher",
        parameters=[
            {"publish_rate_hz": publish_rate},
            {"width": width},
            {"height": height},
            {"channels": channels},
            {"slot_count": slot_count},
            {"max_iterations": max_iterations},
            {"zoom": zoom},
            {"offset_x": offset_x},
            {"offset_y": offset_y},
            {"constant_real": constant_real},
            {"constant_imag": constant_imag},
            {"animate": animate},
            {"animation_speed": animation_speed},
            {"frame_id": frame_id},
            {"shm_name": shm_name},
            {"device_index": device_index},
            {"topic_name": topic_name},
        ],
        output="screen",
    )

    subscriber = Node(
        package="julia_set",
        executable="julia_set_subscriber",
        name="julia_set_subscriber",
        parameters=[
            {"topic_name": topic_name},
            {"sample_bytes": sample_bytes},
            {"log_full_copy": log_full_copy},
        ],
        output="screen",
    )

    return LaunchDescription([*arguments, publisher, subscriber])
