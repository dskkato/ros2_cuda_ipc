import launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    publisher = Node(
        package="sample_nodes",
        executable="gpu_pointcloud_publisher",
        name="gpu_pointcloud_publisher",
        parameters=[
            {"publish_rate_hz": 10.0},
            {"frame_id": "gpu_lidar"},
            {"width": 1024},
            {"height": 1},
            {"slot_count": 4},
        ],
        output="screen",
    )

    subscribers = [
        Node(
            package="sample_nodes",
            executable="gpu_pointcloud_subscriber",
            name="gpu_pointcloud_subscriber_1",
            output="screen",
        ),
        Node(
            package="sample_nodes",
            executable="gpu_pointcloud_subscriber",
            name="gpu_pointcloud_subscriber_2",
            output="screen",
        ),
    ]

    return LaunchDescription([publisher, *subscribers])
