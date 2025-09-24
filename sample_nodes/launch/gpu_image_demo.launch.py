import launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    publisher = Node(
        package="sample_nodes",
        executable="gpu_image_publisher",
        name="gpu_image_publisher",
        parameters=[
            {"publish_rate_hz": 30.0},
            {"frame_id": "gpu_camera"},
            {"width": 640},
            {"height": 480},
            {"channels": 3},
            {"slot_count": 4},
            {"pending_ttl_ms": 100},
        ],
        output="screen",
    )

    subscribers = [
        Node(
            package="sample_nodes",
            executable="gpu_image_subscriber",
            name="gpu_image_subscriber_1",
            output="screen",
        ),
        Node(
            package="sample_nodes",
            executable="gpu_image_subscriber",
            name="gpu_image_subscriber_2",
            output="screen",
        ),
    ]

    return LaunchDescription([publisher, *subscribers])
