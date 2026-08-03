import platform
from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_NSYS_FLAGS = "--trace=osrt,nvtx,cuda"


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument("image_topic", default_value="/fanout/image_gpu"),
        DeclareLaunchArgument("preview_topic", default_value="/fanout/preview/image"),
        DeclareLaunchArgument(
            "encoder_status_topic", default_value="/fanout/encoder_like/status"
        ),
        DeclareLaunchArgument(
            "inference_status_topic", default_value="/fanout/inference_like/status"
        ),
        DeclareLaunchArgument("publish_rate_hz", default_value="30.0"),
        DeclareLaunchArgument("width", default_value="1920"),
        DeclareLaunchArgument("height", default_value="1080"),
        DeclareLaunchArgument("device_index", default_value="0"),
        DeclareLaunchArgument("frame_id", default_value="fanout_camera_frame"),
        DeclareLaunchArgument("block_count", default_value="4"),
        DeclareLaunchArgument(
            "shm_name_prefix", default_value="/ros2_cuda_ipc_fanout"
        ),
        DeclareLaunchArgument("preview_copy_every_n", default_value="1"),
        DeclareLaunchArgument("log_every_n", default_value="30"),
        DeclareLaunchArgument("encoder_downscale", default_value="2"),
        DeclareLaunchArgument("enable_nsys", default_value="false"),
        DeclareLaunchArgument(
            "nsys_profile_label",
            default_value="",
            description="Label appended to nsys profile outputs",
        ),
        DeclareLaunchArgument(
            "nsys_profile_flags",
            default_value=DEFAULT_NSYS_FLAGS,
            description="Flags forwarded to nsys profile",
        ),
    ]

    ld = LaunchDescription(arguments)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld


def launch_setup(context) -> List[Node]:
    def value(name: str) -> str:
        return LaunchConfiguration(name).perform(context)

    def as_int(name: str) -> int:
        return int(float(value(name)))

    def as_float(name: str) -> float:
        return float(value(name))

    width_value = as_int("width")
    height_value = as_int("height")

    publish_rate = as_float("publish_rate_hz")
    block_count = as_int("block_count")
    device_index = as_int("device_index")
    image_topic = value("image_topic")
    preview_topic = value("preview_topic")
    encoder_status_topic = value("encoder_status_topic")
    inference_status_topic = value("inference_status_topic")
    log_every_n = as_int("log_every_n")

    enable_nsys = IfCondition(LaunchConfiguration("enable_nsys")).evaluate(context)
    nsys_flags = value("nsys_profile_flags") or DEFAULT_NSYS_FLAGS
    nsys_label = value("nsys_profile_label")
    profile_base = None
    if enable_nsys:
        profile_base = build_profile_name(
            nsys_label, width_value, height_value, publish_rate
        )

    publisher = make_node(
        enable_nsys,
        profile_base,
        nsys_flags,
        "publisher",
        package="multi_process_image_fanout",
        executable="gpu_image_publisher",
        name="gpu_image_publisher",
        parameters=[
            {
                "topic_name": image_topic,
                "publish_rate_hz": publish_rate,
                "width": width_value,
                "height": height_value,
                "frame_id": value("frame_id"),
                "block_count": block_count,
                "shm_name_prefix": value("shm_name_prefix"),
                "device_index": device_index,
            }
        ],
        output="screen",
    )

    preview = make_node(
        enable_nsys,
        profile_base,
        nsys_flags,
        "preview",
        package="multi_process_image_fanout",
        executable="preview_node",
        name="preview_node",
        parameters=[
            {
                "input_topic_name": image_topic,
                "output_topic_name": preview_topic,
                "copy_every_n": as_int("preview_copy_every_n"),
                "log_every_n": log_every_n,
            }
        ],
        output="screen",
    )

    encoder = make_node(
        enable_nsys,
        profile_base,
        nsys_flags,
        "encoder-like",
        package="multi_process_image_fanout",
        executable="encoder_like_node",
        name="encoder_like_node",
        parameters=[
            {
                "input_topic_name": image_topic,
                "status_topic_name": encoder_status_topic,
                "downscale": as_int("encoder_downscale"),
                "log_every_n": log_every_n,
            }
        ],
        output="screen",
    )

    inference = make_node(
        enable_nsys,
        profile_base,
        nsys_flags,
        "inference-like",
        package="multi_process_image_fanout",
        executable="inference_like_node",
        name="inference_like_node",
        parameters=[
            {
                "input_topic_name": image_topic,
                "status_topic_name": inference_status_topic,
                "log_every_n": log_every_n,
            }
        ],
        output="screen",
    )

    return [publisher, preview, encoder, inference]


def make_node(
    enable_nsys: bool,
    profile_base: str,
    nsys_flags: str,
    profile_suffix: str,
    **kwargs,
) -> Node:
    if enable_nsys and profile_base:
        kwargs["prefix"] = f"nsys profile {nsys_flags} -o {profile_base}-{profile_suffix}"
    return Node(**kwargs)


def build_profile_name(
    label: str,
    width: int,
    height: int,
    publish_rate: float,
) -> str:
    size_token = f"{width}x{height}"
    rate_int = int(publish_rate)
    rate_token = (
        f"{rate_int}hz"
        if abs(publish_rate - rate_int) < 1e-6
        else f"{publish_rate:.1f}hz"
    )
    base = f"fanout-{platform.machine()}-{size_token}-{rate_token}"
    label = label.strip()
    if label:
        base = f"{base}-{label}"
    return base
