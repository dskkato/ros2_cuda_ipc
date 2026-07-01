import platform
from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

RESOLUTIONS = {
    "16K": (15360, 8640),
    "8K": (7680, 4320),
    "4K": (3840, 2160),
    "1080p": (1920, 1080),
    "720p": (1280, 720),
    "480p": (852, 480),
}
DEFAULT_NSYS_FLAGS = "--trace=osrt,nvtx,cuda"
TRUE_TOKENS = {"1", "true", "on", "yes"}


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument("publish_rate_hz", default_value="30.0"),
        DeclareLaunchArgument("resolution", default_value="1080p"),
        DeclareLaunchArgument("width", default_value="1280"),
        DeclareLaunchArgument("height", default_value="720"),
        DeclareLaunchArgument("channels", default_value="1"),
        DeclareLaunchArgument(
            "colorize_shm_name", default_value="/ros2_cuda_ipc_julia_colorized"
        ),
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
        DeclareLaunchArgument(
            "memory_backend",
            default_value="cuda_ipc",
            description="Memory backend: cuda_ipc or vmm_fd",
        ),
        DeclareLaunchArgument("topic_name", default_value="julia_set/image"),
        DeclareLaunchArgument(
            "colorized_topic_name", default_value="julia_set/colorized"
        ),
        DeclareLaunchArgument("cpu_topic_name", default_value="julia_set/raw"),
        DeclareLaunchArgument(
            "compressed_topic_name", default_value="julia_set/compressed"
        ),
        DeclareLaunchArgument("log_full_copy", default_value="false"),
        DeclareLaunchArgument("use_compressed_output", default_value="false"),
        DeclareLaunchArgument("compressed_format", default_value="jpeg"),
        DeclareLaunchArgument("jpeg_quality", default_value="95"),
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

    def as_bool(name: str) -> bool:
        return value(name).strip().lower() in TRUE_TOKENS

    resolution_key = value("resolution")
    width_value = as_int("width")
    height_value = as_int("height")
    if resolution_key in RESOLUTIONS:
        width_value, height_value = RESOLUTIONS[resolution_key]

    publish_rate = as_float("publish_rate_hz")
    slot_count = as_int("slot_count")
    channels = as_int("channels")
    max_iterations = as_int("max_iterations")
    device_index = as_int("device_index")
    memory_backend = value("memory_backend")
    zoom = as_float("zoom")
    offset_x = as_float("offset_x")
    offset_y = as_float("offset_y")
    constant_real = as_float("constant_real")
    constant_imag = as_float("constant_imag")
    animation_speed = as_float("animation_speed")

    animate = as_bool("animate")
    log_full_copy = as_bool("log_full_copy")
    use_compressed_output = as_bool("use_compressed_output")
    compressed_format = value("compressed_format")
    jpeg_quality = as_int("jpeg_quality")

    frame_id = value("frame_id")
    shm_name = value("shm_name")
    topic_name = value("topic_name")
    colorized_topic = value("colorized_topic_name")
    cpu_topic = value("cpu_topic_name")
    compressed_topic = value("compressed_topic_name")

    enable_nsys = IfCondition(LaunchConfiguration("enable_nsys")).evaluate(context)
    nsys_flags = value("nsys_profile_flags") or DEFAULT_NSYS_FLAGS
    nsys_label = value("nsys_profile_label")

    profile_base = None
    if enable_nsys:
        profile_base = build_profile_name(
            nsys_label, resolution_key, width_value, height_value, publish_rate
        )

    publisher_params = {
        "publish_rate_hz": publish_rate,
        "width": width_value,
        "height": height_value,
        "channels": channels,
        "slot_count": slot_count,
        "max_iterations": max_iterations,
        "zoom": zoom,
        "offset_x": offset_x,
        "offset_y": offset_y,
        "constant_real": constant_real,
        "constant_imag": constant_imag,
        "animate": animate,
        "animation_speed": animation_speed,
        "frame_id": frame_id,
        "shm_name": shm_name,
        "device_index": device_index,
        "topic_name": topic_name,
        "memory_backend": memory_backend,
    }

    publisher_kwargs = dict(
        package="julia_set",
        executable="julia_set_publisher",
        name="julia_set_publisher",
        parameters=[publisher_params],
        output="screen",
    )
    if enable_nsys and profile_base:
        publisher_kwargs["prefix"] = (
            f"nsys profile {nsys_flags} -o {profile_base}-publisher"
        )

    publisher = Node(**publisher_kwargs)

    colorize_shm_name = value("colorize_shm_name")

    colorize_params = {
        "input_topic_name": topic_name,
        "output_topic_name": colorized_topic,
        "output_slot_count": slot_count,
        "output_channels": 3,
        "output_encoding": "rgb8",
        "output_shm_name": colorize_shm_name,
        "memory_backend": memory_backend,
    }

    colorize_kwargs = dict(
        package="julia_set",
        executable="colorize_node",
        name="colorize_node",
        parameters=[colorize_params],
        output="screen",
    )
    if enable_nsys and profile_base:
        colorize_kwargs["prefix"] = (
            f"nsys profile {nsys_flags} -o {profile_base}-colorize"
        )

    colorize = Node(**colorize_kwargs)

    if not use_compressed_output:
        transport_params = {
            "input_topic_name": colorized_topic,
            "cpu_topic_name": cpu_topic,
        }

        transport_kwargs = dict(
            package="gpu_image_transport",
            executable="gpu_image_transport",
            name="gpu_image_transport",
            parameters=[transport_params],
            output="screen",
        )

        if enable_nsys and profile_base:
            transport_kwargs["prefix"] = (
                f"nsys profile {nsys_flags} -o {profile_base}-gpu-transport"
            )

        gpu_transport = Node(**transport_kwargs)
    else:
        compressed_params = {
            "input_topic_name": colorized_topic,
            "cpu_topic_name": compressed_topic,
            "compressed_format": compressed_format,
            "jpeg_quality": jpeg_quality,
        }

        compressed_kwargs = dict(
            package="gpu_image_transport",
            executable="gpu_image_transport_compressed",
            name="gpu_image_transport_compressed",
            parameters=[compressed_params],
            output="screen",
        )

        if enable_nsys and profile_base:
            compressed_kwargs["prefix"] = (
                f"nsys profile {nsys_flags} -o {profile_base}-gpu-transport-compressed"
            )

        gpu_transport = Node(**compressed_kwargs)

    nodes: List[Node] = [publisher, colorize, gpu_transport]

    return nodes


def build_profile_name(
    label: str,
    resolution_key: str,
    width: int,
    height: int,
    publish_rate: float,
) -> str:
    resolution_token = (
        resolution_key if resolution_key in RESOLUTIONS else f"{width}x{height}"
    )
    rate_int = int(publish_rate)
    rate_token = (
        f"{rate_int}hz"
        if abs(publish_rate - rate_int) < 1e-6
        else f"{publish_rate:.1f}hz"
    )
    base = f"julia-set-{platform.machine()}-{resolution_token}-{rate_token}"
    label = label.strip()
    if label:
        base = f"{base}-{label}"
    return base
