"""Launch pipeline with sample GPU image publisher and simple increment component."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description() -> LaunchDescription:
    launch_args = [
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('channels', default_value='3'),
        DeclareLaunchArgument('dtype', default_value='u8'),
        DeclareLaunchArgument('proc_count', default_value='1'),
        DeclareLaunchArgument('inplace_enabled', default_value='false'),
        DeclareLaunchArgument('slot_count', default_value='4'),
        DeclareLaunchArgument('device_index', default_value='0'),
        DeclareLaunchArgument('publish_rate_hz', default_value='30.0')
    ]

    return LaunchDescription(launch_args + [OpaqueFunction(function=_launch_setup)])


def _launch_setup(context):
    width = int(LaunchConfiguration('width').perform(context))
    height = int(LaunchConfiguration('height').perform(context))
    channels = int(LaunchConfiguration('channels').perform(context))
    dtype = LaunchConfiguration('dtype').perform(context)
    proc_count = int(LaunchConfiguration('proc_count').perform(context))
    inplace_enabled = LaunchConfiguration('inplace_enabled').perform(context).lower() == 'true'
    slot_count = int(LaunchConfiguration('slot_count').perform(context))
    device_index = int(LaunchConfiguration('device_index').perform(context))
    publish_rate_hz = float(LaunchConfiguration('publish_rate_hz').perform(context))

    publisher_node = Node(
        package='sample_nodes',
        executable='gpu_image_publisher',
        name='gpu_image_publisher',
        parameters=[{
            'width': width,
            'height': height,
            'channels': channels,
            'dtype': dtype,
            'publish_rate_hz': publish_rate_hz,
            'device_index': device_index,
            'slot_count': slot_count,
            'shm_name': '/ros2_cuda_ipc_simple_increment_demo'
        }],
        output='screen')

    increment_component = ComposableNode(
        package='simple_increment',
        plugin='simple_increment::SimpleIncrementNode',
        name='simple_increment',
        parameters=[{
            'input_topic': 'gpu_image',
            'output_topic': 'gpu_image_incremented',
            'proc_count': proc_count,
            'inplace_enabled': inplace_enabled,
            'slot_count': slot_count,
            'device_index': device_index,
            'output_shm_name': '/ros2_cuda_ipc_simple_increment_output'
        }]
    )

    container = ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container_mt',
        name='simple_increment_container',
        namespace='',
        composable_node_descriptions=[increment_component],
        output='screen'
    )

    return [publisher_node, container]

