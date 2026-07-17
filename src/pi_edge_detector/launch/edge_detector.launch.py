"""Launch file for pi_edge_detector.

Starts a multi-threaded component container and loads the
EdgeDetectorNode as a composable component with intra-process
communication enabled for zero-copy frame transfer.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('pi_edge_detector')
    default_params_file = os.path.join(pkg_share, 'config', 'default_params.yaml')

    # ---- Launch arguments ----
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the parameter YAML file'
    )

    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/camera/image_raw',
        description='Input image topic to subscribe to'
    )

    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/edge_detector/edges',
        description='Output edge image topic to publish on'
    )

    # ---- Multi-threaded component container ----
    container = ComposableNodeContainer(
        name='edge_detector_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='pi_edge_detector',
                plugin='pi_edge_detector::EdgeDetectorNode',
                name='edge_detector',
                parameters=[LaunchConfiguration('params_file')],
                remappings=[
                    ('input/image_raw', LaunchConfiguration('input_topic')),
                    ('output/edges', LaunchConfiguration('output_topic')),
                ],
                extra_arguments=[
                    {'use_intra_process_comms': True}
                ],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        params_file_arg,
        input_topic_arg,
        output_topic_arg,
        container,
    ])
