"""Launch file for pi_yolov26_detector.

Starts a multi-threaded component container and loads the
Yolov26DetectorNode as a composable component with intra-process
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
    pkg_share = get_package_share_directory('pi_yolov26_detector')
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
        default_value='/yolov26_detector/detections',
        description='Output annotated image topic to publish on'
    )

    # ---- Multi-threaded component container ----
    container = ComposableNodeContainer(
        name='yolov26_detector_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='pi_yolov26_detector',
                plugin='pi_yolov26_detector::Yolov26DetectorNode',
                name='yolov26_detector',
                parameters=[
                    LaunchConfiguration('params_file'),
                    {'model_path': os.path.join(pkg_share, 'models', 'yolov8n.onnx')}
                ],
                remappings=[
                    ('input/image_raw', LaunchConfiguration('input_topic')),
                    ('output/detections', LaunchConfiguration('output_topic')),
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
