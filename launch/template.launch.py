# SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    ros2_cpp_template_dir = get_package_share_directory('ros2_cpp_template')
    config_dir = os.path.join(ros2_cpp_template_dir, 'config')

    params_file = LaunchConfiguration('params_file')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            config_dir, 'template.param.yaml'),
        description='Param')

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites={},
        convert_types=True)

    launch_component = GroupAction([
        Node(
            name='ros2_cpp_template_component',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params],
            arguments=['--ros-args'],
            output='screen')
    ])

    load_composable_nodes = GroupAction(
        actions=[
            LoadComposableNodes(
                target_container="ros2_cpp_template_component",
                composable_node_descriptions=[
                    ComposableNode(
                        package='ros2_cpp_template',
                        plugin='Ros2CppTemplate::Template',
                        name='ros2_cpp_template',
                        parameters=[configured_params]),
                ],
            )
        ]
    )

    ld = LaunchDescription()
    ld.add_action(declare_params_file_cmd)
    ld.add_action(load_composable_nodes)
    ld.add_action(launch_component)

    return ld
