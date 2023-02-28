# SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    ros2_cpp_template_dir = get_package_share_directory('ros2_cpp_template')
    params_file = os.path.join(ros2_cpp_template_dir, 'config', 'template.param.yaml')

    launch_component = GroupAction([
        Node(
            name='ros2_cpp_template_component',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[params_file],
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
                        plugin='Ros2CppTemplate::TemplateComposition',
                        name='ros2_cpp_template',
                        parameters=[params_file]),
                ],
            )
        ]
    )

    ld = LaunchDescription()
    ld.add_action(load_composable_nodes)
    ld.add_action(launch_component)

    return ld
