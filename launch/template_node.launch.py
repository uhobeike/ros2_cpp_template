# SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node


def generate_launch_description():
    ros2_cpp_template_dir = get_package_share_directory('ros2_cpp_template')
    params_file = os.path.join(ros2_cpp_template_dir, 'config', 'template.param.yaml')

    launch_node = GroupAction([
        Node(
            name='ros2_cpp_template_node',
            package='ros2_cpp_template',
            executable='template_node',
            parameters=[params_file],
            output='screen')
    ])

    ld = LaunchDescription()
    ld.add_action(launch_node)

    return ld
