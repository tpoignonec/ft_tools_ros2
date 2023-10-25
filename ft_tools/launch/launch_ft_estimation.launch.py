# Copyright 2023 ICube Laboratory, University of Strasbourg
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_description_content',
            description='Parsed URDF robot model',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'extimation_node_config_package',
            default_value='ft_tools',
            description='Description package with the config file in "share/config".',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'extimation_node_config_file',
            default_value='config_ft_estimation.yaml',
            description='Estimation node config file.',
        )
    )
    # Retrieve URDF
    robot_description_content = LaunchConfiguration(
        'robot_description_content'
    )
    extimation_node_config_package = LaunchConfiguration(
        'extimation_node_config_package'
    )
    extimation_node_config_file = LaunchConfiguration(
        'extimation_node_config_file'
    )
    robot_description = {'robot_description': robot_description_content}

    # Launch estimation node
    ft_estimation_node_config = PathJoinSubstitution(
        [
            FindPackageShare(extimation_node_config_package),
            'config',
            extimation_node_config_file,
        ]
    )
    ft_estimation_node = Node(
        package='ft_tools',
        executable='ft_estimation_node',
        namespace='',
        parameters=[robot_description, ft_estimation_node_config],
        output='both',
    )

    nodes = [
        ft_estimation_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
