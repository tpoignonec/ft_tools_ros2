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
from launch.actions import DeclareLaunchArgument  # ,IncludeLaunchDescription
# from launch.conditions import IfCondition, UnlessCondition
# from launch.event_handlers import OnProcessExit, OnProcessStart
# from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_package',
            default_value='iiwa_description',
            description='Description package with robot URDF/xacro files. Usually the argument \
                         is not set, it enables use of a custom description.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file',
            default_value='iiwa.config.xacro',
            description='URDF/XACRO description file with the robot.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='true',
            description='Start robot with fake hardware mirroring command to its states.',
        )
    )

    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare(description_package), 'config', description_file]
            ),
            ' ',
            'prefix:=', '""'
            ' ',
            'use_sim:=', 'false',
            ' ',
            'use_fake_hardware:=', use_fake_hardware,
            ' ',
            'description_package:=', description_package,
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    # Launch calibration node
    ft_estimation_node_config = PathJoinSubstitution(
        [
            FindPackageShare('ft_tools'),
            'config',
            'config_ft_estimation.yaml',
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
