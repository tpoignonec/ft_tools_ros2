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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
# from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='Start robot with fake hardware mirroring command to its states.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.170.10.2',
            description='Robot IP of FRI interface',
        )
    )

    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    robot_ip = LaunchConfiguration('robot_ip')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [
                    FindPackageShare('ft_tools_examples'),
                    'config',
                    'iiwa_exp.config.xacro'
                ]
            ),
            ' ',
            'prefix:=', '""'
            ' ',
            'use_sim:=', 'false',
            ' ',
            'use_fake_hardware:=', use_fake_hardware,
            ' ',
            'robot_ip:=', robot_ip,
        ]
    )
    # robot_description = {'robot_description': robot_description_content}

    # Launch Moveit2 planning pipe
    iiwa_planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ft_tools_examples'),
            '/launch',
            '/iiwa_planning.launch.py'
        ]),
        launch_arguments={
            'robot_description_content': robot_description_content,
            'start_rviz': 'true',
            'use_sim': 'false',
        }.items(),
    )

    # Launch calibration node
    calibration_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ft_tools'),
            '/launch',
            '/launch_ft_calibration.launch.py'
        ]),
        launch_arguments={
            'robot_description_content': robot_description_content,
            'calibration_node_config_package': 'ft_tools_examples',
            'calibration_node_config_file': 'config_ft_calibration.yaml',
        }.items(),
    )

    # Launch estimation node
    extimation_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ft_tools'),
            '/launch',
            '/launch_ft_estimation.launch.py'
        ]),
        launch_arguments={
            'robot_description_content': robot_description_content,
            'extimation_node_config_package': 'ft_tools_examples',
            'extimation_node_config_file': 'config_ft_estimation.yaml',
        }.items(),
    )

    # Return
    nodes = [
        iiwa_planning_launch,
        calibration_node_launch,
        extimation_node_launch
    ]

    return LaunchDescription(declared_arguments + nodes)
