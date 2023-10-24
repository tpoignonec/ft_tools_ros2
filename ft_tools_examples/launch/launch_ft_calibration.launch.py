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
            'use_fake_hardware',
            default_value='true',
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
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_port',
            default_value='30200',
            description='Robot port of FRI interface.',
        )
    )
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    robot_ip = LaunchConfiguration('robot_ip')
    robot_port = LaunchConfiguration('robot_port')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare(
                    'ft_tools_examples',
                    'config',
                    'iiwa_exp.config.xacro'
                )]
            ),
            ' ',
            'prefix:=', '""'
            ' ',
            'use_sim:=', 'false',
            ' ',
            'use_fake_hardware:=', use_fake_hardware,
            ' ',
            'robot_ip:=', robot_ip,
            ' ',
            'robot_port:=', robot_port,
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    # Launch calibration node
    ft_calibration_node_config = PathJoinSubstitution(
        [
            FindPackageShare('ft_tools_examples'),
            'config',
            'config_ft_calibration.yaml',
        ]
    )
    ft_calibration_node = Node(
        package='ft_tools',
        executable='ft_calibration_node',
        namespace='',
        parameters=[robot_description, ft_calibration_node_config],
        output='both',
    )
    ft_calibration_gui_node = Node(
        package='ft_gui',
        executable='ft_calibration_gui',
        namespace='',
        output='both',
    )

    nodes = [
        ft_calibration_node,
        ft_calibration_gui_node
    ]

    return LaunchDescription(declared_arguments + nodes)
