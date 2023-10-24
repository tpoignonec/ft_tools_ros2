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
# from launch.conditions import IfCondition, UnlessCondition
# from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
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
    robot_description = {'robot_description': robot_description_content}

    # Running with Moveit2 planning
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='/',
        output='both',
        parameters=[robot_description]
    )

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
        robot_state_pub_node,
        iiwa_planning_launch,
        ft_calibration_node,
        ft_calibration_gui_node
    ]

    return LaunchDescription(declared_arguments + nodes)
