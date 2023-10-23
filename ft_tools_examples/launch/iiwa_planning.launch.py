# Copyright 2022 ICube Laboratory, University of Strasbourg
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
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
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
            'start_rviz',
            default_value='true',
            description='Start RViz2 automatically with this launch file.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Start robot in Gazebo simulation.',
        )
    )

    # Initialize Arguments
    robot_description_content = LaunchConfiguration('robot_description_content')
    use_sim = LaunchConfiguration('use_sim')
    start_rviz = LaunchConfiguration('start_rviz')
    namespace = '/'

    # Get URDF from param
    robot_description = {'robot_description': robot_description_content}

    # Get SRDF via xacro
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare('iiwa_description'), "srdf", "iiwa.srdf.xacro"]
            ),
            " ",
            "name:=", "iiwa",
            " ",
            'description_package:=', 'iiwa_description',
        ]
    )

    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_content
    }

    # Get planning parameters
    robot_description_planning_joint_limits = PathJoinSubstitution([
            FindPackageShare('iiwa_description'), "moveit2", "iiwa_joint_limits.yaml",
        ]
    )

    robot_description_planning_cartesian_limits = PathJoinSubstitution([
            FindPackageShare('iiwa_description'), "moveit2", "iiwa_cartesian_limits.yaml",
        ]
    )

    move_group_capabilities = {
        "capabilities": """pilz_industrial_motion_planner/MoveGroupSequenceAction \
            pilz_industrial_motion_planner/MoveGroupSequenceService"""
    }

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare('iiwa_description'), "moveit2", "kinematics.yaml"]
    )

    planning_pipelines_config = PathJoinSubstitution([
            FindPackageShare('iiwa_description'), "moveit2", "planning_pipelines_config.yaml",
        ]
    )

    ompl_planning_config = PathJoinSubstitution([
            FindPackageShare('iiwa_description'), "moveit2", "ompl_planning.yaml",
        ]
    )

    moveit_controllers = PathJoinSubstitution(
        [FindPackageShare('iiwa_description'),
            "moveit2", "iiwa_moveit_controller_config.yaml"]
    )

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=namespace,
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning_cartesian_limits,
            robot_description_planning_joint_limits,
            planning_pipelines_config,
            ompl_planning_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            move_group_capabilities,
            {"use_sim_time": use_sim},
        ],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('iiwa_description'), 'rviz', 'iiwa.rviz']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_planning_cartesian_limits,
            robot_description_planning_joint_limits,
            robot_description_kinematics,
            planning_pipelines_config,
            ompl_planning_config,
        ],
        condition=IfCondition(start_rviz),
    )

    nodes = [
        move_group_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
