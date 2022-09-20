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
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('scara_description'), 'config', 'scara.config.xacro']
            ),
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('scara_description'), 'rviz', 'scara.rviz']
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution(
                [FindPackageShare('gazebo_ros'),
                    'launch', 'gazebo.launch.py']
            )]
        ),
        launch_arguments={'verbose': 'false'}.items(),
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', robot_description, '-entity', 'scara'],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['scara_joint_velocity_controller'],
    )

    slider_config = PathJoinSubstitution(
        [
            FindPackageShare('scara_bringup'),
            'config',
            'scara_vel_sp_config.yaml',
        ]
    )

    slider_node = Node(
        package='slider_publisher', 
        executable='slider_publisher', 
        name='slider_publisher',
        arguments = [slider_config])

    nodes = [
        gazebo,
        robot_state_pub_node,
        spawn_entity,
        rviz_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        slider_node
    ]

    return LaunchDescription(nodes)