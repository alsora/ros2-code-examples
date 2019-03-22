# Copyright 2019 Open Source Robotics Foundation, Inc.
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

# /* Author: Darby Lim */

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_dir = LaunchConfiguration('map',
                                default=os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'map', 'map.yaml'))

    param_dir = LaunchConfiguration('params',
                                default=os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'param', 'burger_params.yaml'))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    rviz_config_dir = os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'rviz', 'tb3_navigation2.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        Node(
            package='nav2_map_server',
            node_executable='map_server',
            node_name='map_server', 
            output='screen',
            parameters=[{ 'use_sim_time': use_sim_time}, { 'yaml_filename': map_dir}]),

        Node(
            package='nav2_amcl',
            node_executable='amcl',
            node_name='amcl',
            output='screen',
            parameters=[{ 'use_sim_time': use_sim_time}]),

        Node(
            package='nav2_world_model',
            node_executable='world_model',
            output='screen',
            parameters=[param_dir]),

        Node(
            package='dwb_controller',
            node_executable='dwb_controller',
            output='screen',
            parameters=[param_dir]),

        Node(
            package='nav2_navfn_planner',
            node_executable='navfn_planner',
            node_name='navfn_planner',
            output='screen',
            parameters=[{ 'use_sim_time': use_sim_time}]),

        Node(
            package='nav2_bt_navigator',
            node_executable='bt_navigator',
            node_name='bt_navigator',
            output='screen',
            parameters=[{ 'use_sim_time': use_sim_time}]),

        Node(
            package='nav2_mission_executor',
            node_executable='mission_executor',
            node_name='mission_executor',
            output='screen',
            parameters=[{ 'use_sim_time': use_sim_time}]),

        Node(
            package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'),

        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/world_model', 'use_sim_time', use_sim_time],
            output='screen'),

        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/global_costmap/global_costmap', 'use_sim_time', use_sim_time],
            output='screen'),

        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/local_costmap/local_costmap', 'use_sim_time', use_sim_time],
            output='screen')
])
