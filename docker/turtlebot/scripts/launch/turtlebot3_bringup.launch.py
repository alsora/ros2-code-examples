import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    urdf = '/root/scripts/visualization/turtlebot3_wafflepi.urdf'
    return LaunchDescription([
        Node(package='robot_state_publisher', node_executable='robot_state_publisher', output='screen', arguments=[urdf]),
        Node(package='turtlebot3_node', node_executable='time_sync', output='screen'),
        Node(package='turtlebot3_node', node_executable='odometry_publisher', output='screen'),
        Node(package='turtlebot3_node', node_executable='tf_publisher', output='screen'),
        Node(package='turtlebot3_node', node_executable='joint_states_publisher', output='screen'),
        Node(package='turtlebot3_node', node_executable='scan_publisher', output='screen')
])
