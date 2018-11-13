import os

from ament_index_python.packages import get_package_share_directory
from ros2run.api import get_executable_path

def launch(launch_descriptor, argv):
    ld = launch_descriptor
    package = 'cartographer_ros'
    turtlebot3_cartographer_prefix = get_package_share_directory('turtlebot3_cartographer')
    cartographer_config_dir = '/root/scripts/cartographer_config/'
    ld.add_process(
        cmd=[
            get_executable_path(package_name=package, executable_name='cartographer_node'),
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', 'turtlebot2.lua'  #'turtlebot3_lds_2d.lua'
        ],
        name='cartographer_node',
    )


    ld.add_process(
        cmd=[
            get_executable_path(package_name='rviz2', executable_name='rviz2'),
            '-d', '/root/scripts/visualization/turtlebot.rviz'
        ],
        name='rviz2_node',
    )

    return ld
