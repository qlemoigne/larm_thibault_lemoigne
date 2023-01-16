import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    tbot_sim_path = get_package_share_directory('tbot_start')
    launch_file_dir = os.path.join(tbot_sim_path, 'launch')

    return LaunchDescription([


        Node(package='rviz2', namespace='', executable='rviz2', prefix='gnome-terminal -x'),

        Node(package='grp_vincent', namespace='', executable='scan_echo'),

        Node(package='teleop_twist_keyboard', namespace='', executable='teleop_twist_keyboard', prefix='gnome-terminal -x', remappings=[('cmd_vel', '/multi/cmd_teleop')]),
    
    ])
