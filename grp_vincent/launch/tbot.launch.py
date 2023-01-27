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

    # ros2 launch slam_toolbox online_async_launch.py

    slam_toolbox_path = get_package_share_directory('slam_toolbox')
    launch_file_dir2 = os.path.join(slam_toolbox_path, 'launch')


    naviation_path = get_package_share_directory('nav2_bringup')
    launch_file_dir3 = os.path.join(naviation_path, 'launch')

    return LaunchDescription([
        
    
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/minimal.launch.py'])
            ),

        

        Node(package='grp_vincent', namespace='', executable='scan_echo'),

        Node(package='grp_vincent', namespace='', executable='move'),

        Node(package='grp_vincent', namespace='', executable='objects'),

         # PATH FINDING
         
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir2, '/online_async_launch.py']),
            launch_arguments={'use_sim_time': 'False'}.items()
            ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir3, '/navigation_launch.py']))
    
    ])
