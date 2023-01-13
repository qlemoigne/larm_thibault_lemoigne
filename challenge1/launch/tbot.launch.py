import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    tbot_sim_path = get_package_share_directory('tbot_sim')
    launch_file_dir = os.path.join(tbot_sim_path, 'launch','includes')

    return LaunchDescription([
        
    
        Node(package='tbot_start', namespace='', executable='start_base', prefix='gnome-terminal -x'),
        
        Node(package='ros1_bridge', namespace='', executable='dynamic_bridge', prefix='gnome-terminal -x'),

        Node(package='urg_node', namespace='', executable='urg_node_driver', parameters=[
            {"serial_port": "/dev/ttyACM0"}
        ], prefix='gnome-terminal -x'),

        #Node(package='challenge1', namespace='', executable='camera', prefix='gnome-terminal -x'),

        Node(package='challenge1', namespace='', executable='scan_echo', prefix='gnome-terminal -x'),

        #Node(package='challenge1', namespace='', executable='move', prefix='gnome-terminal -x'),
    
    ])
