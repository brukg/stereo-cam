from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('stereo_cam')
    
    # Stereo camera node
    stereo_node = Node(
        package='stereo_cam',
        executable='stereo_node',
        parameters=[os.path.join(pkg_dir, 'config', 'camera_params.yaml')],
        output='screen'
    )

    # Stella SLAM node with command line arguments
    slam_node = Node(
        package='stella_vslam_ros',
        executable='run_slam',
        arguments=[
            '--vocab', os.path.join(pkg_dir, 'config', 'orb_vocab.fbow'),
            '--config', os.path.join(pkg_dir, 'config', 'stella.yaml'),
            '--map-db-out', os.path.join(pkg_dir, 'data', 'map.msg'),
            '--ros-args', '-p', 'publish_tf:=true'
        ],
        output='screen'
    )

    return LaunchDescription([
        stereo_node,
        slam_node
    ]) 