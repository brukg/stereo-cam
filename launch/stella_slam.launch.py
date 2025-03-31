from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('stereo_cam')
    default_config = os.path.join(pkg_dir, 'config', 'camera_params.yaml')

    # Load URDF
    urdf_file = os.path.join(pkg_dir, 'urdf', 'stereo_camera.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file, ' parent:=camera_frame']),
        value_type=str
    )    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Whether to start RVIZ'
    )
    default_config  = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to config file'
    )
    # Stereo camera node
    stereo_node = Node(
        package='stereo_cam',
        executable='stereo_node',
        parameters=[
                    {'robot_description': robot_description},
                    LaunchConfiguration('config_file')
                ],
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
            '--ros-args', '-p', 'publish_tf:=true', '-p', 'camera_frame:=camera_link', '-p', 'map_frame:=map', '-p', 'odom_frame:=camera_link'
        ],
        output='screen'
    )

    rsp =  Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )


    rviz_node = Node(
        package='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz')),
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_dir, 'config', 'stereo_cam.rviz')]
        )

    return LaunchDescription([
        rviz_arg,
        default_config,
        stereo_node,
        slam_node,
        rsp,
        rviz_node
    ]) 