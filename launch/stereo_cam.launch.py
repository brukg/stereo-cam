from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('stereo_cam')
    default_config = os.path.join(pkg_dir, 'config', 'camera_params.yaml')
    
    # Load URDF
    urdf_file = os.path.join(pkg_dir, 'urdf', 'stereo_camera.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file, ' parent:=camera_frame']),
        value_type=str
    )

    return LaunchDescription([
        # Config file argument
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to config file'
        ),

        # Optional override arguments
        DeclareLaunchArgument(
            'resolution_preset',
            default_value='720p',
            description='Resolution preset (full, 1080p, 720p, vga)'
        ),

        # Stereo camera node
        Node(
            package='stereo_cam',
            executable='stereo_node',
            parameters=[
                LaunchConfiguration('config_file'),
                {'robot_description': robot_description},
                {'resolution_preset': LaunchConfiguration('resolution_preset')},
                {'frame_rate': 30}
            ],
            output='screen'
        ),

        # Robot state publisher for URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        )
    ])