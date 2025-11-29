from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('omni_base_description')

    # Path to xacro
    xacro_file = os.path.join(pkg_share, 'urdf', 'omni_base.urdf.xacro')

    # Convert xacro -> URDF
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_raw,
                'use_sim_time': False
            }]
        ),

        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen'
        )
    ])
