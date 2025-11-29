from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('omni_base_bringup').find('omni_base_bringup')

    return LaunchDescription([

        # Cartographer SLAM Node
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer',
            output='screen',
            arguments=[
                '-configuration_directory', PathJoinSubstitution([pkg_share, 'config']),
                '-configuration_basename', 'cartographer.lua'
            ],
            parameters=[
                {'use_sim_time': False}
            ]
        ),

        # Occupancy Grid (map publisher) Node
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[
                {
                    'resolution': 0.05,
                    'use_sim_time': False
                }
            ]
        ),
    ])
