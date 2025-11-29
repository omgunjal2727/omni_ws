from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os

def generate_launch_description():
    # Get package directories
    omni_base_description = FindPackageShare('omni_base_description')
    omni_base_bringup = FindPackageShare('omni_base_bringup')
    
    # Path to SLAM params
    slam_params = PathJoinSubstitution([
        omni_base_bringup,
        'config',
        'slam_params.yaml'
    ])
    
    # Path to RViz config
    rviz_config = PathJoinSubstitution([
        omni_base_bringup,
        'config',
        'slam_view.rviz'
    ])
    
    return LaunchDescription([
        # Include robot description launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    omni_base_description,
                    'launch',
                    'description.launch.py'
                ])
            ])
        ),
        
        # Include RPLidar launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    omni_base_bringup,
                    'launch',
                    'rplidar.launch.py'
                ])
            ])
        ),
        
        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params]
        ),
        
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': False}]
        ),
    ])