import os

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument, TimerAction
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config_path = LaunchConfiguration('config')

    configure_node = Node(
        package='lifecycle_msgs',
        executable='lifecycle_node_cli',
        name='configure_nav_filter',
        arguments=['--change-state', '/nav_filter', 'configure']
    )

    return LaunchDescription([
        # Static TF: imu_link -> base_link
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                '0', '0', '0',   # x y z
                '0', '0', '0',   # roll pitch yaw
                'base_link', 'imu_link'
            ],
            output='screen'
        ),

        # Static TF: twist_link -> imu_link
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                '0', '0', '0',
                '0', '0', '0',
                'imu_link', 'twist_link'
            ],
            output='screen'
        ),

        # Static TF: pose_link -> imu_link
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                '0', '0', '0',
                '0', '0', '0',
                'imu_link', 'pose_link'
            ],
            output='screen'
        ),
        DeclareLaunchArgument(
            'config',
            default_value=os.path.join(
                get_package_share_directory('navigation_filter'),  # Replace with your package name
                'config',
                'nav_filter_params.yaml'
            ),
            description='Path to the YAML config file for nav_filter'
        ),
        LifecycleNode(
            package='navigation_filter',  # Replace with your actual package name
            executable='nav_filter',  # Replace with your built node executable
            name='nav_filter',
            namespace='nav_filter',
            output='screen',
            parameters=[config_path]
        )
    ])
