from launch import LaunchDescription
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    return LaunchDescription([
        LifecycleNode(
            package='navigation_filter',  # Replace with your actual package name
            executable='nav_filter',  # Replace with your built node executable
            name='nav_filter',
            namespace='nav_filter',
            output='screen',
            parameters=[
                {'base_link': 'base_link'},
                {'imu_link': 'base_link'},
                {'twist_link': 'base_link'},
                {'pose_link': 'base_link'}
            ]
        )
    ])
