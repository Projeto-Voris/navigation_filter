from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import ExecuteProcess

def generate_launch_description():
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
