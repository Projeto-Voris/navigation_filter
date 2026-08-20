import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('sensor_fusion')
    
    config_path_arg = DeclareLaunchArgument(
        'params',
        default_value=os.path.join(pkg_share, 'params', 'sensor_fusion_params.yaml'),
        description='Caminho para o arquivo de parâmetros'
    )

    nav_filter_node = LifecycleNode(
        package='sensor_fusion',
        executable='sensor_fusion_node.py',  # Ajustado de 'nav_filter' para 'nav_filter_node'
        name='sensor_fusion',
        namespace='', 
        output='screen',
        parameters=[LaunchConfiguration('params')],
        remappings=[
            ('/pose', '/mavros/local_position/pose'),           # Conecta no Pose do SLAM
            ('/imu', '/mavros/imu/data_raw'),   # Conecta na IMU do MAVROS
            ('/dvl', '/mavros/local_position/velocity_body'),   # Conecta na IMU do MAVROS
        ]
    )

    ''' 
    
    tf_base_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_camera',
        arguments=['0', '0', '0', '0', '0', '0', 'SM2/base_link', 'SM2/left_camera_link'],
        parameters=[{'use_sim_time': True}]
    )

    tf_base_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_imu',
        arguments=['0', '0', '0', '0', '0', '0', 'SM2/base_link', 'imu_link'],
        parameters=[{'use_sim_time': True}]
    )

    '''

    return LaunchDescription([
        config_path_arg,
        # tf_base_imu,
        # tf_base_camera,
        nav_filter_node
    ])