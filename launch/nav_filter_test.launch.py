import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. Caminhos e Configurações
    pkg_share = get_package_share_directory('navigation_filter')
    
    config_path_arg = DeclareLaunchArgument(
        'config',
        default_value=os.path.join(pkg_share, 'config', 'nav_filter_params.yaml'),
        description='Caminho para o arquivo de parâmetros'
    )

    # 2. O Nó de Ciclo de Vida (Lifecycle Node)
    # IMPORTANTE: Verifique se no seu CMakeLists o nome em add_executable é 'nav_filter_node'
    nav_filter_node = LifecycleNode(
        package='navigation_filter',
        executable='nav_filter_node',  # Ajustado de 'nav_filter' para 'nav_filter_node'
        name='navigation_filter',
        namespace='', # Removi o namespace para facilitar o acesso via CLI /nav_filter
        output='screen',
        parameters=[LaunchConfiguration('config')],
        remappings=[
            ('/pose', '/mavros/local_position/pose'),           # Conecta no Pose do SLAM
            ('/imu', '/mavros/imu/data_raw'),   # Conecta na IMU do MAVROS
            ('/dvl', '/mavros/local_position/velocity_body'),   # Conecta na IMU do MAVROS
        ]
    )

    # 3. Transformações Estáticas (Usando Node em vez de ExecuteProcess)
    # Formato: [x, y, z, yaw, pitch, roll, frame_id, child_frame_id]
   
    # 3. Transformações Estáticas (Ajustadas para o ORB-SLAM3)
    # Formato: [x, y, z, yaw, pitch, roll, frame_id, child_frame_id]
    
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

    return LaunchDescription([
        config_path_arg,
        tf_base_imu,
        tf_base_camera,
        nav_filter_node
    ])