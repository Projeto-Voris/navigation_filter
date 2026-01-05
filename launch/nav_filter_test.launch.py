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
        name='nav_filter',
        namespace='', # Removi o namespace para facilitar o acesso via CLI /nav_filter
        output='screen',
        parameters=[LaunchConfiguration('config')]
    )

    # 3. Transformações Estáticas (Usando Node em vez de ExecuteProcess)
    # Formato: [x, y, z, yaw, pitch, roll, frame_id, child_frame_id]
    tf_base_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
    )

    tf_imu_twist = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'imu_link', 'twist_link']
    )

    tf_imu_pose = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'imu_link', 'pose_link']
    )

    return LaunchDescription([
        config_path_arg,
        tf_base_imu,
        tf_imu_twist,
        tf_imu_pose,
        nav_filter_node
    ])