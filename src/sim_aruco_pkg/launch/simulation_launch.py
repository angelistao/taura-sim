import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # --- Encontra os caminhos corretos dentro do seu pacote ---
    pkg_share = get_package_share_directory('sim_aruco_pkg')
    world_path = os.path.join(pkg_share, 'worlds', 'landing_aruco.world')

    # !! IMPORTANTE !!: Altere este caminho para onde você clonou o repositório do ardupilot
    ardupilot_dir = '/home/angelistao/ardupilot' 

    # --- Definição dos Nós e Processos ---

    # 1. Iniciar o ArduPilot SITL
    ardupilot_sitl = ExecuteProcess(
        cmd=[
            'sim_vehicle.py', '-v', 'ArduCopter',
            '--model', 'gazebo-iris',
            '--map', '--console'
        ],
        cwd=ardupilot_dir,
        output='screen'
    )

    # 2. Iniciar o Gazebo com o seu mundo
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose', world_path,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    # 3. Iniciar o MAVROS
    mavros = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        parameters=[{'fcu_url': 'udp://:14550@'}]
    )
    
    # 4. Iniciar o nó de detecção ArUco
    aruco_detector = Node(
        package='aruco_ros',
        executable='marker_publisher',
        name='aruco_marker_publisher',
        remappings=[
            ('/camera/image_raw', '/cam_inferior/image_raw'),
            ('/camera/camera_info', '/cam_inferior/camera_info')
        ],
        parameters=[{'marker_size': 0.15}] # Ajustado para o tamanho no seu SDF
    )

    # 5. Iniciar o seu nó de controle autônomo
    autoland_node = Node(
        package='sim_aruco_pkg',
        executable='auto_pouso_node.py',
        name='autoland_node',
        output='screen'
    )

    return LaunchDescription([
        ardupilot_sitl,
        gazebo,
        mavros,
        aruco_detector,
        autoland_node # Adicionado para iniciar seu script automaticamente
    ])