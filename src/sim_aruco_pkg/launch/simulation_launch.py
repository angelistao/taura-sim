import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():

    # Caminho para o seu arquivo .world
    world_path = os.path.join('caminho/para/sua/pasta', 'landing_pad.world')

    # 1. Iniciar o ArduPilot SITL
    ardupilot_sitl = ExecuteProcess(
        cmd=[
            'sim_vehicle.py',
            '-v', 'ArduCopter',
            '--model', 'gazebo-iris', # Modelo de drone a ser usado
            '--map', '--console'
        ],
        cwd='/caminho/para/sua/pasta/ardupilot', # Diretório onde você clonou o ardupilot
        output='screen'
    )

    # 2. Iniciar o Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # 3. Iniciar o MAVROS
    mavros = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        parameters=[{'fcu_url': 'udp://:14550@'}] # Conecta ao SITL via UDP
    )
    
    # 4. Iniciar o nó de detecção ArUco
    # Remapeia o tópico de imagem da câmera inferior para o que o nó aruco espera
    aruco_detector = Node(
        package='aruco_ros',
        executable='marker_publisher',
        name='aruco_marker_publisher',
        remappings=[
            ('/camera/image_raw', '/cam_inferior/image_raw'),
            ('/camera/camera_info', '/cam_inferior/camera_info')
        ],
        parameters=[{'marker_size': 0.1}] # Tamanho do marcador em metros
    )

    return LaunchDescription([
        ardupilot_sitl,
        gazebo,
        mavros,
        aruco_detector
    ])