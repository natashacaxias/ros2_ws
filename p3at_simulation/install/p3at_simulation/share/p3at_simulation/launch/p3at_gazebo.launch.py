import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Caminho para o diretório de compartilhamento do nosso pacote
    pkg_share = get_package_share_directory('p3at_simulation')

    # Caminho para o pacote de integração do Gazebo
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # --- AJUSTE: Adicionar o diretório do nosso pacote ao caminho de recursos do Gazebo ---
    # Isso permite que o Gazebo encontre as pastas 'meshes', 'worlds', etc.
    set_model_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=pkg_share
    )

    # Iniciar o Gazebo com um mundo vazio
    # Se quiser usar um mundo seu, troque 'empty.sdf' pelo caminho do seu arquivo
    # Ex: os.path.join(pkg_share, 'worlds', 'meu_mundo.sdf')
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # Processar o arquivo XACRO do robô
    urdf_file_path = os.path.join(pkg_share, 'urdf', 'p3at.xacro')
    robot_description_config = xacro.process_file(urdf_file_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Iniciar o Robot State Publisher
    start_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Iniciar o Spawner para colocar o robô no Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-entity', 'p3at'],
        output='screen'
    )

    # Retornar a descrição do lançamento
    return LaunchDescription([
        set_model_path,
        start_gazebo,
        start_robot_state_publisher,
        spawn_entity,
    ])
