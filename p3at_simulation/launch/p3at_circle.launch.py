from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Você pode trocar qual launch do simulador incluir via argumento
    sim_launch_arg = DeclareLaunchArgument(
        'sim_launch',
        default_value='p3at_gazebo.launch.py',
        description='Arquivo de launch dentro de p3at_simulation/launch a ser incluído'
    )

    # Caminho: <share>/p3at_simulation/launch/<sim_launch>
    sim_launch_path = PathJoinSubstitution([
        FindPackageShare('p3at_simulation'),
        'launch',
        LaunchConfiguration('sim_launch')
    ])

    include_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch_path)
    )

    controller = Node(
        package='p3at_simulation',
        executable='p3at_dqn_controller',         # entry point do setup.py
        name='p3at_controller_node',
        output='screen',
        parameters=[{'cmd_vel_topic': '/cmd_vel'}],  # tópico que você confirmou
    )

    return LaunchDescription([
        sim_launch_arg,
        include_sim,
        controller,
    ])

