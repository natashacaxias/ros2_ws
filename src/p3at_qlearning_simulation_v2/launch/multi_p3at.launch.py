from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('p3at_qlearning_simulation_v2')
    urdf_file = os.path.join(pkg_share, 'urdf', 'pioneer3at.urdf')

    # === Novo Gazebo (Harmonic) ===
    gz_pkg = get_package_share_directory('ros_gz_sim')
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()  # Mundo vazio
    )

    # === Spawn dos robôs ===
    spawn_p3at1 = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_p3at1',
        output='screen',
        arguments=[
            '-name', 'p3at1',
            '-x', '0', '-y', '0', '-z', '0.1',
            '-file', urdf_file
        ]
    )

    spawn_p3at2 = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_p3at2',
        output='screen',
        arguments=[
            '-name', 'p3at2',
            '-x', '3', '-y', '3', '-z', '0.1',
            '-file', urdf_file
        ]
    )

    # === Bridge para /cmd_vel e /odom ===
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        arguments=[
            '/model/p3at1/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/p3at1/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/model/p3at2/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/p3at2/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'
        ]
    )

    return LaunchDescription([
        gz_launch,
        spawn_p3at1,
        spawn_p3at2,
        bridge
    ])
