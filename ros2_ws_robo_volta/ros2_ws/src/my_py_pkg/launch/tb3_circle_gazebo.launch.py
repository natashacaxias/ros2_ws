from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Gazebo + TurtleBot3
    tb3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'launch', 'turtlebot3_world.launch.py'
            )
        )
    )

    circle = Node(
        package='my_py_pkg',
        executable='turtle_controller',   # o mesmo que você já usa
        output='screen',
        remappings=[('/turtle1/cmd_vel', '/cmd_vel')]
    )

    return LaunchDescription([tb3_gazebo, circle])

