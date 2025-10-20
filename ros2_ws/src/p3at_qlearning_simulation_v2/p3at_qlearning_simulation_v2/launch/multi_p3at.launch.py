from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Robô 1
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'p3at1',
                '-x', '0', '-y', '0', '-z', '0.1',
                '-file', '/home/robot/ros2_ws/src/p3at_qlearning_simulation_v2/p3at_qlearning_simulation_v2/urdf/pioneer3at.urdf'
            ],
            output='screen'
        ),

        # Robô 2
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'p3at2',
                '-x', '3', '-y', '3', '-z', '0.1',
                '-file', '/home/robot/ros2_ws/src/p3at_qlearning_simulation_v2/p3at_qlearning_simulation_v2/urdf/pioneer3at.urdf'
            ],
            output='screen'
        ),
    ])

