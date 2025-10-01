from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import TextSubstitution

def generate_launch_description():
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world', default=PathJoinSubstitution([
        FindPackageShare('p3at_simulation'), 'worlds', 'simple_world.world'
    ]))
    urdf_file = LaunchConfiguration('urdf', default=PathJoinSubstitution([
        FindPackageShare('p3at_simulation'), 'urdf', 'p3at.urdf.xacro'
    ]))

    # Start Gazebo (Garden) with the world
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '-r', world_file],
        output='screen'
    )

    # Robot descriptions via xacro
    robot_description_1 = Command(['xacro ', urdf_file, ' name:=p3at_1'])
    robot_description_2 = Command(['xacro ', urdf_file, ' name:=p3at_2'])

    # Robot State Publisher nodes
    rsp1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_p3at_1',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time, 'robot_description': robot_description_1}
        ]
    )

    rsp2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_p3at_2',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time, 'robot_description': robot_description_2}
        ]
    )

    # Spawn two robots from /robot_description via ros_gz_sim
    spawn1 = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_p3at_1',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'p3at_1', '-x', '0', '-y', '0']
    )

    spawn2 = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_p3at_2',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'p3at_2', '-x', '2', '-y', '0']
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value=world_file),
        DeclareLaunchArgument('urdf', default_value=urdf_file),
        gz_sim,
        rsp1,
        rsp2,
        spawn1,
        spawn2,
    ])