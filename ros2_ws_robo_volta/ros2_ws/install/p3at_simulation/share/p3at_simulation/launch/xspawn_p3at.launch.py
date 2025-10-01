from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
import os
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
 pkg_share = get_package_share_directory('p3at_simulation')
 world_path = os.path.join(pkg_share, 'worlds', 'simple_world.world')
 urdf_path = os.path.join(pkg_share, 'urdf', 'p3at.urdf.xacro')
 # Argumentos para o launch file
 use_sim_time = LaunchConfiguration('use_sim_time', default='true')
 # Ações de lançamento

 # 1. Inicia o Gazebo Garden com o mundo
 gz_sim = IncludeLaunchDescription(
  PythonLaunchDescriptionSource(
      os.path.join(
         get_package_share_directory('ros_gz_sim'),
         'launch',
         'gz_sim.launch.py'
      )
   ),
   launch_arguments={'gz_args': world_path}.items()
 )
 
 # 2. Inicia o Robot State Publisher para o robô 1
 # Note que a variável 'robot_name' é passada via argumento para o xacro
 robot_state_publisher_1 = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher_1',
    output='screen',
    parameters=[{'use_sim_time': use_sim_time, 'robot_description': open(urdf_path).read()}],
    arguments=[{'robot_name': 'p3at_1'}]
 )
 
 # 3. Inicia o Robot State Publisher para o robô 2
 robot_state_publisher_2 = Node(
  package='robot_state_publisher',
  executable='robot_state_publisher',
  name='robot_state_publisher_2',
  output='screen',
  parameters=[{'use_sim_time': use_sim_time, 'robot_description': open(urdf_path).read()}],
  arguments=[{'robot_name': 'p3at_2'}]
 )
 
 # 4. Spawna os robôs no mundo
 spawn_entity_1 = Node(
  package='ros_gz_sim',
  executable='create',
  output='screen',
  arguments=['-topic', 'robot_description', '-name', 'p3at_1', '-x', '0', '-y', '0']
 )
 
 spawn_entity_2 = Node(
  package='ros_gz_sim',
  executable='create',
  output='screen',
  arguments=['-topic', 'robot_description', '-name', 'p3at_2', '-x', '2', '-y', '0']
 )
 
 return LaunchDescription([
  gz_sim,
  robot_state_publisher_1,
  robot_state_publisher_2,
  spawn_entity_1,
  spawn_entity_2
 ])
