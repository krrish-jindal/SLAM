import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import  LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import time
import xacro

 
 
def generate_launch_description():
 
  # Constants for paths to different files and folders
  gazebo_models_path = 'models'
  package_name = 'my_amr_description'
  robot_name_in_model = 'my_amr'
  rviz_config_file_path = 'launch/rviz.rviz'
  urdf_file_path = 'urdf/my_amr.xacro'
  world_file_path = 'world/bookstore.world'
  
  # Pose where we want to spawn the robot
  # spawn_x_val = '-1.928008'
  # spawn_y_val = '5.098231'
  # spawn_z_val = '0.019008'
  # spawn_yaw_val = '-1.825072'
 
  spawn_x_val = '0.0'
  spawn_y_val = '0.'
  spawn_z_val = '0.0'
  spawn_yaw_val = '0.0'
  ############ You do not need to change anything below this line #############
   
  # Set the path to different files and folders.  
  pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
  pkg_share = FindPackageShare(package=package_name).find(package_name)
  default_urdf_model_path = os.path.join(pkg_share, urdf_file_path)
  default_rviz_config_path = os.path.join(pkg_share, rviz_config_file_path)
  world_path = os.path.join(pkg_share, world_file_path)
  gazebo_models_path = os.path.join(pkg_share, "model")
  os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
  robot_description_config_ebot = xacro.process_file(default_urdf_model_path)
  robot_description_ebot = robot_description_config_ebot.toxml()
     

# Subscribe to the joint states of the robot, and publish the 3D pose of each link.    
  start_robot_state_publisher_cmd = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    output='both',
    parameters=[{'use_sim_time': True}],
    arguments=[default_urdf_model_path]
    )

  start_joint_state_publisher_cmd = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher')
 
  # Launch RViz
  start_rviz_cmd = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', default_rviz_config_path])
  print(f"RViz Config File Path: {default_rviz_config_path}")

  # Start Gazebo server
  start_gazebo_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    launch_arguments={'world': world_path}.items()
    )   

    
  # Start Gazebo client    
  start_gazebo_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))) 
  # Launch the robot
  
  
  spawn_entity_cmd = Node(
    package='gazebo_ros', 
    executable='spawn_entity.py',
    arguments=[
        '-entity', robot_name_in_model, 
        '-topic', 'robot_description',
        '-x', spawn_x_val,
        '-y', spawn_y_val,
        '-z', spawn_z_val,
        '-Y', spawn_yaw_val
        ],
        
    output='screen')
  
  static_tf_publisher_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0.0', '0.0', '0.0',
            '0.0', '0', '2.09', # No rotation for static transform
            'map', 'world'
        ],
        output='screen'
    )
  # Create the launch description and populate
  ld = LaunchDescription()

  # Add any actions
  ld.add_action(start_gazebo_server_cmd)
  ld.add_action(start_gazebo_client_cmd)
  # time.sleep(3)
  ld.add_action(spawn_entity_cmd)
  ld.add_action(start_robot_state_publisher_cmd)
  # ld.add_action(start_joint_state_publisher_cmd)
  ld.add_action(start_rviz_cmd)
  # ld.add_action(static_tf_publisher_cmd)
 
  return ld