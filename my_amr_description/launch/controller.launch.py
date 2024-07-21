from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    path1 = "/home/kakashi/catkin_ws/src/my_amr_description/launch/controller.yaml"
    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'use_sim_time': True}],
            arguments=[path1],
            output='screen'
        ),
        # Spawner for joint position controllers
        Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=['Revolute_5_position_controller', 'Revolute_6_position_controller', 'joint_state_controller'],
            namespace='my_amr',
        ),
        # Robot state publisher
    ])
