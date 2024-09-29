from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tello_vicon',
            executable='simple_dynamics_node',
            name='simple_node'
        ),
        Node(
            package='tello_vicon',
            executable='asmc_node',
            name='asmc_node'
            )]) 
