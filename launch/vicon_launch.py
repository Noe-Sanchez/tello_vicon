from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tello_vicon',
            namespace='tello',
            executable='differentiator_node',
            name='differentiator_node'
        ),
        Node(
            package='tello_vicon',
            namespace='tello',
            executable='tello_wrapper.py',
            name='tello_wrapper'
        ),
        Node(
            package='tello_vicon',
            namespace='tello',
            executable='asmc_node',
            name='asmc_node'
            )]) 
