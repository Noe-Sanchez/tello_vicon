from launch import LaunchDescription
from launch_ros.actions import Node
import sys

def generate_launch_description():
  num_drones = int(sys.argv[4:][0].split('=')[1])

  ld = []

  for i in range(num_drones):
    ld.append(Node(
      package='tello_vicon',
      executable='asmc_node',
      name='asmc_node_' + str(i),
      output='screen',
      parameters=[{'drone_id': i}],
      namespace='tello_' + str(i)))
      #remappings=[('/tello_estimator_pose', '/tello_estimator_pose_' + str(i)),
      #            ('/tello_estimator_velocity', '/tello_estimator_velocity_' + str(i)),
      #            ('/tello_reference_pose', '/tello_reference_pose_' + str(i)),
      #            ('/tello_reference_velocity', '/tello_reference_velocity_' + str(i))]
      #))
    ld.append(Node(
      package='tello_vicon',
      executable='simple_dynamics_node',
      name='simple_dynamics_node_' + str(i),
      output='screen',
      parameters=[{'drone_id': i}],
      namespace='tello_' + str(i)))
      #remappings=[('/tello_vicon_pose', '/tello_vicon_pose_' + str(i))]
      #))

  return LaunchDescription(ld)


