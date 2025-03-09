#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Twist, TwistStamped, PoseArray, Pose  
from std_msgs.msg import String, Int32
import time
import math
import os
import pathlib

class TelloDescription(Node):
  def __init__(self) -> None:
    super().__init__('tello_description')
    
    # Get num_drones from parameter
    self.declare_parameter('num_drones', 1)
    self.num_drones = self.get_parameter('num_drones').value

    self.robot_description = String()
    # Use provided absolute path 
    self.description = pathlib.Path("/root/mrsl_ws/src/tello_vicon/urdf/tello.urdf").read_text()

    # Create publishers
    self.description_pubs = []
    for i in range(self.num_drones):
      self.description_pubs.append(self.create_publisher(String, f'tello_{i}/description', 10))

    self.get_logger().info('Tello description published')
    
    self.publish_timer = self.create_timer(1.0, self.publish_description)

  def publish_description(self) -> None:
    for i in range(self.num_drones):
      #self.robot_description.data = self.description.replace('rgba="0.3 0.0 1.0 1.0"','rgba="' + str(round(i/(self.num_drones-1),1)) + ' 0.0 ' + str(round(1-(i/(self.num_drones-1)), 1)) + ' 1.0"' )
      #self.robot_description.data = self.description.replace('rgba="0.3 0.0 1.0 1.0"','rgba="' + str(round(i/(self.num_drones-1),1)) + ' ' + str(round(1-(i/(self.num_drones-1)), 1)) + ' 0.0 1.0"' )
      self.robot_description.data = self.description.replace('rgba="0.3 0.0 1.0 1.0"','rgba="0.0 ' + str(round(i/(self.num_drones-1), 2)) + ' ' + str(round(1-(i/(self.num_drones-1)), 2)) + ' 1.0"' )
      #self.robot_description.data = self.description.replace('rgba="0.3 0.0 1.0 1.0"','rgba="' + str(i/(self.num_drones-1)) + ' 0.0 ' + str(1-(i/(self.num_drones-1))) + ' 1.0"' )
      #self.robot_description.data = self.description.replace('"tello"','"tello' + str(i)+'"')
      self.robot_description.data = self.robot_description.data.replace('"tello"','"tello' + str(i)+'"')
      self.description_pubs[i].publish(self.robot_description)
      #print('Published tello description')

 #   self.publish_timer = self.create_timer(1.0, self.publish_description)

#  def publish_description(self) -> None: 
#    for i in range(self.num_drones):


def main(args=None) -> None:
    print('Publishing tello description...')
    rclpy.init(args=args)
    tellod = TelloDescription()
    rclpy.spin(tellod)
    tello_reference.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
