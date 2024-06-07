#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String, Int32
import time
import math
# Qos durability
from rclpy.qos import QoSProfile
from rclpy.qos import DurabilityPolicy

class TelloReference(Node):
  def __init__(self) -> None:
    super().__init__('tello_reference')
    
    # Create qos profile with durability policy
    qos_profile = QoSProfile(depth=10)
    qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL

    # Create publisher
    #self.reference_publisher = self.create_publisher(PoseStamped, 'tello/reference', 10)
    self.reference_publisher = self.create_publisher(PoseStamped, 'tello/reference/position', qos_profile)

    # Initialize variables
    self.reference_pose = PoseStamped()

    self.timer = self.create_timer(0.1, self.timer_callback)
    self.time = 0

  def timer_callback(self) -> None:
    self.reference_pose.pose.position.x = 0.5 * math.sin(self.time)
    self.reference_pose.pose.position.y = 0.5 * math.cos(self.time)
    self.reference_pose.pose.position.z = 1+0.5 * math.sin(self.time/2)
    self.reference_pose.pose.orientation.x = 0.0
    self.reference_pose.pose.orientation.y = 0.0
    self.reference_pose.pose.orientation.z = 0.0
    self.reference_pose.pose.orientation.w = 1.0
    self.time += 0.1

    self.reference_publisher.publish(self.reference_pose)

def main(args=None) -> None:
    print('Starting tello reference node...')
    rclpy.init(args=args)
    tello_reference = TelloReference()
    rclpy.spin(tello_reference)
    tello_reference.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
