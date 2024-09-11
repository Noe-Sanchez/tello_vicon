#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Twist, TwistStamped
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
    self.reference_publisher = self.create_publisher(PoseStamped, 'tello/reference/pose', qos_profile)
    self.reference_velocity_publisher = self.create_publisher(TwistStamped, 'tello/reference/velocity', qos_profile)

    # Initialize variables
    self.reference_pose = PoseStamped()
    self.reference_pose.header.frame_id = 'world'
    self.reference_velocity = TwistStamped()

    self.timer = self.create_timer(0.01, self.timer_callback)
    self.time = 0

  def timer_callback(self) -> None:
    self.reference_pose.pose.position.x = 0.5 * math.sin(self.time/4)
    self.reference_pose.pose.position.y = 0.5 * math.cos(self.time/4)
    self.reference_pose.pose.position.z = 1+0.5 * math.sin(self.time/8)
    self.reference_pose.pose.orientation.x = 0.0
    self.reference_pose.pose.orientation.y = 0.0
    self.reference_pose.pose.orientation.z = 0.71
    self.reference_pose.pose.orientation.w = 0.71

    #self.reference_velocity.linear.x = 0.25 *  0.5 * math.cos(self.time/4)
    #self.reference_velocity.linear.y = 0.25 * -0.5 * math.sin(self.time/4)
    #self.reference_velocity.linear.z = 0.125 * 0.5 * math.cos(self.time/8)
    #self.reference_velocity.angular.x = 0.0
    #self.reference_velocity.angular.y = 0.0
    #self.reference_velocity.angular.z = 0.0

    self.reference_velocity.twist.linear.x = 0.25 *  0.5 * math.cos(self.time/4)
    self.reference_velocity.twist.linear.y = 0.25 * -0.5 * math.sin(self.time/4)
    self.reference_velocity.twist.linear.z = 0.125 * 0.5 * math.cos(self.time/8)
    self.reference_velocity.twist.angular.x = 0.0
    self.reference_velocity.twist.angular.y = 0.0
    self.reference_velocity.twist.angular.z = 0.0

    self.time += 0.01

    self.reference_publisher.publish(self.reference_pose)
    self.reference_velocity_publisher.publish(self.reference_velocity)

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
