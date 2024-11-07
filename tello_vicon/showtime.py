#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Twist, TwistStamped, PoseArray, Pose  
from std_msgs.msg import String, Int32
import time
import math
# Qos durability
from rclpy.qos import QoSProfile
from rclpy.qos import DurabilityPolicy

class TelloReference(Node):
  def __init__(self) -> None:
    super().__init__('showtime')
    
    # Create qos profile with durability policy
    qos_profile = QoSProfile(depth=10)
    qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL

    # Get num_drones from parameter
    self.declare_parameter('num_drones', 1)
    self.num_drones = self.get_parameter('num_drones').value

    # Create publishers for each drone
    #self.reference_publisher = self.create_publisher(PoseStamped, 'tello/reference/pose', qos_profile)
    #self.reference_velocity_publisher = self.create_publisher(TwistStamped, 'tello/reference/velocity', qos_profile)
    self.pose_publisher_list = []
    self.velocity_publisher_list = []
    
    self.leader_pose_publisher     = self.create_publisher(PoseStamped,  'tello_leader/tello/estimator/pose', 10)
    self.leader_velocity_publisher = self.create_publisher(TwistStamped, 'tello_leader/tello/estimator/velocity', 10)
    self.leader_pose = PoseStamped()
    self.leader_velocity = TwistStamped()

    self.formation_definition_publisher = self.create_publisher(PoseArray, 'formation_definition', 10)
    self.formation_definition = PoseArray()

    self.timer = self.create_timer(0.01, self.timer_callback)
    self.time = 0

    self.follower_pose_list = []
    for i in range(self.num_drones):
      self.follower_pose_list.append(Pose())

  def timer_callback(self) -> None:
    # Leader
    self.leader_pose.header.stamp = self.get_clock().now().to_msg()
    self.leader_pose.pose.position.x = math.cos(self.time/16)
    self.leader_pose.pose.position.y = math.sin(self.time/16)
    self.leader_pose.pose.position.z = 1.0
    self.leader_pose.pose.orientation.x = 0.0
    self.leader_pose.pose.orientation.y = 0.0
    self.leader_pose.pose.orientation.z = math.sin(self.time*math.pi/(2*16))#0.71
    self.leader_pose.pose.orientation.w = math.cos(self.time*math.pi/(2*16))#0.71
    
    self.leader_pose.header.frame_id = 'world'

    self.leader_pose_publisher.publish(self.leader_pose)

    # Iterate over all drones
    for i in range(self.get_parameter('num_drones').value):
      self.follower_pose_list[i].position.x = math.cos(i*math.pi/2)*(math.sin(self.time/16)/3 + 0.7)
      self.follower_pose_list[i].position.y = math.sin(i*math.pi/2)*(math.sin(self.time/16)/3 + 0.7)
      self.follower_pose_list[i].position.z = 0.0
      self.follower_pose_list[i].orientation.x = 0.0
      self.follower_pose_list[i].orientation.y = 0.0
      self.follower_pose_list[i].orientation.z = 0.0
      self.follower_pose_list[i].orientation.w = 1.0

    self.formation_definition.header.stamp = self.get_clock().now().to_msg()
    self.formation_definition.poses = self.follower_pose_list

    self.formation_definition_publisher.publish(self.formation_definition)

    self.time += 0.01


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