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
import random

class TelloReference(Node):
  def __init__(self) -> None:
    super().__init__('show_shart')
    
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
    
    self.leader_pose_publisher     = self.create_publisher(PoseStamped,  '/formation/leader/pose', 10)
    self.leader_velocity_publisher = self.create_publisher(TwistStamped, '/formation/leader/velocity', 10)
    self.leader_pose = PoseStamped()
    self.leader_velocity = TwistStamped()

    self.formation_definition_publisher = self.create_publisher(PoseArray, '/formation/definition', 10)
    self.formation_definition = PoseArray()
    self.formation_dot_definition_publisher = self.create_publisher(PoseArray, '/formation/velocity', 10)
    self.formation_dot_definition = PoseArray()

    self.timer = self.create_timer(0.01, self.timer_callback)

    self.follower_pose_list = []
    self.follower_velocity_list = []
    for i in range(self.num_drones):
      self.follower_pose_list.append(Pose())
      self.follower_velocity_list.append(Pose())

    self.scramble_vector = []
    for i in range(self.num_drones):
      print(i)
      # Apend random element
      while True:
        rand_val = random.randint(0, self.num_drones-1)
        if rand_val not in self.scramble_vector:
          self.scramble_vector.append(rand_val)
          break
      #if rand_val not in self.scramble_vector:
      #  self.scramble_vector.append(rand_val)
    print("New configuration: ", self.scramble_vector)

  def timer_callback(self) -> None:
    # Leader
    self.leader_pose.header.stamp = self.get_clock().now().to_msg()
    self.leader_pose.pose.position.x = 0.0
    self.leader_pose.pose.position.y = 0.0
    self.leader_pose.pose.position.z = 1.0
    self.leader_pose.pose.orientation.x = 0.0
    self.leader_pose.pose.orientation.y = 0.0
    self.leader_pose.pose.orientation.z = 0.0
    self.leader_pose.pose.orientation.w = 1.0 
    
    self.leader_pose.header.frame_id = 'world'
    self.leader_pose_publisher.publish(self.leader_pose)

      
    self.leader_velocity.header.stamp = self.get_clock().now().to_msg()
    self.leader_velocity.twist.linear.x  = 0.0 
    self.leader_velocity.twist.linear.y  = 0.0 
    self.leader_velocity.twist.linear.z  = 0.0
    self.leader_velocity.twist.angular.x = 0.0
    self.leader_velocity.twist.angular.y = 0.0
    self.leader_velocity.twist.angular.z = 0.0 
    
    self.leader_velocity.header.frame_id = 'world'
    self.leader_velocity_publisher.publish(self.leader_velocity)

    # Iterate over all drones
    for i in range(self.get_parameter('num_drones').value):
      #self.follower_pose_list[i].position.x = (i%4) - min(4, self.num_drones)/2 + 0.5 
      #self.follower_pose_list[i].position.y = float(math.floor((i+0.01)/4) - math.floor((self.num_drones-0.01)/4)/2)
      #self.follower_pose_list[i].position.z = 0.0
      #self.follower_pose_list[i].orientation.x = 0.0
      #self.follower_pose_list[i].orientation.y = 0.0
      #self.follower_pose_list[i].orientation.z = 0.0
      #self.follower_pose_list[i].orientation.w = 1.0
      self.follower_pose_list[self.scramble_vector[i]].position.x = (i%4) - min(4, self.num_drones)/2 + 0.5 
      self.follower_pose_list[self.scramble_vector[i]].position.y = float(math.floor((i+0.01)/4) - math.floor((self.num_drones-0.01)/4)/2)
      #self.follower_pose_list[self.scramble_vector[i]].position.x = 0.75*math.cos(i*math.pi/2)
      #self.follower_pose_list[self.scramble_vector[i]].position.y = 0.75*math.sin(i*math.pi/2)
      self.follower_pose_list[self.scramble_vector[i]].position.z = 0.0
      self.follower_pose_list[self.scramble_vector[i]].orientation.x = 0.0
      self.follower_pose_list[self.scramble_vector[i]].orientation.y = 0.0
      self.follower_pose_list[self.scramble_vector[i]].orientation.z = 0.0
      self.follower_pose_list[self.scramble_vector[i]].orientation.w = 1.0

      #self.follower_velocity_list[i].position.x = 0.0
      #self.follower_velocity_list[i].position.y = 0.0
      #self.follower_velocity_list[i].position.z = 0.0
      #self.follower_velocity_list[i].orientation.x = 0.0
      #self.follower_velocity_list[i].orientation.y = 0.0
      #self.follower_velocity_list[i].orientation.z = 0.0
      #self.follower_velocity_list[i].orientation.w = 0.0 

      self.follower_velocity_list[self.scramble_vector[i]].position.x = 0.0
      self.follower_velocity_list[self.scramble_vector[i]].position.y = 0.0
      self.follower_velocity_list[self.scramble_vector[i]].position.z = 0.0
      self.follower_velocity_list[self.scramble_vector[i]].orientation.x = 0.0
      self.follower_velocity_list[self.scramble_vector[i]].orientation.y = 0.0
      self.follower_velocity_list[self.scramble_vector[i]].orientation.z = 0.0
      self.follower_velocity_list[self.scramble_vector[i]].orientation.w = 0.0 

    self.formation_definition.header.stamp = self.get_clock().now().to_msg()
    self.formation_definition.poses = self.follower_pose_list
    self.formation_definition.header.frame_id = 'leader'

    self.formation_definition_publisher.publish(self.formation_definition)

    self.formation_dot_definition.header.stamp = self.get_clock().now().to_msg()
    self.formation_dot_definition.poses = self.follower_velocity_list
    self.formation_dot_definition.header.frame_id = 'leader'

    self.formation_dot_definition_publisher.publish(self.formation_dot_definition)

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
