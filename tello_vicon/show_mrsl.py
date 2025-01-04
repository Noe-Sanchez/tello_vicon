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
    super().__init__('show_mrsl')
    
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
    self.time = 0

    self.follower_pose_list = []
    self.follower_velocity_list = []
    for i in range(self.num_drones):
      self.follower_pose_list.append(Pose())
      self.follower_velocity_list.append(Pose())

  def timer_callback(self) -> None:
    if self.time < 5:
      # Leader
      self.leader_pose.header.stamp = self.get_clock().now().to_msg()
      self.leader_pose.pose.position.x = 0.0
      self.leader_pose.pose.position.y = 0.0
      self.leader_pose.pose.position.z = 1.0 
      self.leader_pose.pose.orientation.x = 0.0
      self.leader_pose.pose.orientation.y = 0.0
      self.leader_pose.pose.orientation.z = 0.0 
      self.leader_pose.pose.orientation.w = 1.0
       
      self.leader_velocity.header.stamp = self.get_clock().now().to_msg()
      self.leader_velocity.twist.linear.x = 0.0
      self.leader_velocity.twist.linear.y = 0.0
      self.leader_velocity.twist.linear.z = 0.0
      self.leader_velocity.twist.angular.x = 0.0
      self.leader_velocity.twist.angular.y = 0.0
      self.leader_velocity.twist.angular.z = 0.0 
  
      # Iterate over all drones
      for i in range(self.get_parameter('num_drones').value):
        self.follower_pose_list[i].position.x = math.cos(i*math.pi/2)
        self.follower_pose_list[i].position.y = math.sin(i*math.pi/2)
        self.follower_pose_list[i].position.z = 0.0 
        self.follower_pose_list[i].orientation.x = 0.0
        self.follower_pose_list[i].orientation.y = 0.0
        self.follower_pose_list[i].orientation.z = 0.0
        self.follower_pose_list[i].orientation.w = 1.0
  
        self.follower_velocity_list[i].position.x = 0.0 
        self.follower_velocity_list[i].position.y = 0.0
        self.follower_velocity_list[i].position.z = 0.0
        self.follower_velocity_list[i].orientation.x = 0.0
        self.follower_velocity_list[i].orientation.y = 0.0
        self.follower_velocity_list[i].orientation.z = 0.0
        self.follower_velocity_list[i].orientation.w = 0.0 

    elif self.time < 15:
      self.leader_pose.pose.position.x = -0.5
      self.leader_pose.pose.position.y = -0.5
      print('Corner left down')

    elif self.time < 25:
      self.leader_pose.pose.position.x = 0.5
      self.leader_pose.pose.position.y = -0.5
      print('Corner right down')

    elif self.time < 35:
      self.leader_pose.pose.position.x = 0.5
      self.leader_pose.pose.position.y = 0.5
      print('Corner right up')

    elif self.time < 45:
      self.leader_pose.pose.position.x = -0.5
      self.leader_pose.pose.position.y = 0.5
      print('Corner left up')

    elif self.time < 55:
      self.leader_pose.pose.position.x = 0.0
      self.leader_pose.pose.position.y = 0.0
      print('Center')

    elif self.time < 65:
      self.follower_pose_list[0].position.x =  1.45
      self.follower_pose_list[2].position.x = -1.45
      self.follower_pose_list[1].position.y =  1.0
      self.follower_pose_list[3].position.y = -1.0
      print('Separation')

    elif self.time < 75:
      self.follower_pose_list[1].position.y = -1.0
      self.follower_pose_list[3].position.y =  1.0
      print('Crossover')

    elif self.time < 94.2:
      self.follower_pose_list[0].position.x =  1.0
      self.follower_pose_list[2].position.x = -1.0
      print('Reconvergence')

    elif self.time < 116.18:
      self.leader_pose.pose.orientation.z = math.sin(self.time/32)
      self.leader_pose.pose.orientation.w = math.cos(self.time/32)
      self.follower_pose_list[0].position.x =  1.25
      self.follower_pose_list[2].position.x = -1.25
      print('Rotate')

    elif self.time < 135:
      self.follower_pose_list[0].position.z = -0.15
      self.follower_pose_list[1].position.z =  0.0
      self.follower_pose_list[2].position.z =  0.15
      self.follower_pose_list[3].position.z =  0.3
      print('Altitude')

    elif self.time < 145:
      self.leader_pose.pose.position.x = -0.5
      print('Down spiral')

    elif self.time < 155:
      self.leader_pose.pose.position.x = 0.5
      print('Top spiral')

    elif self.time < 175:
      self.leader_pose.pose.position.x = 0.0
      self.leader_pose.pose.position.y = 0.0
      self.follower_pose_list[0].position.x =  1.45
      self.follower_pose_list[2].position.x = -1.45
      self.follower_pose_list[1].position.y =  1.0
      self.follower_pose_list[3].position.y = -1.0
      self.follower_pose_list[0].position.z =  0.0
      self.follower_pose_list[1].position.z =  0.3
      self.follower_pose_list[2].position.z =  0.0
      self.follower_pose_list[3].position.z =  -0.3
      print('Redis')

    elif self.time < 195:
      self.follower_pose_list[1].position.y = -1.0
      self.follower_pose_list[3].position.y =  1.0
      print('Z Crossover')
    
    elif self.time < 210:
      self.follower_pose_list[0].position.x =  1.0
      self.follower_pose_list[2].position.x = -1.0
      self.follower_pose_list[0].position.z =  0.0
      self.follower_pose_list[1].position.z =  0.0
      self.follower_pose_list[2].position.z =  0.0
      self.follower_pose_list[3].position.z =  0.0
      print('Reconvergence')
      


    #elif self.time < 165:
    #  self.leader_pose.pose.position.x = 0.0
    #  self.leader_pose.pose.position.y = 0.0



    self.leader_pose.header.frame_id = 'world'
    self.leader_pose_publisher.publish(self.leader_pose)
    self.leader_velocity.header.frame_id = 'world'
    self.leader_velocity_publisher.publish(self.leader_velocity)
    
    self.formation_definition.header.stamp = self.get_clock().now().to_msg()
    self.formation_definition.poses = self.follower_pose_list
    self.formation_definition.header.frame_id = 'leader'

    self.formation_definition_publisher.publish(self.formation_definition)

    self.formation_dot_definition.header.stamp = self.get_clock().now().to_msg()
    self.formation_dot_definition.poses = self.follower_velocity_list
    self.formation_dot_definition.header.frame_id = 'leader'

    self.formation_dot_definition_publisher.publish(self.formation_dot_definition)

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
