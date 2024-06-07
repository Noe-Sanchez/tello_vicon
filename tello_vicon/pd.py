#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String, Int32
import time
from djitellopy import Tello
import transforms3d.euler as euler
import transforms3d.quaternions as quat
import math

class TelloControl(Node):
  def __init__(self) -> None:
    super().__init__('tello_control')

    # Create subscribers
    self.vicon_subscriber = self.create_subscription(PoseStamped, "/vicon/tello2/tello2", self.vicon_callback, 10)
    self.reference_subscriber = self.create_subscription(PoseStamped, "/tello/reference", self.reference_callback, 10)
    self.enable_susbcriber = self.create_subscription(String, "/tello/enable", self.enable_callback, 10)

    self.battery_publisher = self.create_publisher(Int32, "/tello/battery", 10)
    self.error_publisher = self.create_publisher(PoseStamped, "/tello/error", 10)

    # Initialize variables
    self.vicon_pose = PoseStamped()
    self.reference_pose = PoseStamped()
    self.error = PoseStamped()
    self.previous_error = PoseStamped()
    self.enable = False
    self.battery = Int32()

    # Create a timer to publish control commands
    self.control_timer = self.create_timer(0.1, self.control_callback)
    self.battery_timer = self.create_timer(1, self.battery_callback)

    # Initialize Tello
    self.tello = Tello("192.168.0.148", 8889)
    self.tello.connect()

    battery = self.tello.get_battery()

    print("Battery: ", battery)

    if battery < 20:
      print("Battery low: ", battery)
      self.tello.end()
      exit()
    else:
      self.tello.takeoff()
      print("Takeoff")

    self.gains = [50, 50, 50, 50]

  # Subcription Callbacks
  def vicon_callback(self, msg):
    self.vicon_pose = msg

  def reference_callback(self, msg):
    self.reference_pose = msg

  def enable_callback(self, msg):
    self.enable = True if msg.data == "True" else False

  # Timer Callbacks
  def battery_callback(self):
    battery = self.tello.get_battery()
    self.battery.data = battery
    self.battery_publisher.publish(self.battery)

  def control_callback(self): 
    #cx = self.gains[0] * (self.reference_pose.pose.position.x - self.vicon_pose.pose.position.x)
    #cy = self.gains[1] * (self.reference_pose.pose.position.y - self.vicon_pose.pose.position.y)
    #cz = self.gains[2] * (self.reference_pose.pose.position.z - self.vicon_pose.pose.position.z)
    self.error.pose.position.x = self.reference_pose.pose.position.x - self.vicon_pose.pose.position.x
    self.error.pose.position.y = self.reference_pose.pose.position.y - self.vicon_pose.pose.position.y
    self.error.pose.position.z = self.reference_pose.pose.position.z - self.vicon_pose.pose.position.z
   
    # Make quaternions from msgs
    #orientation = [self.vicon_pose.pose.orientation.w, self.vicon_pose.pose.orientation.x, self.vicon_pose.pose.orientation.y, self.vicon_pose.pose.orientation.z]
    #desired_orientation = [self.reference_pose.pose.orientation.w, self.reference_pose.pose.orientation.x, self.reference_pose.pose.orientation.y, self.reference_pose.pose.orientation.z]
    #error_quat = quat.qmult(quat.qconjugate(orientation), desired_orientation)
    #orientation_euler = euler.quat2euler(orientation)
    #error_euler = euler.quat2euler(error_quat)
    #self.error.pose.orientation.z = error_euler[2]
    #print("Error: ", self.error.pose.orientation.z)

    cx = self.gains[0] * self.error.pose.position.x + 0.1 * (self.error.pose.position.x - self.previous_error.pose.position.x)
    cy = self.gains[1] * self.error.pose.position.y + 0.1 * (self.error.pose.position.y - self.previous_error.pose.position.y)
    cz = self.gains[2] * self.error.pose.position.z + 0.1 * (self.error.pose.position.z - self.previous_error.pose.position.z)
    #cyaw = self.gains[3] * self.error.pose.orientation.z + 0.1 * (self.error.pose.orientation.z - self.previous_error.pose.orientation.z)
    
    #cx = cx * math.cos(orientation_euler[2]) + cy * math.sin(orientation_euler[2])
    #cy = -cx * math.sin(orientation_euler[2]) + cy * math.cos(orientation_euler[2])

    #cyaw = self.gains[3] * (self.reference_pose.pose.orientation.z - orientation[2])
    #cyaw = self.gains[3] * (self.reference_pose.pose.orientation.z - self.vicon_pose.pose.orientation.z)

    #print("Unsaturated: ", cy, cx, cz, cyaw)
    #print("Unsaturated: ", cy, cx, cz, cyaw)
    print("Unsaturated: ", cy, cx, cz, 0)
    
    #print("Saturated: ", cy, cx, cz, cyaw)

    if self.enable:
      #self.tello.send_rc_control(-int(cy), int(cx), int(cz), -int(cyaw))
      self.tello.send_rc_control(-int(cy), int(cx), int(cz), 0)

    self.error_publisher.publish(self.error)
    self.previous_error = self.error

def main(args=None) -> None:
    print('Starting tello control node...')
    rclpy.init(args=args)
    tello_control = TelloControl()
    rclpy.spin(tello_control)
    tello_control.tello.end()
    tello_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
