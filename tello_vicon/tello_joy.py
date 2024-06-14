#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Twist
from std_msgs.msg import String, Int32
import time
from djitellopy import Tello
import pygame

class TelloWrapper(Node):
  def __init__(self) -> None:
    super().__init__('tello_wrapper')

    # Create subscribers
    self.enable_susbcriber = self.create_subscription(String, "/tello/enable", self.enable_callback, 10)

    self.battery_publisher = self.create_publisher(Int32, "/tello/battery", 10)
    self.joy_publisher = self.create_publisher(Twist, "/tello/joy", 10)

    # Initialize variables
    self.enable = False
    self.battery = Int32()
    self.joy = Twist()
    
    pygame.init()
    self.joystick = pygame.joystick.Joystick(0)
    self.joystick.init()

    # Create a timer to publish control commands
    self.battery_timer = self.create_timer(1, self.battery_callback)
    self.control_timer = self.create_timer(0.1, self.control_callback)

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

  def enable_callback(self, msg):
    self.enable = True if msg.data == "True" else False

  # Timer Callbacks
  def battery_callback(self):
    battery = self.tello.get_battery()
    self.battery.data = battery
    self.battery_publisher.publish(self.battery)

  def control_callback(self):
    if self.enable:
      #self.tello.send_rc_control(-int(cy), int(cx), int(cz), 0)
      #self.tello.send_rc_control(-msg.linear.x, msg.linear.y, msg.linear.z, 0)
      roll = 0
      pitch = 0
      yaw = 0
      throttle = 0
      event = pygame.event.poll()
      roll     = self.joystick.get_axis(0)*100
      pitch    = -self.joystick.get_axis(1)*100
      yaw      = self.joystick.get_axis(3)*100
      throttle = self.joystick.get_axis(2)*100
      
      self.tello.send_rc_control(int(roll), int(pitch), int(throttle), int(yaw))
      self.joy.linear.x = float(roll) 
      self.joy.linear.y = float(pitch)
      self.joy.linear.z = float(throttle)
      self.joy.angular.z = float(yaw)

      self.joy_publisher.publish(self.joy)            

def main(args=None) -> None:
    print('Starting tello wrapper node...')
    rclpy.init(args=args)
    tello_wrapper = TelloWrapper()
    rclpy.spin(tello_wrapper)
    tello_wrapper.tello.end()
    pygame.quit()
    tello_wrapper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
