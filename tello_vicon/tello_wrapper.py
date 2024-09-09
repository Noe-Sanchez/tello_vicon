#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Twist
from std_msgs.msg import String, Int32
import time
from djitellopy import Tello

class TelloWrapper(Node):
  def __init__(self) -> None:
    super().__init__('tello_wrapper')

    # Create subscribers
    self.control_subscriber = self.create_subscription(Twist, "/tello/control/uaux", self.control_input_callback, 10)
    self.enable_susbcriber = self.create_subscription(String, "/tello/enable", self.enable_callback, 10)

    self.battery_publisher = self.create_publisher(Int32, "/tello/battery", 10)

    # Initialize variables
    self.enable = False
    self.battery = Int32()
    self.control_input = Twist()

    # Create a timer to publish control commands
    self.battery_timer = self.create_timer(1, self.battery_callback)
    self.control_timer = self.create_timer(0.01, self.control_callback)

    # Initialize Tello
    #self.tello = Tello("192.168.0.139", 8889)
    self.tello = Tello("10.15.232.94", 8889)
    self.tello.connect()
    self.tello.set_speed(100)
    #self.tello.end()
    #self.tello = Tello("192.168.0.148", 8889)
    #self.tello.connect()

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

  def control_input_callback(self, msg):
    self.control_input = msg    

  def control_callback(self):
    if self.enable:
      msg = self.control_input
      #self.tello.send_rc_control(-int(cy), int(cx), int(cz), 0)
      #self.tello.send_rc_control(-msg.linear.x, msg.linear.y, msg.linear.z, 0)
      self.tello.send_rc_control(int(msg.linear.x), int(msg.linear.y), int(msg.linear.z), int(msg.angular.z))
      print("Control: ", msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)

def main(args=None) -> None:
    print('Starting tello wrapper node...')
    rclpy.init(args=args)
    tello_wrapper = TelloWrapper()
    rclpy.spin(tello_wrapper)
    tello_wrapper.tello.end()
    tello_wrapper.destroy_node()
    print('Ending tello connection')
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
