#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Twist
from std_msgs.msg import String, Int32
import time
import sys
from djitellopy import TelloSwarm
import functools

# Import qos 
from rclpy.qos import QoSProfile

class TelloWrapper(Node):
  def __init__(self) -> None:
    super().__init__('tello_wrapper')

    #namespace = sys.argv[-1]
    #print("Namespace: ", namespace)
    
    # Declare and get num_drones parameter
    self.declare_parameter('num_drones', 0)
    self.num_drones = self.get_parameter('num_drones').value
    #print("Num drones: ", self.num_drones)
    self.get_logger().info('Number of drones: %d' % self.num_drones)
    
    #namespace = "bro"

    self.control_subscribers = []
    self.control_inputs = []

    # No queue size in qos_profile
    qos_profile = QoSProfile(depth=10, history=1, reliability=2, durability=2)

    # Create subscribers
    #self.control_subscriber = self.create_subscription(Twist, namespace + "/tello/control/uaux", self.control_input_callback, 10)
    for i in range(self.num_drones):
      #self.control_subscribers.append(self.create_subscription(Twist, "/tello_" + str(i) + "/tello/control/uaux", functools.partial(self.control_input_callback, i=i), 10))
      self.control_subscribers.append(self.create_subscription(Twist, "/tello_" + str(i) + "/tello/control/uaux", functools.partial(self.control_input_callback, i=i), qos_profile))
      self.control_inputs.append(Twist())

    self.enable_susbcriber = self.create_subscription(String, "master/enable", self.enable_callback, 10)


    #self.battery_publisher = self.create_publisher(Int32, namespace + "/tello/battery", 10)

    #self.echo_subscriber =  self.create_subscription(String, "/tello/echo1", functools.partial(self.echo_callback, name="tello 1"), 10)
    #self.echo_subscriber2 = self.create_subscription(String, "/tello/echo2", functools.partial(self.echo_callback, name="tello 2"), 10)

    # Initialize variables
    #self.enable = False
    self.enable = True 
    #self.battery = Int32()
    #self.control_input = Twist()

    # Create a timer to publish control commands
    #self.battery_timer = self.create_timer(1, self.battery_callback)
    self.control_timer = self.create_timer(0.01, self.control_callback)
    
    TelloIps = []
    for i in range(self.num_drones):
      if i == 0:
        TelloIps.append("10.15.232.96")
      if i == 1:
        TelloIps.append("10.15.232.94")
      if i == 2:
        TelloIps.append("10.15.232.80")
      if i == 3:
        TelloIps.append("10.15.232.63")
      #TelloIps.append("10.15.232." + str(96 + i))

    self.swarm = TelloSwarm.fromIps(TelloIps)
    self.swarm.connect()
    self.swarm.set_speed(100)
    self.swarm.takeoff()

    # Initialize Tello

    #self.tello = Tello("192.168.0.139", 8889)
    #self.tello = Tello("10.15.232.94", 8889)
    #self.tello.connect()
    #self.tello.set_speed(100)
    #self.tello.turn_motor_on()
    #time.sleep(5)
    #self.tello.turn_motor_off()
    #self.tello.end()
    #self.tello = Tello("192.168.0.148", 8889)
    #self.tello.connect()

    #battery = self.tello.get_battery()

    #print("Battery: ", battery)

    #if battery < 20:
    #  print("Battery low: ", battery)
    #  self.tello.end()
    #  exit()
    #else:
    #  self.tello.takeoff()
    #  print("Takeoff")

  #def echo_callback(self, msg, name):
  #  self.get_logger().info('Same callback for %s: "%s"' % (name, msg.data))

  def enable_callback(self, msg):
    self.enable = True if msg.data == "True" else False

  # Timer Callbacks
  #def battery_callback(self):
    #battery = self.tello.get_battery()
    #self.battery.data = battery
    #self.battery_publisher.publish(self.battery)
  #  pass

  #def control_input_callback(self, msg):
  #  self.control_input = msg    
  
  def control_input_callback(self, msg, i):
    self.control_inputs[i] = msg

  def control_callback(self):
    for i in range(self.num_drones):
      if self.enable:
        self.swarm.parallel(lambda i, tello: tello.send_rc_control(int(self.control_inputs[i].linear.x), int(self.control_inputs[i].linear.y), int(self.control_inputs[i].linear.z), int(self.control_inputs[i].angular.z)))
        #print("Control: ", self.control_inputs[i].linear.x, self.control_inputs[i].linear.y, self.control_inputs[i].linear.z, self.control_inputs[i].angular.z)
      #things
      else:
        self.swarm.parallel(lambda i, tello: tello.send_rc_control(0, 0, 0, 0))
        #print("Control: ", 0, 0, 0, 0)
      #self.tello.send_rc_control(-int(cy), int(cx), int(cz), 0)
      #self.tello.send_rc_control(-msg.linear.x, msg.linear.y, msg.linear.z, 0)
      
      #self.tello.send_rc_control(int(msg.linear.x), int(msg.linear.y), int(msg.linear.z), int(msg.angular.z))
      #print("Control: ", msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)

def main(args=None) -> None:
    print('Starting tello wrapper node...')
    rclpy.init(args=args)
    tello_wrapper = TelloWrapper()
    rclpy.spin(tello_wrapper)
    #tello_wrapper.tello.send_rc_control(0, 0, 0, 0)
    #tello_wrapper.tello.end()
    tello_wrapper.swarm.parallel(lambda i, tello: tello.send_rc_control(0, 0, 0, 0))
    tello_wrapper.swarm.land()
    tello_wrapper.swarm.end()
    tello_wrapper.destroy_node()
    print('Ending tello connection')
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        #tello_wrapper.tello.send_rc_control(0, 0, 0, 0)
        tello_wrapper.swarm.parallel(lambda i, tello: tello.send_rc_control(0, 0, 0, 0))
        tello_wrapper.swarm.land()
        print(e)
