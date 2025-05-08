import rclpy
from rclpy.node import Node
from comms_interfaces.msg import MotorControl

class KeyboardCommand(Node):
  speeds = [0.0, 0.0, 0.0, 0.0]
  
  def __init__(self):
    super().__init__("keyboard_command")
  
    self.motor_control = self.create_publisher(MotorControl, 'motor_control', 10)
    self.create_timer(0.1, self.motor_callback)
    
  def motor_callback(self):
    msg = MotorControl()
    msg.fl = self.speeds[0]
    msg.bl = self.speeds[1]
    msg.fr = self.speeds[2]
    msg.br = self.speeds[3]
    self.motor_control.publish(msg)