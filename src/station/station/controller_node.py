#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from rover_interfaces.msg import MotorControl
from rover_interfaces.msg import OdomInfo

DEADZONE = 0.05
TURN_SCALING = 0.75
LIN_AXIS = 1
ANG_AXIS = 3
MAX_SPEED = 1

T_PAD_MAX_X = 1919
T_PAD_MAX_Y = 942

MAX_SPEED = 1 # CHANGE TO OUR MAX SPEED

#sets the max and min speeds
def clip(value, min, max):
    if value < min:
        return min
    elif value > max:
        return max
    else:
        return value

#the controller subscribes to /joy, /cmd_vel, /and odom_info
#and publishes MotorControl messages to /motor_control using data from the joysticks
class Controller(Node):
    # FL, BL, FR, BR
    speeds = [0.0, 0.0, 0.0, 0.0]

    def __init__(self):
        super().__init__("controller_node")

        # create the publisher
        self.motor_control = self.create_publisher(MotorControl, 'motor_control', 10)
        self.create_timer(0.01, self.motor_callback) # sends every 0.5 seconds, can be changed
    
        # create the subscribers
        self.velocity_callback = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
    
    #function for the publisher to /motor_control
    def motor_callback(self):
        msg = MotorControl()
        msg.fl = self.speeds[0]
        msg.bl = self.speeds[1]
        msg.fr = self.speeds[2]
        msg.br = self.speeds[3]
        self.motor_control.publish(msg)
        
    def joy_callback(self, joy_msg):
      x = self.joy_scaling(joy_msg.axes[LIN_AXIS])
      z = TURN_SCALING * self.joy_scaling(joy_msg.axes[ANG_AXIS])
      
      self.speeds = map(self.vel_clip,
      [
        x + z, x + z,
        x - z, x - z
      ])
    
    def joy_scaling(self, x):
        return 0.0 if abs(x) < DEADZONE else x / (1.0 - DEADZONE)
    
    def vel_clip(self, x):
        return clip(x, -MAX_SPEED, MAX_SPEED)

def main(args=None):
    #initialize ros2 communications
    rclpy.init(args=args)

    #initialize node
    node = Controller()

    #keep the node alive, enable all callbacks
    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
