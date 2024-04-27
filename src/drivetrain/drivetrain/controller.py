#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from interfaces.msg import MotorControl
from numpy import clip

DEADZONE = 0.05
TURN_SCALING = 0.75
LIN_AXIS = 1
ANG_AXIS = 3
MAX_SPEED = 1

# the controller subscribes to /joy, /cmd_vel, /and odom_info
# and publishes MotorControl messages to /motor_control using data from the joysticks
class Controller(Node):
    _fl_speed = 0.0
    _fr_speed = 0.0
    _bl_speed = 0.0
    _br_speed = 0.0

    def __init__(self):
        super().__init__("controller")

        # create the publisher
        # self.motor_publisher = self.create_publisher(MotorControl, 'motor_control', 10)
        # self.create_timer(0.01, self.motor_callback) # sends every 0.5 seconds, can be changed
    
        # create the subscribers
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        
    # function for the subscriber for /cmd_vel
    def velocity_callback(self, msg):
        
        #extract the linear and angular velocities
        x = self.joy_scaling(msg.axes[LIN_AXIS])
        z = TURN_SCALING * self.joy_scaling(msg.axes[ANG_AXIS])

        self._fl_speed = self.vel_clip(x + z)
        self._bl_speed = self.vel_clip(x + z)
        self._fr_speed = self.vel_clip(x - z)
        self._br_speed = self.vel_clip(x - z)

    #f unction for the publisher to /motor_control
    #def motor_callback(self):
        msg = MotorControl()
        msg.fl = self._fl_speed
        msg.bl = self._bl_speed
        msg.fr = self._fr_speed
        msg.br = self._br_speed
        self.motor_control.publish(msg)

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
