#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from interfaces.msg import MotorControl
from interfaces.msg import OdomInfo

#sets the max and min speeds
def _clip(value, min, max):
    if value < min:
        return min
    elif value > max:
        return max
    else:
        return value

#the controller subscribes to /joy, /cmd_vel, /and odom_info
#and publishes MotorControl messages to /motor_control using data from the joysticks
class Controller(Node):
    _fl_speed = 0.0
    _fr_speed = 0.0
    _bl_speed = 0.0
    _br_speed = 0.0

    def __init__(self):
        super().__init__("controller")

        #create the publisher
        self.motor_control = self.create_publisher(MotorControl, 'motor_control', 10)
        self.create_timer(0.001, self.motor_callback)#sends every 0.5 seconds, can be changed
    
        #create the subscribers
        self.velocity_callback = self.create_subscription(Twist, 'cmd_vel', self.velocity_callback, 10)
        
    #function for the subscriber for /cmd_vel
    def velocity_callback(self, msg):
        
        #extract the linear and angular velocities
        x = msg.linear.x
        z = msg.angular.z

        # Forward (x>0)
        # Backward (x<0)
        # Left (z>0)
        # Right (z<0)

        if (z > 0):
            left_speed = z*-1
            right_speed = z
        elif (z < 0):
            left_speed = z
            right_speed = z*-1
        else:
            left_speed = x
            right_speed = x


        self._fl_speed = left_speed
        self._bl_speed = left_speed
        self._fr_speed = right_speed
        self._br_speed = right_speed

    #function for the publisher to /motor_control
    def motor_callback(self):
        msg = MotorControl()
        msg.fl = self._fl_speed
        msg.bl = self._bl_speed
        msg.fr = self._fr_speed
        msg.br = self._br_speed
        self.motor_control.publish(msg)


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
