#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from rover_interfaces.msg import MotorControl
from rover_interfaces.msg import OdomInfo

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

    _fl_current_speed = 0.0
    _fr_current_speed = 0.0
    _bl_current_speed = 0.0
    _br_current_speed = 0.0

    _wheel_base = 1;

    axes = Joy().axes

    def __init__(self):
        super().__init__("controller_node")

        #create the publisher
        self.motor_control = self.create_publisher(MotorControl, 'motor_control', 10)
        self.create_timer(0.001, self.motor_callback)#sends every 0.5 seconds, can be changed
    
        #create the subscribers
        self.joy_callback = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.velocity_callback = self.create_subscription(Twist, 'cmd_vel', self.velocity_callback, 10)
        self.odom_callback = self.create_subscription(OdomInfo, 'odom_info', self.odom_callback, 10)

    #function for the subscriber for /joy
    def joy_callback(self, msg):
        """
        axes[0] - x axis of left stick
        axes[1] - y axis of left stick

        axes[3] - x axis of right stick
        axes[4] - y axis of right stick 
        
        Range is [0,1]
        """
        # array of floats
        self.axes = msg.axes
        #array of ints
        self.buttons = msg.buttons
        
        #calculate the wheel speed in m/s
        #left_speed = linear - angular*self._wheel_base/2
        #right_speed = linear + angular*self._wheel_base/2

        #this is where the closed loop comes in. until we know what kind of data
        #the encoders will return, it's an open loop

        # to go forwards, the stick is pointed up
        # this means that axes[3] is = 0 while axes[4] > 0
        # and consequently left speeds needs to match right speeds at axes[4]
        if (msg.axes[3] == 0.0) & (msg.axes[4] > 0.0):
            self._fl_speed = msg.axes[4]
            self._bl_speed = msg.axes[4]
            self._fr_speed = msg.axes[4]
            self._br_speed = msg.axes[4]

        # to go backwards, the stick is pointed down
        # this means that axes[3] is = 0 while axes[4] < 0
        # and consequently left speeds need to match right speeds at axes[4]
        elif (msg.axes[3] == 0.0) & (msg.axes[4] < 0.0):
            self._fl_speed = msg.axes[4]
            self._bl_speed = msg.axes[4]
            self._fr_speed = msg.axes[4]
            self._br_speed = msg.axes[4]
        # to go left, the stick is pointed left
        # this means that axes[3] is > 0 while axes[4] = 0
        # and consequently right speeds need to be the opposite of left speeds at axes[3]
        elif (msg.axes[4] == 0.0) & (msg.axes[3] > 0.0):
            self._fl_speed = msg.axes[3]*-1.0
            self._bl_speed = msg.axes[3]*-1.0
            self._fr_speed = msg.axes[3]
            self._br_speed = msg.axes[3]

        # to go right, the stick is pointed right
        # this means that axes[3] is < 0 while axes[4] = 0
        # and consequently right speeds need to be the opposite of left speeds at axes[3]
        elif (msg.axes[4] == 0.0) & (msg.axes[3] < 0.0):
            self._fl_speed = msg.axes[3]*-1.0
            self._bl_speed = msg.axes[3]*-1.0
            self._fr_speed = msg.axes[3]
            self._br_speed = msg.axes[3]

        #otherwise we can pretty much combine them into a vector
        else:
            # slight turn to right
            # want left wheels to be spinning faster than right wheels
            if (msg.axes[3] < 0.0) & (msg.axes[4] > 0.0):
                self._fl_speed = abs(msg.axes[3])
                self._bl_speed = abs(msg.axes[3])
                self._fr_speed = msg.axes[4]
                self._br_speed = msg.axes[4]

            # slight turn to left
            # want right wheels to be spinning faster than left wheels
            elif (msg.axes[3] > 0.0) & (msg.axes[4] > 0.0):
                self._fl_speed = msg.axes[3]
                self._bl_speed = msg.axes[3]
                self._fr_speed = msg.axes[4]
                self._br_speed = msg.axes[4]


            #all above doesn't work correctly for slight turns. ish. idk
    
    #function for the subscriber for /cmd_vel
    def velocity_callback(self, msg):
        
        #extract the linear and angular velocities
        linear = msg.linear.x
        angular = msg.angular.z

        #calculate wheel speeds in m/s
        left_speed = linear - angular*self._wheel_base/2
        right_speed = linear + angular*self._wheel_base/2

        #PID loop goes here.

        self._fl_speed = left_speed
        self._bl_speed = left_speed
        self._fr_speed = right_speed
        self._br_speed = right_speed


    #function for the subscriber for /odom_info
    def odom_callback(self, msg):
        _fl_current_speed = msg.fl
        _fr_current_speed = msg.fr
        _bl_current_speed = msg.bl
        _br_current_speed = msg.br

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
