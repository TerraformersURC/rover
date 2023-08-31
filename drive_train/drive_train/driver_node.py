#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from std_msgs.msg import Float32
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from rover_interfaces.msg import MotorControl
from rover_interfaces.msg import OdomInfo
import serial, time
from .comm import encode
import numpy as np

arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=.1)
time.sleep(1)

# convert percentage to PWM value for motors
# speed is range [-1.5, 1.5] backwards:[-1.5, 0] forwards:[0, 1.5]
# PWM is range [0, 255] backwards:[0, 127] forwards:[127, 255]
def convertToPWM(speed_percent):
    #NewValue = (((OldValue - OldMin) * NewRange) / OldRange) + NewMin
    pwm = (((speed_percent+1.5)*255)/3)-1.5
    return int(pwm)

#the driver will subscribe to /motor_control
#and publish to /odom_info
class Driver(Node):

    #data pushed to Arduino
    _fl_speed = 0.0
    _fr_speed = 0.0
    _bl_speed = 0.0
    _br_speed = 0.0

    #data from encoder
    _fl_current_speed = 0.0
    _fr_current_speed = 0.0
    _bl_current_speed = 0.0
    _br_current_speed = 0.0

    #i'd like to convert these into parameters
    _timeout = 2.0 #how long should it go without new input
    _max_speed = 1.0 #max speed of motors
        
    def __init__(self):
        super().__init__("driver_node")

        #create the subscriber for /motor_control
        self.motor_callback = self.create_subscription(MotorControl, 'motor_control', self.motor_callback, 10)
        
        self.cmd_vel_callback = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        #create the publisher for /odom_info
        self.odom_info = self.create_publisher(OdomInfo, 'odom_info', 10)
        #send data every 0.5 seconds
        self.create_timer(0.5, self.odom_callback)


    #function for the publisher to /odom_info
    def odom_callback(self):
        msg = OdomInfo()

        #self._fl_current_speed = read the current speed of a wheel and store it
        
        #construct the msg
        msg.fl = self._fl_current_speed
        msg.fr = self._fr_current_speed
        msg.bl = self._bl_current_speed
        msg.br = self._br_current_speed
        self.odom_info.publish(msg)

    
    #function for the subscriber to /motor_control
    def motor_callback(self, msg):
        self._fl_speed = msg.fl
        self._fr_speed = msg.fr
        self._bl_speed = msg.bl
        self._br_speed = msg.br

        pwm = (((self._fl_speed + 1.5) * 800) / 3) + 1100
        self.get_logger().debug('My log message %d' % (pwm))
        arduino.write(int(pwm))

        speeds = np.array(np.absolute(np.multiply([self._fl_speed,self._fr_speed, self._bl_speed, self._br_speed], 255/1.5)).astype(int), dtype='uint8')
        #print(len(speeds))
        #arduino.write(encode(speeds,self._bl_speed>=0,self._br_speed>=0))
    
    def cmd_vel_callback(self, msg):
        pwm = (((msg.linear.x + 1.5) * 800) / 3) + 1100
        self.get_logger().debug('My log message %d' % (pwm))
        arduino.write(int(pwm))



def main(args=None):
    #initialize ros2 communications
    rclpy.init(args=args)

    #initialize node
    node = Driver()

    #keep the node alive, enable all callbacks
    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
