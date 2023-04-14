#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from std_msgs.msg import Float32
import geometry_msgs.msg
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO # import the GPIO library
from rover_interfaces.msg import MotorControl
from rover_interfaces.msg import OdomInfo

#set the GPIO modes
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

#how many times to turn the pin on and off each second AKA duty cycle
FREQUENCY = 20

# convert percentage to PWM value for motors
# speed is range [-1, 1] backwards:[-1, 0] forwards:[0, 1]
# PWM is range [0, 255] backwards:[0, 127] forwards:[127, 255]
def convertToPWM(speed_percent):
    #NewValue = (((OldValue - OldMin) * NewRange) / OldRange) + NewMin
    pwm = (((speed_percent+1)*255)/2)+0
    return pwm


#represents a single motor
#each motor has its own PWM pin the ESC, as well as a pin for encoder input
#no idea what the encoder data is going to be
class Motor:
    def __init__(self, data_pin, encoder_pin):
        GPIO.setup(data_pin, GPIO.OUT) #ESC PWM
        GPIO.setup(encoder_pin, GPIO.IN) #Encoder

        self.pwm = GPIO.PWM(data_pin, FREQUENCY)

    #send a PWM signal to a motor to make it move
    def move(self, _speed):
        _speed_pwm = convertToPWM(_speed)
        self.pwm.start(_speed_pwm)

#the driver will subscribe to /motor_control
#and publish to /odom_info
class Driver(Node):

    #data pushed to PWM then to ESCs
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
    _wheel_base = 1.0 #distance between wheels
        
    def __init__(self):
        super().__init__("new_driver_node")

        #assign pins to motors and encoders. 
        #important to note that the RPi only has one hardware PWM pin
        self.fl_motor = Motor(1, 2)
        self.fr_motor = Motor(3, 4)
        self.bl_motor = Motor(5, 6)
        self.br_motor = Motor(7, 8)


        #create the subscriber for /motor_control
        self.motor_callback = self.create_subscription(MotorControl, 'motor_control', self.motor_callback, 10)
        
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
    
        self.fl_motor.move(msg.fl)
        self.fr_motor.move(msg.fr)
        self.bl_motor.move(msg.bl)
        self.br_motor.move(msg.br)


            # if we haven't received new commands for a while, we 
            # may have lost contact with rover so stop moving
           # self._delay = Timer.time_since_last_call
           # if _delay < self._timeout:
           #     keep the motors spinning at the same velocities
           # else:
           #     stop all the motors

def main(args=None):
    #initialize ros2 communications
    rclpy.init(args=args)

    #initialize node
    node = Driver()

    #keep the node alive, enable all callbacks
    rclpy.spin(node)

    #when closing out GPIO
    GPIO.cleanup()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
