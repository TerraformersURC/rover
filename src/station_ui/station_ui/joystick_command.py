#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from comms_interfaces.msg import MotorControl

DEADZONE = 0.05
LIN_AXIS = 1
ANG_AXIS = 3
YAW_AXIS = 6
PITCH_AXIS = 7

MAX_SPEED = 1
TURN_SCALING = 0.5
YAW_SCALING = 1.0
PITCH_SCALING = 1.0

T_PAD_MAX_X = 1919
T_PAD_MAX_Y = 942

MAX_SPEED = 1.0 # CHANGE TO OUR MAX SPEED

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
    
    def __init__(self):
        super().__init__("controller_node")
        self.declare_parameter('max_speed', MAX_SPEED)
        self.declare_parameter('turn_scaling', TURN_SCALING)
        self.max_speed = self.get_parameter('max_speed').value
        self.turn_scaling = self.get_parameter('turn_scaling').value
        self.yaw_scaling = YAW_SCALING
        self.pitch_scaling = PITCH_SCALING
        self.output = [0.0, 0.0, 0.0, 0.0, 0, 0]
        self.current_rotation = [0, 0]

        # create the publisher
        self.motor_control = self.create_publisher(MotorControl, 'motor_control', 10)
        self.create_timer(0.1, self.motor_callback)
    
        # create the subscribers
        self.velocity_callback = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
    
    #function for the publisher to /motor_control
    def motor_callback(self):
        msg = MotorControl()
        msg.fl = self.output[0]
        msg.bl = self.output[1]
        msg.fr = self.output[2]
        msg.br = self.output[3]
        msg.yaw = self.output[4]
        msg.pitch = self.output[5]
        self.motor_control.publish(msg)
        
    def joy_callback(self, joy_msg):
      # BOTH ARE FLIPPED
      x = self.joy_scaling(-joy_msg.axes[LIN_AXIS])
      z = self.turn_scaling * self.joy_scaling(-joy_msg.axes[ANG_AXIS])
      yaw_diff = (self.yaw_scaling * joy_msg.axes[YAW_AXIS])
      pitch_diff = (self.pitch_scaling * joy_msg.axes[PITCH_AXIS])
      self.current_rotation = [
        max(self.current_rotation[0] + yaw_diff, 225),
        max(self.current_rotation[1] + pitch_diff, 225),
      ]
      pitch = self.yaw_scaling * joy_msg.axes[PITCH_AXIS]
      yaw = self.yaw_scaling * joy_msg.axes[YAW_AXIS]
      
      self.output = list(map(self.vel_clip, [
        x - z, x - z, 
        x + z, x + z,
      ])) + list(map(int, self.current_rotation))
    
    def joy_scaling(self, x):
        return 0.0 if abs(x) < DEADZONE else x / (1.0 - DEADZONE)
    
    def vel_clip(self, x):
        return float(clip(x, -self.max_speed, self.max_speed))

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
