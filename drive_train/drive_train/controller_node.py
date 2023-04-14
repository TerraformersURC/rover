#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import geometry_msgs.msg
from geometry_msgs.msg import Twist

#the controller will publish to /vel
#and subscribe to /odom
class Controller(Node):

    def __init__(self):
        super().__init__("controller")

        #create the publisher for /vel
        self.publisher_ = self.create_publisher(Twist, 'vel', 10)
        self.counter_ = 0
        self.create_timer(1.0, self.timer_callback)

        #create the subscriber for /odom
        self.subscription = self.create_subscription(
            Float32,
            'odom',
            self.listener_callback,
            10
        )
        self.subscription

    #function for the publisher for /vel
    def timer_callback(self):
        msg = Twist()
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        self.publisher_.publish(msg)

    #function for the subscriber for /odom
    def listener_callback(self, msg):
        self.get_logger().info("Wheel odometry Reading: '%s'" % msg.data)
    

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
