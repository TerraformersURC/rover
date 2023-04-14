#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import geometry_msgs.msg
from geometry_msgs.msg import Twist

#the driver will subscribe to /vel
#and publish to /odom
class Driver(Node):

    def __init__(self):
        super().__init__("driver")

        #create the subscriber for /vel
        self.subscription = self.create_subscription(
            Twist,
            'vel',
            self.listener_callback,
            10
        )
        self.subscription

        #create the publisher for /odom
        self.publisher_ = self.create_publisher(Float32, 'odom', 10)
        self.counter_ = 0
        self.create_timer(1.0, self.timer_callback)

    #function for the subscriber for /vel
    def listener_callback(self, msg):
        self.get_logger().info("Linear x: '%s'" % msg.linear.x)
        self.get_logger().info("Linear y: '%s'" % msg.linear.y)
        self.get_logger().info("Linear z: '%s'" % msg.linear.z)
        self.get_logger().info("Angular x: '%s'" % msg.angular.x)
        self.get_logger().info("Angular y: '%s'" % msg.angular.y)
        self.get_logger().info("Angular z: '%s'" % msg.angular.z)
    
    #function for the publisher for /odom
    def timer_callback(self):
        msg = Float32()
        msg.data = 1.0
        self.publisher_.publish(msg)

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
