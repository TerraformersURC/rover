#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from comms_interfaces.msg import StereoCameraStream
from std_msgs.msg import Header

from shared_python.camera_encoding import decode_img

import cv2
import numpy as np

DIMENSIONS = (640, 480)
FPS = 30


class CameraVisual(Node):
    
    def __init__(self):
        super().__init__("controller_node")

        # create the publisher
        self.subscription = self.create_subscription(StereoCameraStream, 'stereo_camera', self.display_frame, 10)
    
    #function for the publisher to /motor_control
    def display_frame(self, msg: StereoCameraStream):
        color_frame = decode_img(msg.rgb.tobytes())
        depth_frame = decode_img(msg.depth.tobytes())
        
        cv2.imshow('Color', color_frame)
        cv2.imshow('Depth', depth_frame)
        cv2.waitKey(1)

def main(args=None):
    #initialize ros2 communications
    rclpy.init(args=args)

    #initialize node
    node = CameraVisual()

    #keep the node alive, enable all callbacks
    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
