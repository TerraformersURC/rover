#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from comms_interfaces.msg import StereoCameraInfo
from sensor_msgs.msg import Image, CameraInfo

import pyrealsense2 as rs
import cv2
import time

DIMENSIONS = (640, 480)
FPS = 30

class RealsensePublisher(Node):
    
    def __init__(self):
        super().__init__("stereo_camera_publisher")
        
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, *DIMENSIONS, rs.format.bgr8, FPS)
        self.config.enable_stream(rs.stream.depth, *DIMENSIONS, rs.format.z16, FPS)

        self.motor_control = self.create_publisher(StereoCameraInfo, 'stereo_camera', 10)
        
        self.pipeline.start(self.config)
        
        self.create_timer(1 / FPS, self.send_frame) # sends every 0.5 seconds, can be changed
    
    #function for the publisher to /motor_control
    def send_frame(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            return
        
        timestamp = time.localtime()
        
        

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
