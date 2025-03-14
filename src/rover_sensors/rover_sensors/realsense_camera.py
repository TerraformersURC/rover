#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from comms_interfaces.msg import StereoCameraStream
from std_msgs import Header

import pyrealsense2 as rs
import cv2
import numpy as np

from comms_interfaces.src.camera_encoding import encode_img

DIMENSIONS = (640, 480)
FPS = 30

class RealsensePublisher(Node):
    
    def __init__(self):
        super().__init__("stereo_camera_publisher")
        
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, *DIMENSIONS, rs.format.bgr8, FPS)
        self.config.enable_stream(rs.stream.depth, *DIMENSIONS, rs.format.z16, FPS)

        self.publisher = self.create_publisher(StereoCameraStream, 'stereo_camera', 10)
        self.frame_id = 0
        
        try:
            self.pipeline.start(self.config)
        except Exception as e:
            self.get_logger().error(f"Failed to start pipeline: {e}")
            exit(1)
        
        self.create_timer(1 / FPS, self.send_frame) # sends every 0.5 seconds, can be changed
    
    #function for the publisher to /motor_control
    def send_frame(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            return
        
        encoded_color = encode_img(color_frame.get_data())
        
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(
                np.asanyarray(depth_frame.get_data()),
                alpha=0.03
            ),
            cv2.COLORMAP_JET
        )
        
        encoded_depth = encode_img(depth_colormap)
        
        if encoded_color is None or encoded_depth is None:
            return
        
        msg = StereoCameraStream()
        
        self.frame_id = self.frame_id + 1
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.frame_id)
        
        msg.rgb.frombytes(encoded_color)
        msg.depth.frombytes(encoded_depth)
        
        self.publisher.publish(msg)
        return

def main(args=None):
    #initialize ros2 communications
    rclpy.init(args=args)

    #initialize node
    node = RealsensePublisher()

    #keep the node alive, enable all callbacks
    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
