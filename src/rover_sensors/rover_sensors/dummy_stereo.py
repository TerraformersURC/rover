#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from comms_interfaces.msg import StereoCameraStream
from std_msgs.msg import Header

import cv2
import numpy as np

from shared_python.camera_encoding import encode_img
from datetime import datetime

IMG_PATH = "testimage.png"

DIMENSIONS = (640, 480)
FPS = 30

class FakePublisher(Node):
    
    def __init__(self):
        super().__init__("stereo_camera_publisher")
        
        self.img = cv2.imread(IMG_PATH)
        
        self.publisher = self.create_publisher(StereoCameraStream, 'stereo_camera', 10)
        self.frame_id = 0
        
        try:
            self.create_timer(1 / FPS, self.send_frame) # sends every 0.5 seconds, can be changed
        except Exception as e:
            exit(1)
        
    
    #function for the publisher to /motor_control
    def send_frame(self):
        
        timestamp_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        # Put timestamp text on top of self.img
        cv2.putText(self.img, timestamp_str, (10, DIMENSIONS[1] // 2), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        
        # Create a blank depth image with the timestamp text for the depth frame
        depth_image = np.zeros((DIMENSIONS[1], DIMENSIONS[0]), dtype=np.uint8)
        cv2.putText(depth_image, timestamp_str, (10, DIMENSIONS[1] // 2), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, 2, cv2.LINE_AA)
        depth_frame = type("DepthFrame", (), {"get_data": lambda: depth_image})()
        
        encoded_color = encode_img(self.img)
        
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
    node = FakePublisher()

    #keep the node alive, enable all callbacks
    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
