#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from comms_interfaces.msg import StereoCameraInfo
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

import cv2
import time

import numpy as np

DIMENSIONS = (10, 10)
FPS = 30
ENCODING_PARAMETERS = (
  int(cv2.IMWRITE_PNG_COMPRESSION),
  0
)

class DummyStereoCamera(Node):
    
    def __init__(self):
        super().__init__("stereo_camera_dummy_publisher")
        
        self.frame_no = 0
        self.publisher = self.create_publisher(StereoCameraInfo, 'stereo_camera', 10)
        
        self.create_timer(1 / FPS, self.send_frame) # sends every 0.5 seconds, can be changed
    
    #function for the publisher to /motor_control
    def send_frame(self):
        
        FAKE_FRAME = np.random.randint(0, 255, DIMENSIONS, dtype=np.uint8)
        
        res, img = cv2.imencode('.png', FAKE_FRAME, ENCODING_PARAMETERS)
        
        if not res:
            return
        
        self.frame_no += 1
        now = time.time()
        sec = int(now)
        nsec = int(1e9 * (now - sec))
        
        msg = StereoCameraInfo()
        
        header = Header()
        header.stamp = Time()
        header.stamp.sec = sec
        header.stamp.nanosec = nsec
        header.frame_id = f'frame {self.frame_no:06d}'
        
        msg.header = header
        msg.rgb.frombytes(img)
        msg.format = 'png'
        msg.depth = [0]
        
        self.publisher.publish(msg)

def main(args=None):
    #initialize ros2 communications
    rclpy.init(args=args)

    #initialize node
    node = DummyStereoCamera()

    #keep the node alive, enable all callbacks
    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
