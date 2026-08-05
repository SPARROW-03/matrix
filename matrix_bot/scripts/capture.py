#!/usr/bin/env python3
"""
capture_frame.py

One-shot helper: subscribes to the camera topic, saves the first frame
received to disk, then exits. Run this once per digit while manually
driving/teleoperating the robot so the digit sign fills a good portion
of the frame, clearly and mostly head-on.

Usage:
    python3 capture_frame.py <output_filename.png>

Example:
    python3 capture_frame.py raw_digit_1.png
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

IMAGE_TOPIC = '/camera_sensor/image_raw'  # match your camera topic


class FrameGrabber(Node):
    def __init__(self, out_path):
        super().__init__('frame_grabber')
        self.out_path = out_path
        self.bridge = CvBridge()
        self.saved = False
        self.create_subscription(Image, IMAGE_TOPIC, self.cb, qos_profile_sensor_data)

    def cb(self, msg):
        if self.saved:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imwrite(self.out_path, frame)
        self.get_logger().info(f'Saved frame to {self.out_path}')
        self.saved = True


def main():
    if len(sys.argv) < 2:
        print('Usage: python3 capture_frame.py <output_filename.png>')
        sys.exit(1)

    rclpy.init()
    node = FrameGrabber(sys.argv[1])
    while rclpy.ok() and not node.saved:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()