#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class PerceptionTestNode(Node):
    def __init__(self):
        super().__init__('perception_test_node')

        # Change this if your camera topic is different
        self.image_topic = '/camera/image_raw'

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        self.get_logger().info(f'Listening to camera topic: {self.image_topic}')

    def image_callback(self, msg: Image):
        try:
            # ROS image -> OpenCV BGR image
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        height, width, _ = frame_bgr.shape

        # Compute average RGB over the whole image
        # OpenCV uses BGR, so convert to RGB for reporting
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        avg_rgb = np.mean(frame_rgb, axis=(0, 1))  # [R, G, B]

        # Pick a scanline near the bottom of the image
        row = int(height * 0.8)
        scanline_bgr = frame_bgr[row, :, :]
        scanline_gray = cv2.cvtColor(scanline_bgr.reshape(1, width, 3), cv2.COLOR_BGR2GRAY)[0]

        # Threshold for dark line detection
        # Pixels below threshold are treated as line candidates
        threshold = 80
        dark_pixels = np.where(scanline_gray < threshold)[0]

        if len(dark_pixels) > 0:
            line_idx = int(np.mean(dark_pixels))
        else:
            line_idx = -1  # line not found

        self.get_logger().info(
            f'line_idx={line_idx}, avg_rgb=({avg_rgb[0]:.1f}, {avg_rgb[1]:.1f}, {avg_rgb[2]:.1f})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()