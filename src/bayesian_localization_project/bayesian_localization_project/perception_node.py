#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import UInt32
from std_msgs.msg import Float64MultiArray
import cv2
from cv_bridge import CvBridge
import numpy as np


# ── Tunable constants ─────────────────────────────────────────────────────────

# Scanline for line/track detection (fraction of image height from top)
SCANLINE_FRACTION = 0.85

# Grayscale threshold: pixels below this are the black track
LINE_DARK_THRESHOLD = 80

# Region of the image to search for coloured patches.
# Camera faces forward; floor patches appear in the bottom portion.
COLOUR_ROW_START = 0.75   # top of search region
COLOUR_ROW_END   = 1.00   # bottom of search region

# HSV saturation threshold (0-255): pixels below this are treated as grey/white
# and ignored for colour classification.  Increase if grey floor bleeds through.
SAT_THRESHOLD = 60

# Minimum number of saturated pixels required to report a colour.
# Lowered from 200: patches on the left/vertical track section produce fewer
# saturated pixels because the camera angle changes through corners.
MIN_COLOURED_PIXELS = 100


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        self.line_pub = self.create_publisher(UInt32, '/line_idx', 10)
        self.colour_pub = self.create_publisher(Float64MultiArray, '/mean_img_rgb', 10)

        self.get_logger().info('Perception node started')

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        h, w = frame.shape[:2]

        # ── Colour: HSV saturation filter over bottom half of image ──────────
        # The TurtleBot3 camera faces forward, so the floor patches appear
        # as coloured regions in the lower portion of the frame.
        # We ignore grey/white/black pixels (low saturation) and only average
        # the genuinely coloured pixels.  This avoids the grey floor diluting
        # the reading when only part of the patch is in view.
        r0 = int(h * COLOUR_ROW_START)
        region = frame[r0:, :]

        hsv = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)
        sat = hsv[:, :, 1]                          # saturation channel
        coloured = sat > SAT_THRESHOLD              # mask of non-grey pixels

        colour_msg = Float64MultiArray()
        if coloured.sum() >= MIN_COLOURED_PIXELS:
            # Mean BGR of only the saturated pixels → convert to RGB
            pixels = region[coloured]               # shape (N, 3)
            mean_bgr = pixels.mean(axis=0)
            colour_msg.data = [float(mean_bgr[2]), float(mean_bgr[1]), float(mean_bgr[0])]
            self.get_logger().info(
                f'COLOUR  rgb=({colour_msg.data[0]:.0f},'
                f'{colour_msg.data[1]:.0f},{colour_msg.data[2]:.0f})'
                f'  sat_px={coloured.sum()}')
        else:
            # No meaningful colour detected — publish neutral grey so the
            # Bayes node knows to treat this as 'nothing'
            colour_msg.data = [200.0, 200.0, 200.0]
        self.colour_pub.publish(colour_msg)

        # ── Line: centroid of dark pixels in scanline ─────────────────────────
        row = int(h * SCANLINE_FRACTION)
        gray_row = cv2.cvtColor(
            frame[row:row + 1, :], cv2.COLOR_BGR2GRAY).flatten()
        dark = np.where(gray_row < LINE_DARK_THRESHOLD)[0]

        line_msg = UInt32()
        if dark.size > 0:
            line_msg.data = int(dark.mean())
        else:
            line_msg.data = w // 2  # no line found: hold last heading
        self.line_pub.publish(line_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
