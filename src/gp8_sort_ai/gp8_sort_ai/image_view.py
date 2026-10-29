#!/usr/bin/env python3
"""Minimal, always-updating image window (OpenCV) for a sensor_msgs/Image topic.

Used for the demo video instead of rqt_image_view, which stopped refreshing.

  ~/pai_final_ws/ai/.venv/bin/python -m gp8_sort_ai.image_view /detections/image "Overhead camera - YOLO"
"""
import sys

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from gp8_sort_ai.yolo_detector import image_to_numpy


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else "/detections/image"
    title = sys.argv[2] if len(sys.argv) > 2 else topic
    rclpy.init()
    node = Node("image_view")
    frame = {}
    node.create_subscription(
        Image, topic, lambda m: frame.__setitem__("img", image_to_numpy(m)),
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
    )
    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if "img" in frame:
                cv2.imshow(title, frame["img"])
            if cv2.waitKey(10) == 27:  # Esc
                break
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
