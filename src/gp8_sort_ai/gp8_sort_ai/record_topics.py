#!/usr/bin/env python3
"""Record image topics to constant-frame-rate MP4 files (off-screen video capture).

Each topic is written at `--fps` frames per second of wall-clock time; the most
recent frame is repeated when no new image arrived, so all outputs stay in sync
and can be stacked side by side with ffmpeg afterwards.

  ~/pai_final_ws/ai/.venv/bin/python -m gp8_sort_ai.record_topics \\
      --out ~/pai_final_ws/media/rec --fps 20 /scene_camera/image /detections/image
Stop with Ctrl+C (or SIGTERM).
"""
import argparse
import os
import signal
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image

from gp8_sort_ai.yolo_detector import image_to_numpy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("topics", nargs="+")
    ap.add_argument("--out", default=os.path.expanduser("~/pai_final_ws/media/rec"))
    ap.add_argument("--fps", type=float, default=20.0)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    rclpy.init()
    node = Node("record_topics")
    latest = {}
    writers = {}
    for t in args.topics:
        node.create_subscription(
            Image, t, lambda m, t=t: latest.__setitem__(t, image_to_numpy(m)),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
    stop = {"flag": False}
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__("flag", True))

    period = 1.0 / args.fps
    next_tick = time.monotonic()
    frames = 0
    node.get_logger().info(f"recording {args.topics} -> {args.out} at {args.fps} fps")
    try:
        while rclpy.ok() and not stop["flag"]:
            rclpy.spin_once(node, timeout_sec=0.005)
            now = time.monotonic()
            if now < next_tick:
                continue
            next_tick += period
            for t, img in latest.items():
                if t not in writers:
                    h, w = img.shape[:2]
                    fn = os.path.join(args.out, t.strip("/").replace("/", "_") + ".mp4")
                    writers[t] = cv2.VideoWriter(fn, cv2.VideoWriter_fourcc(*"mp4v"), args.fps, (w, h))
                    node.get_logger().info(f"{t}: {w}x{h} -> {fn}")
                writers[t].write(img)
            frames += 1
    except KeyboardInterrupt:
        pass
    for w in writers.values():
        w.release()
    node.get_logger().info(f"done, {frames} ticks ({frames / args.fps:.1f} s)")
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
