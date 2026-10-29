#!/usr/bin/env python3
"""Open-vocabulary object detector node (YOLO-World, inference only).

Subscribes to the overhead camera image, runs YOLO-World with the text prompts
listed in task.yaml and publishes

  /detections        std_msgs/String  JSON list of {label, conf, u, v, bbox}
  /detections/image  sensor_msgs/Image annotated image (for RViz / the video)

Run it with the Python virtualenv that has ultralytics installed, e.g.

  ~/pai_final_ws/ai/.venv/bin/python -m gp8_sort_ai.yolo_detector --ros-args -p weights_dir:=~/pai_final_ws/ai/models
"""
import json
import os
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


def image_to_numpy(msg: Image) -> np.ndarray:
    """sensor_msgs/Image (rgb8 / bgr8 / mono8) -> HxWx3 BGR uint8 array."""
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding in ("rgb8", "bgr8"):
        img = buf.reshape(msg.height, msg.step // 3, 3)[:, : msg.width, :]
        if msg.encoding == "rgb8":
            img = img[:, :, ::-1]
    elif msg.encoding == "mono8":
        img = buf.reshape(msg.height, msg.step)[:, : msg.width]
        img = np.stack([img] * 3, axis=-1)
    else:
        raise ValueError(f"unsupported encoding {msg.encoding}")
    return np.ascontiguousarray(img)


def numpy_to_image(img_bgr: np.ndarray, header) -> Image:
    msg = Image()
    msg.header = header
    msg.height, msg.width = img_bgr.shape[:2]
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = np.ascontiguousarray(img_bgr).tobytes()
    return msg


class YoloDetector(Node):
    def __init__(self):
        super().__init__("yolo_detector")
        default_cfg = os.path.join(get_package_share_directory("gp8_sort_ai"), "config", "task.yaml")
        self.declare_parameter("task_config", default_cfg)
        self.declare_parameter("weights_dir", os.path.expanduser("~/pai_final_ws/ai/models"))
        self.declare_parameter("device", "")  # "" = auto (cuda if available)

        with open(self.get_parameter("task_config").value) as f:
            cfg = yaml.safe_load(f)
        self.labels = [o["label"] for o in cfg["objects"]]
        det = cfg["detector"]
        self.conf = float(det.get("conf", 0.2))
        self.imgsz = int(det.get("imgsz", 640))
        self.period = 1.0 / float(det.get("rate_hz", 4.0))

        weights_dir = os.path.expanduser(self.get_parameter("weights_dir").value)
        weights = det["weights"]
        if not os.path.isabs(weights):
            weights = os.path.join(weights_dir, weights)
        os.makedirs(weights_dir, exist_ok=True)
        os.chdir(weights_dir)  # ultralytics caches auxiliary weights (e.g. CLIP) in the cwd

        from ultralytics import YOLO  # imported late: heavy

        self.get_logger().info(f"loading {weights} ...")
        self.model = YOLO(weights)
        names = list(self.model.names.values()) if isinstance(self.model.names, dict) else list(self.model.names)
        if names != self.labels:
            if hasattr(self.model, "set_classes"):
                # open-vocabulary model (YOLO-World): set the text prompts
                self.model.set_classes(self.labels)
            else:
                self.get_logger().warn(f"model classes {names} differ from task labels {self.labels}")
        device = self.get_parameter("device").value
        if not device:
            import torch

            device = "cuda:0" if torch.cuda.is_available() else "cpu"
        self.device = device
        self.get_logger().info(f"YOLO-World ready on {device}, classes={self.labels}")

        self.pub = self.create_publisher(String, "/detections", 10)
        self.pub_img = self.create_publisher(Image, "/detections/image", 2)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(Image, cfg["camera"]["image_topic"], self.on_image, qos)
        self.last = 0.0
        # task status from pick_place, overlaid on the annotated image (for the demo video)
        self.status = ""
        self.create_subscription(String, "/sort_status", lambda m: setattr(self, "status", m.data), 10)

    def on_image(self, msg: Image):
        now = time.monotonic()
        if now - self.last < self.period:
            return
        self.last = now
        img = image_to_numpy(msg)
        res = self.model.predict(img, conf=self.conf, imgsz=self.imgsz, device=self.device, verbose=False)[0]

        dets = []
        for box, cls, conf in zip(res.boxes.xyxy.tolist(), res.boxes.cls.tolist(), res.boxes.conf.tolist()):
            x1, y1, x2, y2 = box
            dets.append({
                "label": self.model.names[int(cls)],
                "conf": round(float(conf), 3),
                "u": round((x1 + x2) / 2.0, 1),
                "v": round((y1 + y2) / 2.0, 1),
                "bbox": [round(x1, 1), round(y1, 1), round(x2, 1), round(y2, 1)],
            })
        out = String()
        out.data = json.dumps({"stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, "detections": dets})
        self.pub.publish(out)

        annotated = res.plot(line_width=2, font_size=10)  # BGR
        self.draw_overlay(annotated, len(dets))
        self.pub_img.publish(numpy_to_image(annotated, msg.header))

    def draw_overlay(self, img, n_det):
        import cv2

        h, w = img.shape[:2]
        cv2.rectangle(img, (0, 0), (w, 28), (30, 30, 30), -1)
        cv2.putText(img, f"overhead camera | YOLO11n (fine-tuned on auto-labelled sim data) | {n_det} objects",
                    (8, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        if self.status:
            cv2.rectangle(img, (0, h - 32), (w, h), (30, 30, 30), -1)
            cv2.putText(img, self.status[:110], (8, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (80, 255, 80), 1, cv2.LINE_AA)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
