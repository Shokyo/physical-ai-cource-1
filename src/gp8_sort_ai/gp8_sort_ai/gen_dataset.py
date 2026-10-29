#!/usr/bin/env python3
"""Generate an automatically labelled YOLO dataset from the running Gazebo scene.

The objects are moved to random poses on the table with the Gazebo `set_pose`
service, an image of the overhead camera is captured and the bounding boxes
are computed by projecting the known 3D geometry of each object through the
camera model (ground truth from the simulator, no manual annotation).

Run with the simulation started (sim.launch.py), using the AI virtualenv:

  ~/pai_final_ws/ai/.venv/bin/python -m gp8_sort_ai.gen_dataset --out ~/pai_final_ws/ai/dataset --train 240 --val 60
"""
import argparse
import math
import os
import random
import subprocess
import time

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
PARK = [3.0, 3.0, 0.2]  # where absent objects are put (out of the camera view)


def quat_z(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def json_quote(text):
    """quote a string for a protobuf text-format request"""
    return '"' + text.replace("\\", "\\\\").replace('"', '\\"') + '"'


def object_sdf(obj, x, y, z, yaw):
    """SDF of one pickable object built from its task.yaml entry."""
    r, g, b = obj.get("color", [0.5, 0.5, 0.5])
    mass = float(obj.get("mass", 0.1))
    if obj.get("shape", "box") == "cylinder":
        rad, h = float(obj["radius"]), float(obj["height"])
        geom = f"<cylinder><radius>{rad}</radius><length>{h}</length></cylinder>"
        ixx = mass * (3 * rad * rad + h * h) / 12
        izz = mass * rad * rad / 2
        iyy = ixx
    else:
        sx, sy, sz = [float(v) for v in obj["size"]]
        geom = f"<box><size>{sx} {sy} {sz}</size></box>"
        ixx = mass * (sy * sy + sz * sz) / 12
        iyy = mass * (sx * sx + sz * sz) / 12
        izz = mass * (sx * sx + sy * sy) / 12
    return (
        f'<sdf version="1.9"><model name="{obj["model"]}"><pose>{x} {y} {z} 0 0 {yaw}</pose>'
        f'<link name="link"><inertial><mass>{mass}</mass><inertia><ixx>{ixx:.3e}</ixx><iyy>{iyy:.3e}</iyy>'
        f'<izz>{izz:.3e}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>'
        f'<collision name="collision"><geometry>{geom}</geometry>'
        f'<surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>'
        f'<visual name="visual"><geometry>{geom}</geometry>'
        f'<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>'
        f'<specular>0.2 0.2 0.2 1</specular></material></visual></link></model></sdf>'
    )


class Gen(Node):
    def __init__(self, cfg, world):
        super().__init__("gen_dataset")
        self.cfg = cfg
        self.world = world
        self.info = None
        self.img = None
        self.stamp = 0.0
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, cfg["camera"]["image_topic"], self.on_img, qos)
        self.create_subscription(CameraInfo, cfg["camera"]["info_topic"], self.on_info, 10)
        self.traj_pub = self.create_publisher(JointTrajectory, "/gp8_arm_controller/joint_trajectory", 1)

    def on_info(self, m):
        self.info = m

    def on_img(self, m):
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step // 3, 3)[:, : m.width]
        self.img = np.ascontiguousarray(a[:, :, ::-1] if m.encoding == "rgb8" else a)
        self.stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def spin_for(self, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.05)

    def move_arm(self, joints, duration=3.0):
        t0 = time.time()
        while self.traj_pub.get_subscription_count() == 0 and time.time() - t0 < 10.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        msg = JointTrajectory()
        msg.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(j) for j in joints]
        pt.time_from_start.sec = int(duration)
        pt.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        msg.points = [pt]
        self.traj_pub.publish(msg)

    def gz_service(self, service, reqtype, req, reptype="gz.msgs.Boolean", timeout=3000):
        cmd = ["gz", "service", "-s", f"/world/{self.world}/{service}", "--reqtype", reqtype,
               "--reptype", reptype, "--timeout", str(timeout), "--req", req]
        r = subprocess.run(cmd, capture_output=True, text=True)
        ok = r.returncode == 0 and "true" in r.stdout
        if not ok:
            self.get_logger().warn(f"{service} failed: {req[:60]}... {r.stdout.strip()} {r.stderr.strip()}")
        return ok

    def set_pose(self, name, x, y, z, yaw):
        qx, qy, qz, qw = quat_z(yaw)
        req = (f'name: "{name}", position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}, '
               f'orientation: {{x: {qx:.6f}, y: {qy:.6f}, z: {qz:.6f}, w: {qw:.6f}}}')
        return self.gz_service("set_pose", "gz.msgs.Pose", req)

    def respawn(self, obj, x, y, z, yaw):
        """Remove the object model and spawn it again at the given pose.

        Models loaded from the world file did not respond to set_pose in our
        setup (Gazebo 9.6), models spawned at runtime do, so the generator
        re-spawns every object once at start-up and uses set_pose afterwards.
        """
        name = obj["model"]
        # remove and WAIT until the model is really gone: a late removal would
        # leave two models with the same name for a moment and the
        # DetachableJoint plugin then cannot resolve the child model.
        if self.model_exists(name):
            self.gz_service("remove", "gz.msgs.Entity", f'name: "{name}", type: MODEL')
            for _ in range(40):
                time.sleep(0.1)
                if not self.model_exists(name):
                    break
            else:
                self.get_logger().warn(f"{name} still exists after remove request")
        ok = self.gz_service("create", "gz.msgs.EntityFactory", "sdf: " + json_quote(object_sdf(obj, x, y, z, yaw)))
        for _ in range(40):
            if self.model_exists(name):
                return ok
            time.sleep(0.1)
        self.get_logger().warn(f"{name} did not appear after create request")
        return False

    def model_exists(self, name):
        r = subprocess.run(["gz", "model", "--list"], capture_output=True, text=True, timeout=10)
        return any(line.strip() == f"- {name}" for line in r.stdout.splitlines())

    def wait_fresh_image(self, after_stamp, min_dt=0.3, timeout=5.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.img is not None and self.stamp >= after_stamp + min_dt:
                return self.img.copy()
        return None

    # ------------------------------------------------------------ projection
    def project(self, pts):
        """world points (N,3) -> pixel (N,2). Camera looks straight down,
        image up = world +x, image right = world -y (see task.yaml)."""
        k = self.info.k
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        cam = np.array(self.cfg["camera"]["position"])
        depth = cam[2] - pts[:, 2]
        u = cx - fx * (pts[:, 1] - cam[1]) / depth
        v = cy - fy * (pts[:, 0] - cam[0]) / depth
        return np.stack([u, v], axis=1)

    def bbox(self, obj, x, y, z, yaw):
        """pixel bbox of the object by projecting its 3D extents."""
        if obj.get("shape", "box") == "cylinder":
            r, h = float(obj["radius"]), float(obj["height"])
            ang = np.linspace(0, 2 * np.pi, 24, endpoint=False)
            ring = np.stack([r * np.cos(ang), r * np.sin(ang)], axis=1)
            pts = np.concatenate([
                np.column_stack([ring, np.full(len(ring), -h / 2)]),
                np.column_stack([ring, np.full(len(ring), h / 2)]),
            ])
        else:
            sx, sy, sz = [float(s) for s in obj["size"]]
            pts = np.array([[sx * a, sy * b, sz * c] for a in (-0.5, 0.5) for b in (-0.5, 0.5) for c in (-0.5, 0.5)])
        c, s = math.cos(yaw), math.sin(yaw)
        rot = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        world = pts @ rot.T + np.array([x, y, z])
        uv = self.project(world)
        return uv[:, 0].min(), uv[:, 1].min(), uv[:, 0].max(), uv[:, 1].max()


def sample_positions(n, rng, xr, yr, min_dist):
    for _ in range(200):
        pts = [(rng.uniform(*xr), rng.uniform(*yr)) for _ in range(n)]
        ok = all(math.hypot(a[0] - b[0], a[1] - b[1]) >= min_dist for i, a in enumerate(pts) for b in pts[i + 1:])
        if ok:
            return pts
    return pts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.expanduser("~/pai_final_ws/ai/dataset"))
    ap.add_argument("--train", type=int, default=240)
    ap.add_argument("--val", type=int, default=60)
    ap.add_argument("--world", default="sort_table")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--task_config", default=os.path.join(get_package_share_directory("gp8_sort_ai"), "config", "task.yaml"))
    args = ap.parse_args()

    with open(args.task_config) as f:
        cfg = yaml.safe_load(f)
    objs = cfg["objects"]
    table_z = float(cfg["table_top_z"])
    zone = cfg["dataset"]
    rng = random.Random(args.seed)

    rclpy.init()
    node = Gen(cfg, args.world)
    node.get_logger().info("waiting for camera ...")
    while node.info is None or node.img is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.get_logger().info("re-spawning the objects")
    for o in objs:
        p = o["initial_pose"]
        node.respawn(o, p[0], p[1], p[2], p[3])
    node.get_logger().info("parking the arm out of the camera view")
    node.move_arm(cfg["motion"]["ready"], 3.0)
    node.spin_for(4.0)

    for split, n in (("train", args.train), ("val", args.val)):
        os.makedirs(os.path.join(args.out, "images", split), exist_ok=True)
        os.makedirs(os.path.join(args.out, "labels", split), exist_ok=True)
        for i in range(n):
            # which objects are present in this sample (at least one)
            present = [o for o in objs if rng.random() < zone.get("presence_prob", 0.85)]
            if not present:
                present = [rng.choice(objs)]
            pos = sample_positions(len(present), rng, zone["x_range"], zone["y_range"], zone.get("min_dist", 0.09))
            poses = {}
            ok = True
            for o, (x, y) in zip(present, pos):
                yaw = rng.uniform(-math.pi, math.pi)
                z = table_z + float(o["height"]) / 2 + 0.001
                poses[o["model"]] = (o, x, y, z, yaw)
                # retry once: a timed-out service call would leave a wrong label
                ok &= node.set_pose(o["model"], x, y, z, yaw) or node.set_pose(o["model"], x, y, z, yaw)
            for k, o in enumerate(objs):
                if o["model"] not in poses:
                    ok &= (node.set_pose(o["model"], PARK[0] + 0.3 * k, PARK[1], PARK[2], 0.0)
                           or node.set_pose(o["model"], PARK[0] + 0.3 * k, PARK[1], PARK[2], 0.0))
            if not ok:
                node.get_logger().warn("set_pose failed twice, skipping this sample")
                continue

            t_set = node.stamp
            node.spin_for(0.3)
            img = node.wait_fresh_image(t_set, min_dt=0.4)
            if img is None:
                node.get_logger().warn("no fresh image, skipping")
                continue
            h, w = img.shape[:2]
            lines = []
            for cls_id, o in enumerate(objs):
                if o["model"] not in poses:
                    continue
                _, x, y, z, yaw = poses[o["model"]]
                x1, y1, x2, y2 = node.bbox(o, x, y, z, yaw)
                x1, x2 = max(0, x1), min(w - 1, x2)
                y1, y2 = max(0, y1), min(h - 1, y2)
                if x2 - x1 < 4 or y2 - y1 < 4:
                    continue
                lines.append(f"{cls_id} {(x1 + x2) / 2 / w:.6f} {(y1 + y2) / 2 / h:.6f} {(x2 - x1) / w:.6f} {(y2 - y1) / h:.6f}")
            name = f"{split}_{i:04d}"
            cv2.imwrite(os.path.join(args.out, "images", split, name + ".png"), img)
            with open(os.path.join(args.out, "labels", split, name + ".txt"), "w") as f:
                f.write("\n".join(lines) + ("\n" if lines else ""))
            if i % 20 == 0:
                node.get_logger().info(f"{split}: {i}/{n}")

    # restore the initial scene (same poses as the world file)
    for o in objs:
        p = o.get("initial_pose")
        if p:
            node.set_pose(o["model"], p[0], p[1], p[2], p[3])

    with open(os.path.join(args.out, "data.yaml"), "w") as f:
        yaml.safe_dump({
            "path": os.path.abspath(args.out),
            "train": "images/train",
            "val": "images/val",
            "names": {i: o["label"] for i, o in enumerate(objs)},
        }, f, sort_keys=False)
    node.get_logger().info(f"dataset written to {args.out}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
