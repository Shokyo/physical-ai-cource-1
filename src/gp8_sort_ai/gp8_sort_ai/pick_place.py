#!/usr/bin/env python3
"""Pick-and-place sorting node.

Waits for YOLO-World detections, converts pixel positions of the detected
objects to world coordinates (fixed overhead camera + known table height),
then uses MoveIt 2 (via pymoveit2) to pick each object with the simulated
vacuum gripper and place it in the tray assigned to its class.

  ros2 run gp8_sort_ai pick_place
"""
import json
import math
import os
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from pymoveit2 import MoveIt2
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Empty, String

ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
# tool z axis pointing straight down (rotation of pi about x)
QUAT_DOWN = (1.0, 0.0, 0.0, 0.0)


class PickPlace(Node):
    def __init__(self):
        super().__init__("pick_place")
        default_cfg = os.path.join(get_package_share_directory("gp8_sort_ai"), "config", "task.yaml")
        self.declare_parameter("task_config", default_cfg)
        self.declare_parameter("min_detections", 3)      # consecutive frames an object must be seen
        self.declare_parameter("dry_run", False)         # plan/move but never attach
        with open(self.get_parameter("task_config").value) as f:
            self.cfg = yaml.safe_load(f)
        self.objects = {o["label"]: o for o in self.cfg["objects"]}
        self.motion = self.cfg["motion"]
        self.cam_pos = self.cfg["camera"]["position"]
        self.table_z = float(self.cfg["table_top_z"])
        # pick zone = table top area (a bit larger than the random placement zone)
        zone = self.cfg.get("dataset", {})
        margin = 0.06
        self.zone_x = (float(zone.get("x_range", [0.3, 0.7])[0]) - margin, float(zone.get("x_range", [0.3, 0.7])[1]) + margin)
        self.zone_y = (float(zone.get("y_range", [-0.3, 0.3])[0]) - margin, float(zone.get("y_range", [-0.3, 0.3])[1]) + margin)

        self.cb_group = ReentrantCallbackGroup()
        self.camera_info = None
        self.latest = {}          # label -> (u, v, conf)
        self.seen_count = {}      # label -> consecutive frames seen
        self.lock = threading.Lock()

        self.create_subscription(CameraInfo, self.cfg["camera"]["info_topic"], self.on_info, 10,
                                 callback_group=self.cb_group)
        self.create_subscription(String, "/detections", self.on_detections, 10, callback_group=self.cb_group)
        self.status_pub = self.create_publisher(String, "/sort_status", 10)

        self.attach_pubs = {
            o["model"]: self.create_publisher(Empty, f"/gp8/attach/{o['model']}", 1) for o in self.cfg["objects"]
        }
        self.detach_pubs = {
            o["model"]: self.create_publisher(Empty, f"/gp8/detach/{o['model']}", 1) for o in self.cfg["objects"]
        }
        # attach state reported by the DetachableJoint plugins ("attached" / "detached")
        self.vacuum_state = {o["model"]: "detached" for o in self.cfg["objects"]}
        for o in self.cfg["objects"]:
            model = o["model"]
            self.create_subscription(
                String, f"/gp8/attached/{model}",
                lambda msg, m=model: self.vacuum_state.__setitem__(m, msg.data), 10,
                callback_group=self.cb_group)

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=ARM_JOINTS,
            base_link_name="world",
            end_effector_name="tcp",
            group_name="arm",
            callback_group=self.cb_group,
        )
        self.moveit2.max_velocity = float(self.motion.get("velocity_scaling", 0.3))
        self.moveit2.max_acceleration = float(self.motion.get("acceleration_scaling", 0.3))
        self.moveit2.planner_id = "RRTConnectkConfigDefault"

    # ------------------------------------------------------------------ callbacks
    def on_info(self, msg: CameraInfo):
        self.camera_info = msg

    def on_detections(self, msg: String):
        if self.camera_info is None:
            return
        data = json.loads(msg.data)
        best = {}
        for d in data["detections"]:
            if d["label"] not in self.objects:
                continue
            # ignore detections outside the pick zone (e.g. objects already placed in a tray)
            x, y, _ = self.pixel_to_world(d["u"], d["v"], float(self.objects[d["label"]]["height"]))
            if not (self.zone_x[0] <= x <= self.zone_x[1] and self.zone_y[0] <= y <= self.zone_y[1]):
                continue
            if d["label"] not in best or d["conf"] > best[d["label"]]["conf"]:
                best[d["label"]] = d
        with self.lock:
            for label in self.objects:
                if label in best:
                    d = best[label]
                    self.latest[label] = (d["u"], d["v"], d["conf"])
                    self.seen_count[label] = self.seen_count.get(label, 0) + 1
                else:
                    self.seen_count[label] = 0

    # ------------------------------------------------------------------ helpers
    def say(self, text: str):
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    def pixel_to_world(self, u: float, v: float, object_height: float):
        """Back-project a pixel onto the plane z = table_top + object_height.

        The camera looks straight down (pose rpy = 0, pi/2, 0): image up is
        world +x, image right is world -y.
        """
        k = self.camera_info.k
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        plane_z = self.table_z + object_height
        depth = self.cam_pos[2] - plane_z
        x = self.cam_pos[0] - (v - cy) * depth / fy
        y = self.cam_pos[1] - (u - cx) * depth / fx
        return x, y, plane_z

    def vacuum(self, model: str, on: bool, timeout: float = 3.0) -> bool:
        """Attach/detach the object and wait for the plugin to confirm; re-send until it does."""
        want = "attached" if on else "detached"
        pub = (self.attach_pubs if on else self.detach_pubs)[model]
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            pub.publish(Empty())
            for _ in range(6):
                time.sleep(0.05)
                if self.vacuum_state.get(model) == want:
                    return True
        self.get_logger().warn(f"vacuum {'ON' if on else 'OFF'} for {model} not confirmed")
        return False

    def warm_up_vacuum(self):
        """Attach and immediately detach every object once while the arm is still.

        After a fresh Gazebo start the first state message ("attached") of each
        DetachableJoint plugin never reached ROS in our setup although the joint
        was created; every later message does. A no-motion attach/detach cycle
        before the task makes the real confirmations reliable. The objects do not
        move because the arm does not move in between.
        """
        self.say("warming up vacuum plugins")
        for o in self.cfg["objects"]:
            self.attach_pubs[o["model"]].publish(Empty())
            time.sleep(0.3)
            self.detach_pubs[o["model"]].publish(Empty())
            time.sleep(0.3)
        for o in self.cfg["objects"]:
            self.vacuum_state[o["model"]] = "detached"

    def goto_joints(self, joints, label=""):
        self.say(f"moving to {label or 'joint target'}")
        self.moveit2.move_to_configuration(joints)
        return self.moveit2.wait_until_executed()

    def goto_pose(self, x, y, z, cartesian=False, label=""):
        self.say(f"{'linear' if cartesian else 'planning'} move to ({x:.3f}, {y:.3f}, {z:.3f}) {label}")
        self.moveit2.move_to_pose(
            position=(x, y, z),
            quat_xyzw=QUAT_DOWN,
            cartesian=cartesian,
            cartesian_max_step=0.005,
            cartesian_fraction_threshold=0.9 if cartesian else 0.0,
            tolerance_position=0.002,
            tolerance_orientation=0.02,
        )
        return self.moveit2.wait_until_executed()

    def goto_pose_robust(self, x, y, z, label=""):
        """Try a straight-line move first, fall back to free-space planning."""
        if self.goto_pose(x, y, z, cartesian=True, label=label):
            return True
        self.get_logger().warn("cartesian move failed, retrying with OMPL")
        return self.goto_pose(x, y, z, cartesian=False, label=label)

    def add_scene(self):
        """Table and trays as collision objects so MoveIt plans around them."""
        self.moveit2.add_collision_box(
            id="table", size=(0.5, 0.7, 0.15), position=(0.5, 0.0, 0.075),
            quat_xyzw=(0.0, 0.0, 0.0, 1.0), frame_id="world",
        )
        for i, o in enumerate(self.cfg["objects"]):
            tx, ty = float(o["tray"][0]), float(o["tray"][1])
            tz = float(o["tray_top_z"])
            self.moveit2.add_collision_box(
                id=f"tray_{i}", size=(0.16, 0.16, tz), position=(tx, ty, tz / 2),
                quat_xyzw=(0.0, 0.0, 0.0, 1.0), frame_id="world",
            )

    # ------------------------------------------------------------------ task
    def pick_and_place(self, label: str, u: float, v: float) -> bool:
        o = self.objects[label]
        h = float(o["height"])
        x, y, top_z = self.pixel_to_world(u, v, h)
        approach = float(self.motion["approach_height"])
        lift = float(self.motion["lift_height"])
        self.say(f"[{label}] detected at pixel ({u:.0f}, {v:.0f}) -> world ({x:.3f}, {y:.3f}, {top_z:.3f})")

        if not self.goto_pose(x, y, top_z + approach, label=f"above {label}"):
            return False
        if not self.goto_pose_robust(x, y, top_z + 0.004, label="descend"):
            return False
        if not self.get_parameter("dry_run").value:
            self.say(f"[{label}] vacuum ON")
            if not self.vacuum(o["model"], True):
                return False
            time.sleep(0.3)
        if not self.goto_pose_robust(x, y, top_z + lift, label="lift"):
            return False

        tx, ty = float(o["tray"][0]), float(o["tray"][1])
        tz = float(o["tray_top_z"])
        place_z = tz + h + 0.01
        if not self.goto_pose(tx, ty, place_z + approach, label=f"above tray of {label}"):
            return False
        if not self.goto_pose_robust(tx, ty, place_z, label="lower"):
            return False
        self.say(f"[{label}] vacuum OFF")
        if not self.get_parameter("dry_run").value:
            self.vacuum(o["model"], False)
        time.sleep(0.3)
        return self.goto_pose_robust(tx, ty, place_z + lift, label="retreat")

    def run(self):
        self.say("waiting for move_group, camera info and detections ...")
        while rclpy.ok() and self.camera_info is None:
            time.sleep(0.2)
        time.sleep(1.0)
        self.add_scene()
        self.warm_up_vacuum()
        self.goto_joints(self.motion["ready"], "ready pose")

        done = set()
        min_seen = int(self.get_parameter("min_detections").value)
        idle_since = time.monotonic()
        while rclpy.ok() and len(done) < len(self.objects):
            with self.lock:
                candidates = [
                    (self.latest[l][2], l, self.latest[l][0], self.latest[l][1])
                    for l in self.objects
                    if l not in done and self.seen_count.get(l, 0) >= min_seen
                ]
            if not candidates:
                if time.monotonic() - idle_since > 30.0:
                    self.say(f"no more detections, sorted {len(done)}/{len(self.objects)}")
                    break
                time.sleep(0.2)
                continue
            candidates.sort(reverse=True)
            conf, label, u, v = candidates[0]
            ok = self.pick_and_place(label, u, v)
            if ok:
                done.add(label)
                self.say(f"[{label}] placed ({len(done)}/{len(self.objects)})")
            else:
                self.say(f"[{label}] FAILED, will retry after re-detection")
                self.vacuum(self.objects[label]["model"], False, timeout=1.0)
            self.goto_joints(self.motion["ready"], "ready pose")
            with self.lock:  # only count detections made from the observation pose
                self.seen_count = {}
            time.sleep(1.0)  # let the detector see the new scene
            idle_since = time.monotonic()

        self.goto_joints(self.motion["home"], "home pose")
        self.say("task finished")


def main(args=None):
    rclpy.init(args=args)
    node = PickPlace()
    executor = MultiThreadedExecutor(4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
