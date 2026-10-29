#!/usr/bin/env python3
"""Reset or randomise the objects in the running Gazebo scene.

  ~/pai_final_ws/ai/.venv/bin/python -m gp8_sort_ai.scene_tool reset           # initial poses (as in the world file)
  ~/pai_final_ws/ai/.venv/bin/python -m gp8_sort_ai.scene_tool random [--seed N]  # random poses on the table

Objects are re-spawned (see gen_dataset.respawn for why).
"""
import argparse
import math
import os
import random
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory

from gp8_sort_ai.gen_dataset import Gen, sample_positions


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["reset", "random"])
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--world", default="sort_table")
    ap.add_argument("--task_config", default=os.path.join(get_package_share_directory("gp8_sort_ai"), "config", "task.yaml"))
    args = ap.parse_args()
    with open(args.task_config) as f:
        cfg = yaml.safe_load(f)
    objs = cfg["objects"]

    rclpy.init()
    node = Gen(cfg, args.world)
    if args.mode == "reset":
        for o in objs:
            p = o["initial_pose"]
            node.respawn(o, p[0], p[1], p[2], p[3])
    else:
        rng = random.Random(args.seed if args.seed is not None else int(time.time()))
        zone = cfg["dataset"]
        pos = sample_positions(len(objs), rng, zone["x_range"], zone["y_range"], zone.get("min_dist", 0.10))
        for o, (x, y) in zip(objs, pos):
            z = float(cfg["table_top_z"]) + float(o["height"]) / 2 + 0.001
            node.respawn(o, x, y, z, rng.uniform(-math.pi, math.pi))
            print(f"{o['model']}: ({x:.3f}, {y:.3f})")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
