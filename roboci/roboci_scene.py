#!/usr/bin/env python3
"""Mirrors Gazebo geometry into MoveIt's planning scene as collision objects.

MoveIt plans against its planning scene, not against Gazebo: without this
node the arm plans as if the workspace were empty, which is exactly the
missing_box negative control (the wrapper passes --no-box). The Gazebo
world never changes; only MoveIt's view of it does.

Adapted from RoboCI's UR example scene node: the planning frame and the
target-box geometry are arguments, since every arm puts its box elsewhere.

Usage: roboci_scene.py [--frame link1] [--no-box]
                       [--box-center "x y z"] [--box-size "sx sy sz"]
                       [--no-ground]
"""

import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive


def box(frame, object_id, size, center):
    obj = CollisionObject()
    obj.header.frame_id = frame
    obj.id = object_id
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = list(size)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = center
    pose.orientation.w = 1.0
    obj.primitives.append(primitive)
    obj.primitive_poses.append(pose)
    obj.operation = CollisionObject.ADD
    return obj


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frame", default="link1")
    ap.add_argument("--no-box", action="store_true")
    ap.add_argument("--box-center", default="0.20 -0.15 0.05")
    ap.add_argument("--box-size", default="0.10 0.10 0.10")
    ap.add_argument("--no-ground", action="store_true")
    args = ap.parse_args()

    box_center = tuple(float(v) for v in args.box_center.split())
    box_size = tuple(float(v) for v in args.box_size.split())

    rclpy.init()
    node = rclpy.create_node("roboci_scene")
    client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    if not client.wait_for_service(timeout_sec=120.0):
        node.get_logger().error("/apply_planning_scene never came up")
        return 1

    scene = PlanningScene()
    scene.is_diff = True
    if not args.no_ground:
        # The ground plane the arm stands on, as a slab whose top sits 2 mm
        # under the base frame: close enough to block through-floor plans,
        # far enough that the base link never starts in collision with it.
        scene.world.collision_objects.append(
            box(args.frame, "ground", (10.0, 10.0, 0.1), (0.0, 0.0, -0.052)))
    if not args.no_box:
        scene.world.collision_objects.append(
            box(args.frame, "target_box", box_size, box_center))

    request = ApplyPlanningScene.Request()
    request.scene = scene
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
    if not future.done() or future.result() is None or not future.result().success:
        node.get_logger().error("MoveIt refused the planning scene")
        return 1
    node.get_logger().info(
        f"planning scene updated ({'with' if not args.no_box else 'without'} target_box)")
    # Give move_group a moment to fold the diff in before the steps start;
    # the step engine's readiness wait covers the rest.
    time.sleep(1.0)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
