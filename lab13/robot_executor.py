#!/usr/bin/env python3
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException
import hello_helpers.hello_misc as hm

POSE_FILE = "/tmp/poses.json"

class PoseExecutor(hm.HelloNode):

    def load_poses(self):
        try:
            with open(POSE_FILE, "r") as f:
                return json.load(f)
        except Exception:
            return {}

    def __init__(self):
        hm.HelloNode.__init__(self)

    def main(self):
        hm.HelloNode.main(
            self,
            'pose_executor',
            'pose_executor',
            wait_for_first_pointcloud=False
        )

        # NOW ROS NODE IS READY

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ReentrantCallbackGroup: allows single poses to run concurrently with the executor
        pose_cb_group = ReentrantCallbackGroup()

        # MutuallyExclusiveCallbackGroup: prevents two sequences from running simultaneously
        sequence_cb_group = MutuallyExclusiveCallbackGroup()

        self.create_subscription(
            String,
            "/run_pose",
            self.run_pose_cb,
            10,
            callback_group=pose_cb_group       # ✅ unblocks move_to_pose(blocking=True)
        )
        self.create_subscription(
            String,
            "/run_sequence",
            self.run_sequence_cb,
            10,
            callback_group=sequence_cb_group   # ✅ prevents overlapping sequences
        )

        self.get_logger().info("PoseExecutor READY")

        # ✅ MultiThreadedExecutor lets callbacks run in parallel threads,
        #    so blocking motion calls don't deadlock the executor
        executor = MultiThreadedExecutor()
        executor.add_node(self)
        executor.spin()

    def run_pose_cb(self, msg):
        name = msg.data.strip()
        if not name:
            self.get_logger().warn("Empty pose name received")
            return
        self.get_logger().info(f"Running pose: {name}")
        self.execute_named_pose(name)

    def run_sequence_cb(self, msg):
        try:
            sequence = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"Invalid sequence JSON: {e}")
            return
        if not isinstance(sequence, list):
            self.get_logger().error("Sequence must be a list of pose names")
            return
        self.get_logger().info(f"Running sequence: {sequence}")
        for name in sequence:
            name = str(name).strip()
            if not name:
                continue
            self.get_logger().info(f"Executing pose in sequence: {name}")
            success = self.execute_named_pose(name)
            if not success:
                self.get_logger().warn(f"Failed pose: {name}")
            time.sleep(1.5)  # small delay between motions

    def execute_named_pose(self, name):
        self.poses = self.load_poses()
        if name not in self.poses:
            self.get_logger().warn(f"Pose not found: {name}")
            return False
        pose = self.poses[name]
        x = math.trunc(pose["position"]["x"] * 100) / 100
        y = math.trunc(pose["position"]["y"] * 100) / 100
        z = math.trunc(pose["position"]["z"] * 100) / 100
        self.execute_motion_from_xyz(x, y, z)
        return True

    def execute_motion_from_xyz(self, x, y, z):
        self.get_logger().info(f"Executing raw pose: {x}, {y}, {z}")
        base_translation = max(-0.3, min(x, 0.3))
        lift_height      = max(0.2,  min(z, 1.0))
        arm_extension    = max(0.0,  min((-y-0.25)/0.75, 1.0))
        self.get_logger().info(f"Executing raw pose2222: base {base_translation}, lift {lift_height}, arm {arm_extension}")

        try:
            self.move_to_pose(
                {'translate_mobile_base': base_translation},
                blocking=True
            )
            self.move_to_pose(
                {'joint_lift': lift_height},
                blocking=True
            )
            self.move_to_pose(
                {'joint_arm': arm_extension},
                blocking=True
            )
        except Exception as e:
            self.get_logger().error(f"Motion failed: {e}")


def main():
    node = PoseExecutor()
    node.main()

if __name__ == "__main__":
    main()