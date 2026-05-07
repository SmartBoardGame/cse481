#!/usr/bin/env python3
import json
import time
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from std_msgs.msg import String
import tf2_ros
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

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        pose_cb_group     = ReentrantCallbackGroup()
        sequence_cb_group = MutuallyExclusiveCallbackGroup()

        self.create_subscription(
            String, "/run_pose", self.run_pose_cb, 10,
            callback_group=pose_cb_group
        )
        self.create_subscription(
            String, "/run_sequence", self.run_sequence_cb, 10,
            callback_group=sequence_cb_group
        )

        self.get_logger().info("PoseExecutor READY")

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
            time.sleep(1.5)

    def execute_named_pose(self, name):
        poses = self.load_poses()
        if name not in poses:
            self.get_logger().warn(f"Pose not found: {name}")
            return False

        pose = poses[name]

        # Prefer joint states (accurate) over TF-derived positions
        if "joints" in pose and pose["joints"]:
            self.execute_motion_from_joints(pose["joints"])
        elif "tf" in pose and pose["tf"]:
            self.get_logger().warn(f"No joint data for '{name}', falling back to TF")
            p = pose["tf"]["position"]
            self.execute_motion_from_xyz(p["x"], p["y"], p["z"])
        else:
            self.get_logger().error(f"Pose '{name}' has no usable data")
            return False

        return True

    def execute_motion_from_joints(self, joints):
        """Execute motion using saved joint positions directly — no axis mapping needed."""
        lift        = joints.get('joint_lift')
        arm         = joints.get('joint_arm_total')
        wrist_yaw   = joints.get('joint_wrist_yaw')
        wrist_pitch = joints.get('joint_wrist_pitch')
        wrist_roll  = joints.get('joint_wrist_roll')
        gripper     = joints.get('joint_gripper_finger_left')

        self.get_logger().info(
            f"Executing joints: lift={lift:.3f} arm={arm:.3f} "
            f"wrist_yaw={wrist_yaw:.3f} wrist_pitch={wrist_pitch:.3f}"
        )

        try:
            if lift is not None:
                self.move_to_pose({'joint_lift': lift}, blocking=True)

            if arm is not None:
                arm = max(0.0, min(arm, 0.52))  # Stretch hardware limit
                self.move_to_pose({'joint_arm': arm}, blocking=True)

            if wrist_yaw is not None:
                self.move_to_pose({'joint_wrist_yaw': wrist_yaw}, blocking=True)

            if wrist_pitch is not None:
                self.move_to_pose({'joint_wrist_pitch': wrist_pitch}, blocking=True)

            if wrist_roll is not None:
                self.move_to_pose({'joint_wrist_roll': wrist_roll}, blocking=True)

            if gripper is not None:
                self.move_to_pose({'joint_gripper_finger_left': gripper}, blocking=True)

        except Exception as e:
            self.get_logger().error(f"Motion failed: {e}")

    def execute_motion_from_xyz(self, x, y, z):
        """Fallback: derive joint targets from TF position (less accurate)."""
        self.get_logger().info(f"Executing from TF xyz: {x:.3f}, {y:.3f}, {z:.3f}")
        base_translation = max(-0.3, min(x, 0.3))
        lift_height      = max(0.2,  min(z, 1.1))
        arm_extension    = max(0.0,  min(y, 0.52))
        try:
            self.move_to_pose({'translate_mobile_base': base_translation}, blocking=True)
            self.move_to_pose({'joint_lift': lift_height}, blocking=True)
            self.move_to_pose({'joint_arm': arm_extension}, blocking=True)
        except Exception as e:
            self.get_logger().error(f"Motion failed: {e}")


def main():
    node = PoseExecutor()
    node.main()

if __name__ == "__main__":
    main()