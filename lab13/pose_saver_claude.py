#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import tf2_ros
from tf2_ros import TransformException

POSE_FILE = "/tmp/poses.json"

TRACKED_JOINTS = [
    'joint_lift',
    'joint_arm_l0',
    'joint_arm_l1',
    'joint_arm_l2',
    'joint_arm_l3',
    'joint_wrist_yaw',
    'joint_wrist_pitch',
    'joint_wrist_roll',
    'joint_gripper_finger_left',
]

class PoseDatabase(Node):

    def __init__(self):
        super().__init__('pose_database')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.latest_joint_states = {}

        self.create_subscription(JointState, "/stretch/joint_states", self.joint_states_cb, 10)
        self.create_subscription(String, "/save_pose", self.save_pose_cb, 10)

        self.poses = self.load_poses()
        self.get_logger().info("PoseDatabase running")

    def joint_states_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.latest_joint_states[name] = pos

    def load_poses(self):
        try:
            with open(POSE_FILE, "r") as f:
                return json.load(f)
        except Exception:
            return {}

    def save_file(self):
        with open(POSE_FILE, "w") as f:
            json.dump(self.poses, f, indent=2)

    def save_pose_cb(self, msg):
        data = json.loads(msg.data)
        name = data["name"]
        frame = data.get("frame", "base_link")
        self.get_logger().info(f"Saving pose '{name}' in frame '{frame}'")

        # --- TF ---
        try:
            t = self.tf_buffer.lookup_transform(
                frame,
                "link_grasp_center",
                rclpy.time.Time()
            )
            tf_data = {
                "frame": frame,
                "position": {
                    "x": t.transform.translation.x,
                    "y": t.transform.translation.y,
                    "z": t.transform.translation.z,
                },
                "orientation": {
                    "x": t.transform.rotation.x,
                    "y": t.transform.rotation.y,
                    "z": t.transform.rotation.z,
                    "w": t.transform.rotation.w,
                }
            }
        except TransformException as e:
            self.get_logger().error(f"TF lookup failed: {e}")
            tf_data = None

        # --- Joint states ---
        if not self.latest_joint_states:
            self.get_logger().warn("No joint states received yet — saving TF only")
            joint_data = {}
        else:
            joint_data = {
                j: self.latest_joint_states[j]
                for j in TRACKED_JOINTS
                if j in self.latest_joint_states
            }
            arm_segments = ['joint_arm_l0', 'joint_arm_l1', 'joint_arm_l2', 'joint_arm_l3']
            joint_data['joint_arm_total'] = sum(
                self.latest_joint_states.get(seg, 0.0) for seg in arm_segments
            )

        self.poses[name] = {
            "tf": tf_data,
            "joints": joint_data,
        }
        self.save_file()
        self.get_logger().info(f"Saved pose '{name}' with joints: {list(joint_data.keys())}")


def main():
    rclpy.init()
    node = PoseDatabase()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()