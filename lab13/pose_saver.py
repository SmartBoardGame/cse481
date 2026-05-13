#!/usr/bin/env python3

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState

import tf2_ros
from tf2_ros import TransformException

POSE_FILE = "/tmp/poses.json"


class PoseDatabase(Node):

    def __init__(self):
        super().__init__('pose_database')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(String, "/save_pose", self.save_pose_cb, 10)
        self.create_subscription(JointState, "/stretch/joint_states", self.joint_states_cb, 10)

        self.poses = self.load_poses()
        self.latest_joint_states = {}


        self.get_logger().info("PoseDatabase running")

    def joint_states_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.latest_joint_states[name] = pos


    def load_poses(self):
        try:
            with open(POSE_FILE, "r") as f:
                return json.load(f)
        except:
            return {}

    def save_file(self):
        with open(POSE_FILE, "w") as f:
            json.dump(self.poses, f, indent=2)

    def save_pose_cb(self, msg):
        import json as pyjson
        data = pyjson.loads(msg.data)

        name = data["name"]
        frame = data["frame"]

        try:
            t = self.tf_buffer.lookup_transform(
                frame,
                "base_link",
                rclpy.time.Time()
            )

            self.poses[name] = {
                "frame": frame,
                "position": {
                    "x": t.transform.translation.x,
                    "y": t.transform.translation.y,
                    "z": t.transform.translation.z
                },
                "orientation": {
                    "x": t.transform.rotation.x,
                    "y": t.transform.rotation.y,
                    "z": t.transform.rotation.z,
                    "w": t.transform.rotation.w
                },
                "gripper_rpy": {
                    "joint_wrist_roll": self.latest_joint_states["joint_wrist_roll"],
                    "joint_wrist_pitch": self.latest_joint_states["joint_wrist_pitch"],
                    "joint_wrist_yaw": self.latest_joint_states["joint_wrist_yaw"]
                },
                "lift_height": self.latest_joint_states["joint_lift"],
                "wrist_extension": self.latest_joint_states["joint_arm_l0"] + self.latest_joint_states["joint_arm_l1"] + self.latest_joint_states["joint_arm_l2"] + self.latest_joint_states["joint_arm_l3"]
            }

            self.save_file()
            self.get_logger().info(f"Saved pose {name}")

        except TransformException as e:
            self.get_logger().error(str(e))


def main():
    rclpy.init()
    node = PoseDatabase()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()