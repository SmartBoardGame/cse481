#!/usr/bin/env python3

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import tf2_ros
from tf2_ros import TransformException

POSE_FILE = "/tmp/poses.json"


class PoseDatabase(Node):

    def __init__(self):
        super().__init__('pose_database')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(String, "/save_pose", self.save_pose_cb, 10)

        self.poses = self.load_poses()

        self.get_logger().info("PoseDatabase running")

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
        print("frame", frame)

        try:
            t = self.tf_buffer.lookup_transform(
                frame,
                "link_grasp_center",
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
                }
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