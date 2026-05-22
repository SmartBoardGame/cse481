#!/usr/bin/env python3

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState

import tf2_ros
from tf2_ros import TransformException

import sys
import time
from math import atan2, sqrt
import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import TransformStamped
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformException, Buffer, TransformListener
from tf_transformations import euler_from_quaternion, quaternion_matrix
from trajectory_msgs.msg import JointTrajectoryPoint
from action_msgs.msg import GoalStatus


POSE_FILE = "/tmp/poses.json"


class ArucoNavigator(Node):

    def __init__(self, node):
        super().__init__('aruco_navigator')
        self.node = node

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(String, "/save_pose", self.save_pose_cb, 10)
        self.create_subscription(JointState, "/stretch/joint_states", self.joint_states_cb, 10)

        self.create_subscription(String, "/run_pose", self.run_pose_cb, 10)
        self.create_subscription(JointState, "/run_sequence", self.run_sequence_cb, 10)

        self.poses = self.load_poses()
        self.latest_joint_states = {}

        self.trajectory_client = ActionClient(
            self.node,
            FollowJointTrajectory,
            "/stretch_controller/follow_joint_trajectory",
        )

        if not self.trajectory_client.wait_for_server(timeout_sec=60.0):
            self.node.get_logger().error("Unable to connect to trajectory server.")
            sys.exit()

        self.get_logger().info("Aruco Navigator running")

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
        
        desired_pose = self.poses[name]

        joints_list = [
            ("joint_lift", desired_pose['lift_height']),
            ("wrist_extension", desired_pose['wrist_extension']),
            ("joint_wrist_yaw", desired_pose['gripper_rpy']['joint_wrist_yaw']),
            ("joint_wrist_pitch", desired_pose['gripper_rpy']['joint_wrist_pitch']),
            ("joint_wrist_roll", desired_pose['gripper_rpy']['joint_wrist_roll'])
        ]

        self.send_base_goal_blocking(joints_list)

    def compute_difference(self, offset):
        # Extract quaternion and rotation matrix of marker in base_link frame
        trans_base = self.tf_buffer.lookup_transform(
                    "base_link", "trash_can", Time()
                )
        x, y, z, w = (
            trans_base.transform.rotation.x,
            trans_base.transform.rotation.y,
            trans_base.transform.rotation.z,
            trans_base.transform.rotation.w,
        )
        R = quaternion_matrix((x, y, z, w))

        # Apply rotation to the offset vector, positive Z DIRECTION IN OUR CASE
        P_dash = np.array([[0], [0], [offset], [1]])
        P = np.array(
            [
                [trans_base.transform.translation.x],
                [trans_base.transform.translation.y],
                [0],
                [1],
            ]
        )
        X = np.matmul(R, P_dash)

        # Compute the marker position with offset in base_link frame
        P_base = X + P
        P_base[3, 0] = 1  # Homogeneous coordinate

        # Extract adjusted position
        base_position_x = P_base[0, 0]
        base_position_y = P_base[1, 0]

        # Compute rotation and translation needed
        phi = atan2(base_position_y, base_position_x)
        dist = sqrt(base_position_x**2 + base_position_y**2)

        _, _, z_rot_base = euler_from_quaternion([x, y, z, w])
        # Calculate final rotation: -phi (cancel rotation needed to align),
        # + z_rot_base (original marker rotation),
        # + pi (such that the base and the marker axis are aligned as shown in tutorial)
        z_rot_base = -phi + z_rot_base + np.pi

        return phi, dist, z_rot_base


    def send_base_goal_blocking(self, joints_list):
        point = JointTrajectoryPoint()
        point.positions = [inc for _, inc in joints_list]
        point.time_from_start = Duration(seconds=5.0).to_msg()

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [joint_name for joint_name, _ in joints_list]
        goal.trajectory.points = [point]

        joint_names_str = ", ".join(goal.trajectory.joint_names)
        self.node.get_logger().info(f"Sending goal for joints: [{joint_names_str}]")

        send_goal_future = self.trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.node.get_logger().error(f"Goal for joints [{joint_names_str}] was rejected!")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result()

        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().warn(
                f"Goal for joints [{joint_names_str}] did not succeed: status {result.status}"
            )
        else:
            self.node.get_logger().info(f"Goal for joints [{joint_names_str}] succeeded.")
                
    def align_to_marker(self, offset):
        phi, dist, final_theta = self.compute_difference(offset)

        # Concurrent turn and drive
        self.send_base_goal_blocking([
            ("rotate_mobile_base", phi),
            ("translate_mobile_base", dist)
        ])

        # Final alignment turn
        self.send_base_goal_blocking([("rotate_mobile_base", final_theta)])


def main():
    rclpy.init()
    node = Node("aruco_navigator")
    aruco = ArucoNavigator(node=node)
    rclpy.spin(aruco)
    aruco.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()