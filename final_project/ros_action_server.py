import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState

import tf2_ros
from tf2_ros import TransformException, Buffer, TransformListener
from tf_transformations import euler_from_quaternion, quaternion_matrix

import sys
import time
from math import atan2, sqrt
import numpy as np
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import TransformStamped
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time
from trajectory_msgs.msg import JointTrajectoryPoint
from action_msgs.msg import GoalStatus

CAN_START_POSE_FILE = "/home/hello-robot/kevin/cse481/final_project/aruco_data/trash_start.json" # this is how stretch approaches the can
CAN_PICKUP_POSE_FILE = "/home/hello-robot/kevin/cse481/final_project/joint_state_data/trash_pickup.json" # this is the extraction poses
RECEPTACLE_START_POSE_FILE = "/home/hello-robot/kevin/cse481/final_project/aruco_data/receptacle_start.json" # this is the approach pose for the receptacle
RECEPTACLE_DROP_POSE_FILE = "/home/hello-robot/kevin/cse481/final_project/joint_state_data/receptacle_drop.json" # this is the pose for dropping the bag into it

TRASH_CAN_OFFSET_ORIENTATION = np.pi
RECEPTACLE_OFFSET_ORIENTATION = np.pi/2

class WasteDisposal(Node):
    def __init__(self, node):
        super().__init__('waste_disposal')
        self.node = node

        # TF and Action Client setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.trajectory_client = ActionClient(
            self.node,
            FollowJointTrajectory,
            "/stretch_controller/follow_joint_trajectory",
        )

        if not self.trajectory_client.wait_for_server(timeout_sec=10.0):
            seld.node.get_logger().error("Unable to connect to trajectory server.")
            
        # Subscriber
        self.subscription = self.create_subscription(
            String,
            'task_execution',
            self.task_callback,
            10
        )
        self.node.get_logger().info("Waste Disposal node started and listening to /task_execution.")

    def task_callback(self, msg):
        task_type = msg.data.strip().lower()
        self.node.get_logger().info(f"Received task: {task_type}")
        
        handler = getattr(self, f"execute_{task_type}", None)
        
        if not handler:
            self.node.get_logger().error(f"Unknown task type: {task_type}")
            return
        
        handler()

    def load_poses(self, file_path):
        try:
            with open(file_path, "r") as f:
                return json.load(f)
        except Exception as e:
            self.node.get_logger().error(f"Could not load poses from {file_path}: {e}")
            return {}

    def send_base_goal_blocking(self, joints_list):
        point = JointTrajectoryPoint()
        point.positions = [float(inc) for _, inc in joints_list]
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
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result()

        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().warn(f"Goal for joints [{joint_names_str}] did not succeed: status {result.status}")
            return False
        
        self.node.get_logger().info(f"Goal for joints [{joint_names_str}] succeeded.")
        return True

    def compute_difference(self, target_frame, offset_x=0, offset_y=0, offset_z=0, offset_orientation=0):
        try:
            self.node.get_logger().info(f"aligning to offsets offset_x {offset_x}, offset_y {offset_y}, offset_z {offset_z}")

            # Extract quaternion and rotation matrix of marker in base_link frame
            trans_base = self.tf_buffer.lookup_transform(
                    "base_link", target_frame, Time()
                    )
            x, y, z, w = (
                trans_base.transform.rotation.x,
                trans_base.transform.rotation.y,
                trans_base.transform.rotation.z,
                trans_base.transform.rotation.w,
            )
            R = quaternion_matrix((x, y, z, w))

            # Apply rotation to the offset vector, positive Z DIRECTION IN OUR CASE
            P_dash = np.array([[offset_x], [offset_y], [offset_z], [1]])
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
            
            # offset_orientation: np.pi for trash can start, np.pi/2 for receptacle
            z_rot_base = -phi + z_rot_base + offset_orientation

            return phi, dist, z_rot_base
        except TransformException as e:
            self.node.get_logger().error(f"Transform error: {e}")
            return None, None, None

    def align_to_marker(self, target_frame, offset_x=0, offset_y=0, offset_z=0, offset_orientation=0):
        self.get_logger().info(f"Aligning to {target_frame} with offset z={offset_z}")
        phi, dist, final_theta = self.compute_difference(target_frame, offset_x, offset_y, offset_z, offset_orientation)
        
        if phi is None:
            return False

        self.send_base_goal_blocking([("rotate_mobile_base", phi)])
        self.send_base_goal_blocking([("translate_mobile_base", dist)])
        self.send_base_goal_blocking([("rotate_mobile_base", final_theta)])
        return True

    def execute_named_pose_from_dict(self, pose_data):
        # pose_data could be the dict from the JSON
        joints_list = [
            ("joint_lift", pose_data.get('lift_height', 0.5)),
            ("wrist_extension", pose_data.get('wrist_extension', 0.1)),
            ("joint_wrist_yaw", pose_data.get('gripper_rpy', {}).get('joint_wrist_yaw', 0.0)),
            ("joint_wrist_pitch", pose_data.get('gripper_rpy', {}).get('joint_wrist_pitch', 0.0)),
            ("joint_wrist_roll", pose_data.get('gripper_rpy', {}).get('joint_wrist_roll', 0.0))
        ]
        return self.send_base_goal_blocking(joints_list)

    def execute_extraction(self):
        # Approach
        self.node.get_logger().info("Executing navigation (approaching trash can)...")
        
        start_poses = self.load_poses(CAN_START_POSE_FILE)
        if "trash_start" in start_poses:
            pose = start_poses["trash_start"]
            target_frame = pose.get("frame", "trash_can")
            offset_z = pose.get("position", {}).get("z", 0.0)
            if self.align_to_marker(target_frame, offset_z=offset_z, offset_orientation=TRASH_CAN_OFFSET_ORIENTATION):
                self.execute_named_pose_from_dict(pose)
        
        # Extraction
        self.node.get_logger().info("Executing extraction (picking up trash)...")

        pickup_poses = self.load_poses(CAN_PICKUP_POSE_FILE)
        # Sequence: before_pickup -> (grip) -> pickup_high -> pickup_retracted
        for pose_name in ["before_pickup", "pickup_high", "pickup_retracted"]:
            if pose_name in pickup_poses:
                self.node.get_logger().info(f"Executing pose: {pose_name}")
                self.execute_named_pose_from_dict(pickup_poses[pose_name])
                time.sleep(5.0)

    def execute_disposal(self):
        # Approach
        self.node.get_logger().info("Executing navigation (approaching receptacle)...")

        start_poses = self.load_poses(RECEPTACLE_START_POSE_FILE)
        if "receptacle_start" in start_poses:
            pose = start_poses["receptacle_start"]
            target_frame = pose.get("frame", "receptacle")
            offset_z = pose.get("position", {}).get("z", 0.0)
            if self.align_to_marker(target_frame, offset_z=offset_z, offset_orientation=RECEPTACLE_OFFSET_ORIENTATION):
                self.execute_named_pose_from_dict(pose)
        
        # Drop
        self.node.get_logger().info("Executing disposal (dropping into receptacle)...")

        drop_poses = self.load_poses(RECEPTACLE_DROP_POSE_FILE)
        if "receptacle_drop" in drop_poses:
            self.node.get_logger().info("Dropping trash...")
            self.execute_named_pose_from_dict(drop_poses["receptacle_drop"])

def main(args=None):
    rclpy.init(args=args)
    node = Node("waste_disposal")
    waste_disposal = WasteDisposal(node=node)
    try:
        rclpy.spin(waste_disposal)
    except KeyboardInterrupt:
        pass
    waste_disposal.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
