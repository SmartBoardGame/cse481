import json
import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

POSE_FILE = "/tmp/poses.json"

class FrameListener(Node):

    def __init__(self):
        super().__init__('stretch_tf_listener')

        self.declare_parameter('target_frame', 'base_link')
        self.target_frame = self.get_parameter(
            'target_frame').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        time_period = 1.0 # seconds
        self.timer = self.create_timer(time_period, self.on_timer)

        self.latest_joint_states = {}

        self.create_subscription(JointState, "/stretch/joint_states", self.joint_states_cb, 10)

    def joint_states_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.latest_joint_states[name] = pos

    def on_timer(self):
        from_frame_rel = self.target_frame
        to_frame_rel = 'trash_can'

        try:
            now = Time()
            trans = self.tf_buffer.lookup_transform(
                to_frame_rel, #base_link
                from_frame_rel,
                now)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        x = math.trunc(trans.transform.translation.x * 100) / 100
        y = math.trunc(trans.transform.translation.y * 100) / 100
        z = math.trunc(trans.transform.translation.z * 100) / 100

        self.get_logger().info(
            f'the pose of target frame {from_frame_rel} with reference to {to_frame_rel} is:\nx: {x}\ny: {y}\nz: {z}')
        self.get_logger().info(
            f'arm extension: {sum([self.latest_joint_states[f"joint_arm_l{x}"] for x in range(0, 4)])}'
        )
        self.get_logger().info(
            f'lift: {self.latest_joint_states["joint_lift"]}'
        )
        # self.get_logger().info(
        #     f'the joint states: {self.latest_joint_states}'
        # )

        try:
            with open(POSE_FILE, "r") as f:
                poses = json.load(f)
        except Exception:
            poses = {}

        poses["current"] = {"position": {"x": x, "y": y, "z": z}}

        with open(POSE_FILE, "w") as f:
            json.dump(poses, f, indent=2)

# y bounds: -0.25 to -0.75 
# z bounds: 0.0 to 1.0
def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()