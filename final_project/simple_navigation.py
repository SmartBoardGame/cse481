#! /usr/bin/env python3
from copy import deepcopy
from geometry_msgs.msg import PoseStamped
from stretch_nav2.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

def main():
    rclpy.init()
    navigator = BasicNavigator()

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 1.8
    goal_pose.pose.position.y = 1.8
    goal_pose.pose.orientation.w = 1.0

    navigator.get_logger().info('Going to receptacle...')
    navigator.goToPose(goal_pose)

    i = 0
    nav_start = navigator.get_clock().now()
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            navigator.get_logger().info('Navigating to goal...')
        if navigator.get_clock().now() - nav_start > Duration(seconds=60.0):
            navigator.get_logger().warn('Timed out, cancelling.')
            navigator.cancelTask()

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info('Reached receptacle!')
    elif result == TaskResult.CANCELED:
        navigator.get_logger().info('Navigation canceled.')
    elif result == TaskResult.FAILED:
        navigator.get_logger().info('Navigation failed.')

    rclpy.shutdown()

if __name__ == '__main__':
    main()