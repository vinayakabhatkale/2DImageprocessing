#!/usr/bin/env python3
import math
import threading
import time
from threading import Event

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from modproft_robot_control_msgs.action import MoveCartesian, MoveJoint, MovePose


class MoveItActionClient(Node):
    def __init__(self):
        super().__init__('moveit_action_client')
        self.action_done_event = Event()
        self.result = False

        self._movecartesian_client = ActionClient(self, MoveCartesian, 'movecartesian')
        self._movejoint_client = ActionClient(self, MoveJoint, 'movejoint')
        self._movepose_client = ActionClient(self, MovePose, 'movepose')


    def move_cartesian(self, pos):
        self.action_done_event.clear()
        self.result = False

        goal_msg = MoveCartesian.Goal()
        goal_msg.posevalues = pos

        self._movecartesian_client.wait_for_server()

        self._send_goal_future = self._movecartesian_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        self.action_done_event.wait()

    def move_joint(self, pos):
        self.action_done_event.clear()
        self.result = False

        goal_msg = MoveJoint.Goal()
        goal_msg.jointvalues = pos

        self._movejoint_client.wait_for_server()

        self._send_goal_future = self._movejoint_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        self.action_done_event.wait()

    def move_pose(self, pos):
        self.action_done_event.clear()
        self.result = False

        goal_msg = MovePose.Goal()
        goal_msg.posevalues = pos

        self._movepose_client.wait_for_server()

        self._send_goal_future = self._movepose_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        self.action_done_event.wait()


    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.result = result.success
        self.get_logger().info('Result: {0}'.format(result.success))
        self.action_done_event.set()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.status))


def run_executor(executor, n):
    executor.add_node(n)
    try:
        while True:
            executor.spin_once()
            time.sleep(0.01)
    finally:
        executor.shutdown()

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q


def main(args=None):
    rclpy.init(args=args)

    node = MoveItActionClient()

    exec = MultiThreadedExecutor(num_threads=8)
    thread = threading.Thread(target=run_executor, args=(exec, node), daemon=True)
    thread.start()

    pose = [0.2, 0.2, 0.5, 0.0, 0.0, 0.0, 0.0]

    q = quaternion_from_euler(np.pi/2, 0.0, 0.0)
    pose[3] = q[0]
    pose[4] = q[1]
    pose[5] = q[2]
    pose[6] = q[3]

    node.move_pose(pose)

if __name__ == '__main__':
    main()
