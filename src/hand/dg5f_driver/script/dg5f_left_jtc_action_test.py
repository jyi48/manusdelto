#!/usr/bin/env python3

# Copyright 2025 tesollo
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the tesollo nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


# Action-based example (control_msgs/action/FollowJointTrajectory).
# Unlike the topic-based *_jtc_test.py, this sends the whole trajectory as a
# single goal and waits for the controller's result/feedback, so the client
# knows whether the motion succeeded.

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


def d2r(deg):
    return deg * 3.141592 / 180.0


class JointTrajectoryActionClient(Node):
    def __init__(self):
        super().__init__('joint_trajectory_action_client')
        self._action_client = ActionClient(
            self, FollowJointTrajectory,
            '/dg5f_left/dg5f_left_controller/follow_joint_trajectory')
        self.joint_names = ['lj_dg_1_1', 'lj_dg_1_2', 'lj_dg_1_3', 'lj_dg_1_4', 'lj_dg_2_1', 'lj_dg_2_2', 'lj_dg_2_3', 'lj_dg_2_4', 'lj_dg_3_1', 'lj_dg_3_2', 'lj_dg_3_3', 'lj_dg_3_4', 'lj_dg_4_1', 'lj_dg_4_2', 'lj_dg_4_3', 'lj_dg_4_4', 'lj_dg_5_1', 'lj_dg_5_2', 'lj_dg_5_3', 'lj_dg_5_4']
        self.time_step = 2  # seconds between consecutive waypoints
        self.angles = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0942477796076938, 0.010471975511965978, 0.03490658503988659, 0.054105206811824215, 0.153588974175501, 1.4259339988793673, 1.6126842288427605, 0.6422811647339133, 0.019198621771937624, 1.3805554383275147, 1.5812683023068625, 0.7347836150896128, -0.006981317007977318, 1.4433872913993107, 1.6126842288427605, 0.8604473212332044, 0.012217304763960306, -0.10821041362364843, 1.541125729510993, 1.6580627893946132],
        ]

    def send_goal(self):
        self.get_logger().info('Waiting for action server ...')
        self._action_client.wait_for_server()

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        for i, positions in enumerate(self.angles):
            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start = Duration(
                sec=self.time_step * (i + 1), nanosec=0)
            goal_msg.trajectory.points.append(point)

        self.get_logger().info('Sending trajectory goal ({} waypoints)'.format(
            len(self.angles)))
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by controller')
            rclpy.shutdown()
            return
        self.get_logger().info('Goal accepted, waiting for result ...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Done. error_code = {}'.format(result.error_code))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Feedback desired: {}'.format(
            list(feedback.desired.positions)))


def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryActionClient()
    node.send_goal()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()
