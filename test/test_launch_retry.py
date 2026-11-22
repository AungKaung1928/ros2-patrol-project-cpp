"""Launch test: the installed C++ patrol_controller against a stand-in Nav2 action server.

The test process serves navigate_to_pose and aborts every goal for the first patrol point.
With the shipped config (retry_on_failure: true, max_retries: 3) the node must try that
point 4 times (first try + 3 retries), then skip to the next point.
Run: launch_test test/test_launch_retry.py (after sourcing the workspace).
"""
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor

FIRST = (0.0, 0.0)    # config/patrol_points.yaml: origin_station
SECOND = (1.5, 1.8)   # config/patrol_points.yaml: north_corridor


@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(
        package='patrol_navigation_project_cpp', executable='patrol_controller', output='screen')
    return launch.LaunchDescription([launch_testing.actions.ReadyToTest(), node]), {'node': node}


class TestBoundedRetry(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_unreachable_point_is_skipped_after_max_retries(self, proc_output, node):
        fake = rclpy.create_node('fake_nav2')
        goals = []

        def execute(goal_handle):
            p = goal_handle.request.pose.pose.position
            goals.append((round(p.x, 3), round(p.y, 3)))
            if goals[-1] == FIRST:
                goal_handle.abort()
            else:
                goal_handle.succeed()
            return NavigateToPose.Result()

        server = ActionServer(fake, NavigateToPose, 'navigate_to_pose', execute)
        executor = MultiThreadedExecutor()
        executor.add_node(fake)
        try:
            start = time.monotonic()  # the node waits ~15 s for Nav2 before its first goal
            while len(goals) < 5 and time.monotonic() - start < 90.0:
                executor.spin_once(timeout_sec=0.1)
        finally:
            server.destroy()
            executor.shutdown()
            fake.destroy_node()
        self.assertGreaterEqual(len(goals), 5, f'goals seen: {goals}')
        self.assertEqual(goals[:5], [FIRST] * 4 + [SECOND])
        proc_output.assertWaitFor('Skipping waypoint', process=node, timeout=5)
