"""Launch test (same as the Python version): a goal rejected by Nav2 must be handled like a failure (retry the same point),
not stall the patrol. The stand-in navigate_to_pose server rejects the first goal and accepts
and succeeds every later one.
Run: launch_test test/test_launch_rejected.py (after sourcing the workspace).
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
from rclpy.action import ActionServer, GoalResponse
from rclpy.executors import MultiThreadedExecutor

PKG = 'patrol_navigation_project_cpp'


@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(package=PKG, executable='patrol_controller', output='screen')
    return launch.LaunchDescription([launch_testing.actions.ReadyToTest(), node]), {'node': node}


class TestRejected(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_rejected_goal_is_retried(self):
        fake = rclpy.create_node('fake_nav2')
        requests = []

        def on_goal(request):
            p = request.pose.pose.position
            requests.append((round(p.x, 3), round(p.y, 3)))
            return GoalResponse.REJECT if len(requests) == 1 else GoalResponse.ACCEPT

        def execute(goal_handle):
            goal_handle.succeed()
            return NavigateToPose.Result()

        server = ActionServer(fake, NavigateToPose, 'navigate_to_pose', execute, goal_callback=on_goal)
        executor = MultiThreadedExecutor()
        executor.add_node(fake)
        try:
            start = time.monotonic()  # the node waits ~17 s for Nav2 before its first goal
            while len(requests) < 3 and time.monotonic() - start < 60.0:
                executor.spin_once(timeout_sec=0.1)
        finally:
            server.destroy()
            executor.shutdown()
            fake.destroy_node()
        self.assertGreaterEqual(len(requests), 3, f'goal requests seen: {requests} (stalled after a rejection?)')
        self.assertEqual(requests[0], requests[1], 'the rejected point was not retried')
