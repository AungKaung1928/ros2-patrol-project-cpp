"""Launch test: with patrol_config.loop_enabled false the node visits each point once and stops.

The test writes its own patrol_points.yaml (2 points, loop off) into a temporary package
overlay placed first on AMENT_PREFIX_PATH, so the installed node loads it instead of the
shipped file. A stand-in navigate_to_pose server accepts every goal.
Run: launch_test test/test_launch_no_loop.py (after sourcing the workspace).
"""
import os
import tempfile
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

PKG = 'patrol_navigation_project_cpp'
CONFIG = '''patrol_points:
  - {name: "a", x: 1.0, y: 0.0, z: 0.0}
  - {name: "b", x: 0.0, y: 1.0, z: 0.0}
patrol_config:
  loop_enabled: false
  wait_at_waypoint: 0.1
  retry_on_failure: true
  max_retries: 3
'''


def make_overlay():
    root = tempfile.mkdtemp(prefix='patrol_overlay_')
    os.makedirs(os.path.join(root, 'share', 'ament_index', 'resource_index', 'packages'))
    open(os.path.join(root, 'share', 'ament_index', 'resource_index', 'packages', PKG), 'w').close()
    os.makedirs(os.path.join(root, 'share', PKG, 'config'))
    with open(os.path.join(root, 'share', PKG, 'config', 'patrol_points.yaml'), 'w') as f:
        f.write(CONFIG)
    return root


@pytest.mark.launch_test
def generate_test_description():
    overlay = make_overlay()
    env = dict(os.environ, AMENT_PREFIX_PATH=overlay + os.pathsep + os.environ.get('AMENT_PREFIX_PATH', ''))
    node = launch_ros.actions.Node(package=PKG, executable='patrol_controller', output='screen', env=env)
    return launch.LaunchDescription([launch_testing.actions.ReadyToTest(), node]), {'node': node}


class TestNoLoop(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_visits_each_point_once_then_stops(self, proc_output, node):
        fake = rclpy.create_node('fake_nav2')
        goals = []

        def execute(goal_handle):
            p = goal_handle.request.pose.pose.position
            goals.append((round(p.x, 3), round(p.y, 3)))
            goal_handle.succeed()
            return NavigateToPose.Result()

        server = ActionServer(fake, NavigateToPose, 'navigate_to_pose', execute)
        executor = MultiThreadedExecutor()
        executor.add_node(fake)
        try:
            start = time.monotonic()  # the node waits ~15 s for Nav2 before its first goal
            while len(goals) < 2 and time.monotonic() - start < 60.0:
                executor.spin_once(timeout_sec=0.1)
            end = time.monotonic() + 8.0  # a looping node would send point a again within ~2 s
            while time.monotonic() < end:
                executor.spin_once(timeout_sec=0.1)
        finally:
            server.destroy()
            executor.shutdown()
            fake.destroy_node()
        self.assertEqual(goals, [(1.0, 0.0), (0.0, 1.0)])
        proc_output.assertWaitFor('Patrol complete', process=node, timeout=5)
