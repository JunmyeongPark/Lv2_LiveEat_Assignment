"""Sensor diagnostics, input freshness and main-node recovery (no hardware required).

source /opt/ros/jazzy/setup.bash
PYTHONPATH=lv2_module5/ros2_ws/src/planning:$PYTHONPATH python3 -m unittest discover \
    -s lv2_module5/ros2_ws/src/planning/test -v
"""
import math
import unittest
from unittest.mock import Mock

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32

from planning.health_monitor import SENSORS, _level_to_int
from planning.planning_master import PlanningMaster


class HealthIntegrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(args=[])
        cls.node = PlanningMaster()
        cls.original_monitor = cls.node.health_monitor
        # Capture output while exercising actual callbacks, message types and main loop.
        cls.node.cmd_vel_pub = Mock()
        cls.node.arm_cmd_pub = Mock()
        cls.node.status_pub = Mock()

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        self.now = 10.0
        n = self.node
        n._now_s = lambda: self.now
        n.health_enabled = True
        n.health_monitor = self.original_monitor
        n.health_monitor._now = lambda: self.now
        n.health_monitor._last_rx = dict.fromkeys(SENSORS)
        n.health_monitor._ok_cnt = dict.fromkeys(SENSORS, 0)
        n.health_monitor._warn_ok = True
        n.state = 'idle'
        n.recovered_from = None
        n.last_fault_reason = None
        n.heading_source = 'none'
        n.health_blocked = False
        n.cur_arm_pose_valid = False
        n.cur_arm_pose = [0.0, 0.0]
        n.last_cam_time = n.last_imu_time = n.last_joint_time = n.last_odom_time = None
        n.cur_depth = 0.0
        n.camera_input_valid = False
        n.detect_streak = 0
        n.search_rot_deg = 0.0
        n.search_cnt = 0
        n.tgt_arm_pose = [30.0, 20.0]
        for pub in (n.cmd_vel_pub, n.arm_cmd_pub, n.status_pub):
            pub.reset_mock()

    def diag(self, sensor, level=0):
        msg = DiagnosticStatus()
        msg.level = bytes([level])
        self.node.health_monitor._cb(sensor, msg)

    def diagnostics(self, count=3):
        for _ in range(count):
            for sensor in SENSORS:
                self.diag(sensor)

    def imu(self, yaw=0.0):
        msg = Imu()
        msg.orientation.z = math.sin(math.radians(yaw) / 2)
        msg.orientation.w = math.cos(math.radians(yaw) / 2)
        self.node.imu_cb(msg)

    def frame(self, depth=1.0, ex=0.0, ey=0.0):
        msg = PointStamped()
        msg.point.x, msg.point.y, msg.point.z = ex, ey, depth
        self.node.cam_cb(msg)

    def joints(self, pan=0.0, tilt=0.0):
        msg = JointState()
        msg.name = [self.node.pan_joint_name, self.node.tilt_joint_name]
        msg.position = [math.radians(pan), math.radians(tilt)]
        self.node.motor_cb(msg)

    def healthy(self):
        self.diagnostics()
        self.imu()
        self.joints()
        for _ in range(3):
            self.frame()
        self.node.run()  # idle -> tracking
        self.node.run()
        self.assertEqual(self.node.state, 'tracking')
        self.assertFalse(self.node.health_blocked)

    def assert_zero_arm_cmd(self):
        msg = self.node.arm_cmd_pub.publish.call_args[0][0]
        self.assertEqual(list(msg.data), [0.0, 0.0])

    def test_startup_without_health_faults_with_zero_command(self):
        self.node.run()
        self.assertEqual(self.node.state, 'fault')
        self.assertEqual(self.node.health_reason, 'mcu_fault')
        self.assertEqual(self.node.cmd_vel_msg.linear.x, 0)
        self.assert_zero_arm_cmd()

    def test_healthy_tracking_has_one_status_and_five_health_subscriptions(self):
        self.healthy()
        self.node.status_pub.reset_mock()
        self.node.run()
        self.assertEqual(len(self.node.health_monitor._subscriptions), 5)
        self.assertEqual(self.node.status_pub.publish.call_count, 1)
        self.assertGreater(self.node.cmd_vel_msg.linear.x, 0)
        self.assertIn('reason=ok heading=imu', self.node._status_text())

    def test_fault_priority_and_zero_arm_command(self):
        self.healthy()
        for sensor in ('camera', 'wheel_motor', 'mcu'):
            self.diag(sensor, 2)
        self.node.arm_cmd_pub.reset_mock()
        self.node.run()
        self.assertEqual(self.node.state, 'fault')
        self.assertEqual(self.node.health_reason, 'mcu_fault')
        self.assert_zero_arm_cmd()
        for _ in range(3):
            self.diag('mcu')
        self.node.run()
        self.assertEqual(self.node.health_reason, 'motor_fault')
        self.assertEqual(self.node.cmd_vel_msg.angular.z, 0)

    def test_camera_fault_commands_zero_pose_and_stays_in_fault(self):
        self.healthy()
        self.joints(30, 10)
        self.diag('camera', 3)
        self.node.run()
        self.assertEqual(self.node.state, 'fault')
        self.assertEqual(self.node.tgt_arm_pose, [0.0, 0.0])
        self.node.run()
        self.assertEqual(self.node.state, 'fault')
        self.assertIn('camera_stale', self.node._status_text())

    def test_recovery_needs_three_health_messages_then_new_detection_frames(self):
        self.healthy()
        self.diag('camera', 2)
        self.node.run()
        for _ in range(2):
            self.diag('camera')
            self.frame()
            self.node.run()
            self.assertTrue(self.node.health_blocked)
        self.diag('camera')
        self.node.run()  # fault -> idle
        self.assertEqual(self.node.state, 'idle')
        for _ in range(2):
            self.frame()
            self.node.run()
            self.assertEqual(self.node.state, 'idle')
        self.frame()
        self.node.run()
        self.assertEqual(self.node.state, 'tracking')

    def test_recovery_reason_shown_then_cleared(self):
        self.healthy()
        self.node.last_cam_time = self.now - 1          # detection_timeout -> FAULT
        self.node.run()
        self.assertEqual(self.node.state, 'fault')
        self.assertIn('reason=detection_timeout', self.node._status_text())
        self.frame()
        self.node.run()                                 # FAULT -> IDLE
        self.assertEqual(self.node.state, 'idle')
        self.assertEqual(self.node.health_reason, 'ok')  # 내부 판정값은 그대로
        self.assertIn('reason=recovered_from:detection_timeout', self.node._status_text())
        self.now += self.node.recovery_reason_hold_s + 0.1
        self.diagnostics()
        self.imu()
        self.joints()
        self.frame()
        self.node.run()
        self.assertIn('reason=ok', self.node._status_text())

    def test_health_gap_resets_count_even_without_evaluate(self):
        self.diagnostics()
        self.now += .6
        self.diag('camera')
        self.assertEqual(self.node.health_monitor._ok_cnt['camera'], 1)
        self.assertFalse(self.node.health_monitor.evaluate().camera_ok)

    def test_backward_clock_resets_health(self):
        self.diagnostics()
        self.now -= 1
        self.assertEqual(self.node.health_monitor.evaluate().reason, 'mcu_fault')

    def test_warn_policy_and_byte_levels(self):
        self.diagnostics()
        self.diag('camera', 1)
        self.assertTrue(self.node.health_monitor.evaluate().camera_ok)
        self.node.health_monitor._warn_ok = False
        self.diag('camera', 1)
        self.assertFalse(self.node.health_monitor.evaluate().camera_ok)
        self.assertEqual(_level_to_int(b'\x02'), 2)
        self.assertEqual(_level_to_int(3), 3)

    def test_imu_fault_uses_encoder_and_resets_search_baseline(self):
        self.healthy()
        self.node.state = 'searching'
        self.node.detect_streak = 0
        self.frame(0.0)
        self.node.search_rot_deg = 100.0
        self.node.prev_yaw_deg = 10.0
        self.diag('imu', 2)
        self.node.odom_yaw_cb(Float32(data=170.0))
        self.node.run()
        self.assertEqual(self.node.heading_source, 'encoder')
        self.assertEqual(self.node.health_reason, 'imu_fallback')
        self.assertAlmostEqual(self.node.search_rot_deg, 100)
        self.node.odom_yaw_cb(Float32(data=-170.0))
        self.node.run()
        self.assertAlmostEqual(self.node.search_rot_deg, 120)
        self.imu(-30)
        for _ in range(3):
            self.diag('imu')
        self.node.run()
        self.assertEqual(self.node.heading_source, 'imu')
        self.assertAlmostEqual(self.node.search_rot_deg, 120)

    def test_imu_silence_also_uses_encoder_even_when_health_ok(self):
        self.healthy()
        self.node.last_imu_time = self.now - 1
        self.node.odom_yaw_cb(Float32(data=45.0))
        self.node.run()
        self.assertEqual(self.node.heading_source, 'encoder')
        self.assertFalse(self.node.health_blocked)

    def test_no_valid_heading_stops(self):
        self.healthy()
        self.diag('imu', 2)
        self.node.run()
        self.assertEqual(self.node.state, 'fault')
        self.assertEqual(self.node.health_reason, 'heading_unavailable')
        self.assertEqual(self.node.cmd_vel_msg.linear.x, 0)

    def test_healthy_diagnostic_does_not_hide_stale_data(self):
        self.healthy()
        self.node.last_cam_time = self.now - 1
        self.node.run()
        self.assertEqual(self.node.health_reason, 'detection_timeout')
        self.assertEqual(self.node.detect_streak, 0)

    def test_invalid_detection_is_not_counted_or_searched(self):
        self.healthy()
        self.frame(float('nan'))
        self.node.run()
        self.assertEqual(self.node.detect_streak, 0)
        self.assertEqual(self.node.health_reason, 'camera_invalid')
        self.assertEqual(self.node.state, 'fault')
        self.frame(1.0, ex=float('inf'))
        self.assertEqual(self.node.detect_streak, 0)

    def test_detection_gap_restarts_streak_before_timer(self):
        self.healthy()
        self.now += .6
        self.frame()
        self.assertEqual(self.node.detect_streak, 1)

    def test_invalid_joint_does_not_publish_old_goal(self):
        self.healthy()
        self.joints(float('nan'), 0)
        self.node.arm_cmd_pub.reset_mock()
        self.node.run()
        self.assertEqual(self.node.health_reason, 'arm_invalid')
        self.assertEqual(self.node.state, 'fault')
        self.assert_zero_arm_cmd()   # 이전 목표(30, 20)가 아니라 [0, 0]

    def test_invalid_imu_falls_back_immediately(self):
        self.healthy()
        msg = Imu()
        msg.orientation.w = 0.0  # Quaternion 기본 w=1이므로 명시적으로 무효화
        self.node.imu_cb(msg)
        self.node.odom_yaw_cb(Float32(data=20.0))
        self.node.run()
        self.assertEqual(self.node.heading_source, 'encoder')

    def test_disabled_diagnostics_keep_actual_input_checks(self):
        self.node.health_monitor = None
        self.node.health_enabled = False
        self.imu()
        self.joints()
        for _ in range(3):
            self.frame()
        self.node.run()
        self.assertFalse(self.node.health_blocked)
        self.assertEqual(self.node.health_reason, 'health_disabled')
        self.node.last_joint_time = self.now - 1
        self.node.run()
        self.assertEqual(self.node.health_reason, 'arm_invalid')

    def test_normal_miss_search_and_two_turns_still_work(self):
        self.healthy()
        self.frame(0.0)
        self.node.run()
        self.assertEqual(self.node.state, 'searching')
        self.assertFalse(self.node.health_blocked)
        for i in range(1, 9):
            self.imu(i * 90)
            self.node.run()
        self.assertEqual(self.node.state, 'lost')
        self.assertEqual(self.node.cmd_vel_msg.angular.z, 0)


if __name__ == '__main__':
    unittest.main()
