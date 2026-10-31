"""Run with ROS sourced; no robot topics/commands are used by these tests."""
import contextlib
import io
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from unittest.mock import Mock

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'rover_web/tools'), str(ROOT/'frob_odometry')]
import rover_motion_executor as motion
from frob_odometry.imu_bias_filter import GyroBiasEstimator, ImuBiasFilter, scale_yaw_measurement


class TurnPlant(motion.MotionController):
    """Exercise production control/settling against inertial motor dynamics."""
    def __init__(self, *, gain=1.0, latency=.08, acceleration=.7, start=0., stall=False, friction=0.):
        self.defaults = motion.MotionDefaults()
        self.odom_topic = '/odometry/filtered'
        self.now = 10.
        self.yaw = start
        self.velocity = 0.
        self.gain, self.latency, self.acceleration = gain, latency, acceleration
        self.stall = stall
        self.friction = friction
        self.pending = []
        self.target = 0.
        self.commands = []
        self.publisher = SimpleNamespace(publish=self.publish)

    def publish(self, command):
        self.pending.append((self.now+self.latency, command.angular.z))
        self.commands.append((self.now, command.angular.z))

    def sleep(self, dt):
        count = max(1, math.ceil(dt/.005))
        dt /= count
        for _ in range(count):
            self.now += dt
            while self.pending and self.pending[0][0] <= self.now:
                _, self.target = self.pending.pop(0)
            desired = self.gain*self.target if not self.stall else 0.
            if abs(desired) < self.friction:
                desired = 0.
            self.velocity += motion.clamp(desired-self.velocity,
                                          -self.acceleration*dt, self.acceleration*dt)
            self.yaw += self.velocity*dt

    def _spin_feedback(self): pass
    def _check_odom_freshness(self): pass
    def current_pose(self): return motion.Pose2D(0., 0., self.yaw)
    def current_angular_velocity(self): return self.velocity


class TurnTests(unittest.TestCase):
    def test_straight_course_control_corrects_motor_startup_yaw(self):
        maximum = .32
        # A positive yaw/rate means the chassis started turning left; the
        # controller must command a right correction immediately.
        correction = motion.straight_angular_command(
            steering_error=0., heading_error=math.radians(-3.),
            yaw_rate=.12, maximum=maximum)
        self.assertLess(correction, 0.)
        self.assertLessEqual(abs(correction), maximum)

        # Mirror symmetry is important because either motor may start first.
        mirrored = motion.straight_angular_command(
            steering_error=0., heading_error=math.radians(3.),
            yaw_rate=-.12, maximum=maximum)
        self.assertAlmostEqual(mirrored, -correction)

    def run_turn(self, degrees, **kwargs):
        plant = TurnPlant(**kwargs)
        start = plant.yaw
        with patch.object(motion.time, 'monotonic', lambda: plant.now), \
             patch.object(motion.time, 'sleep', plant.sleep), \
             patch.object(motion.rclpy, 'ok', lambda: True), \
             contextlib.redirect_stdout(io.StringIO()):
            plant.turn_relative(degrees, angular_speed=.45)
        self.assertLessEqual(abs(math.degrees(plant.yaw-start)-degrees), 1.)
        self.assertLessEqual(abs(plant.velocity), plant.defaults.turn_settle_rate_radps)
        self.assertEqual(plant.commands[-1][1], 0.)
        self.assertLess(plant.now, 70.)
        return plant

    def test_left_right_and_heading_wrap(self):
        for degrees in (90., -90., 180., -180., 6., -6.):
            for gain in (.8, 1., 1.15):
                with self.subTest(degrees=degrees, gain=gain):
                    self.run_turn(degrees, gain=gain, start=math.radians(179))

    def test_slow_braking_and_transport_delay(self):
        for degrees in (90., -90.):
            self.run_turn(degrees, latency=.15, acceleration=.35)

    def test_motor_deadband_near_target(self):
        for degrees in (90., -90.):
            self.run_turn(degrees, friction=.08)

    def test_stall_does_not_report_success(self):
        with self.assertRaisesRegex(RuntimeError, 'не уменьшается'):
            self.run_turn(90., stall=True)

    def test_gyro_bias_freezes_while_moving(self):
        estimator = GyroBiasEstimator(samples=300)
        for _ in range(299): estimator.update(.012, True)
        self.assertFalse(estimator.ready)
        estimator.update(.012, False)
        self.assertEqual(estimator.count, 0)
        for _ in range(300): estimator.update(.012, True)
        self.assertTrue(estimator.ready)
        for _ in range(200): estimator.update(.312, False)
        self.assertAlmostEqual(estimator.bias, .012)
        self.assertAlmostEqual(estimator.update(.013, False), .001)
        self.assertEqual(estimator.update(.012, True), 0.)

    def test_fusion_does_not_reinject_encoder_pose(self):
        config = yaml.safe_load((ROOT/'frob_odometry/config/ekf.yaml').read_text())['ekf_filter_node']['ros__parameters']
        self.assertEqual([i for i,v in enumerate(config['odom0_config']) if v], [6,7,11])
        self.assertEqual([i for i,v in enumerate(config['imu0_config']) if v], [11])
        self.assertNotIn('odom0_twist_rejection_threshold',config)
        self.assertNotIn('imu0_twist_rejection_threshold',config)
        self.assertEqual(motion.create_parser().parse_args(['turn','90']).odom_topic, '/odometry/filtered')

    def test_calibrated_gyro_scales_both_directions_and_variance(self):
        config=yaml.safe_load((ROOT/'frob_odometry/config/odometry_params.yaml').read_text())
        gain=config['imu_bias_filter']['ros__parameters']['yaw_rate_scale']
        self.assertAlmostEqual(gain,1.0289888275)
        for sign in (-1.,1.):
            rate,variance=scale_yaw_measurement(sign*.5,.0004,gain)
            self.assertAlmostEqual(rate,sign*.5*gain)
            self.assertAlmostEqual(variance,.0004*gain**2)
        for invalid in (-1.,0.,float('nan'),float('inf'),2.):
            with self.assertRaises(ValueError):scale_yaw_measurement(.5,.0004,invalid)

    def test_uncalibrated_imu_is_not_used_to_initialize_ekf(self):
        motion.rclpy.init()
        node=ImuBiasFilter()
        try:
            node.pub=Mock()
            node.last_motion=motion.time.monotonic()-2.
            node.last_odom=motion.time.monotonic()
            node.last_publish=0.
            for index in range(299):
                message=motion.Imu()
                message.header.stamp.sec=1
                message.header.stamp.nanosec=index+1
                message.angular_velocity.z=.01
                node.on_imu(message)
            node.pub.publish.assert_not_called()
            message.header.stamp.nanosec=300
            node.on_imu(message)
            node.pub.publish.assert_called_once()
            self.assertTrue(node.estimator.ready)
            self.assertEqual(node.pub.publish.call_args.args[0].angular_velocity.z, 0.)
        finally:
            node.destroy_node()
            motion.rclpy.shutdown()

    def test_missing_uncalibrated_or_stale_sensor_is_not_success(self):
        state = SimpleNamespace(latest_message=object(), defaults=motion.MotionDefaults(),
            latest_odom_monotonic=10., latest_imu_monotonic=10., latest_wheel_monotonic=10.,
            require_fused_feedback=True, imu_calibrated=True)
        with patch.object(motion.time, 'monotonic', lambda:10.1):
            self.assertIsNone(motion.MotionController._feedback_problem(state))
            state.imu_calibrated=False
            self.assertIn('откалибрована', motion.MotionController._feedback_problem(state))
            state.imu_calibrated=True
            for attr, word in [('latest_imu_monotonic','IMU'), ('latest_wheel_monotonic','энкодеров'), ('latest_odom_monotonic','одометрии')]:
                setattr(state, attr, 8.)
                self.assertIn(word, motion.MotionController._feedback_problem(state))
                setattr(state, attr, 10.)

    def test_delayed_and_repeated_packets_cannot_refresh_feedback(self):
        state=SimpleNamespace(defaults=motion.MotionDefaults(), _feedback_stamps={},
                              get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10_000_000_000)))
        message=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=9,nanosec=0)))
        new=motion.MotionController._new_feedback
        self.assertFalse(new(state, 'imu', message))
        message.header.stamp.nanosec=900_000_000
        self.assertTrue(new(state, 'imu', message))
        self.assertFalse(new(state, 'imu', message))


if __name__ == '__main__': unittest.main()
