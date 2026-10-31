#!/usr/bin/env python3
import argparse
import json
import math
import statistics
import time

import rclpy
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


def yaw(message):
    q = message.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def describe(values):
    if not values:
        return None
    return {
        'count': len(values),
        'mean': statistics.fmean(values),
        'stdev': statistics.pstdev(values) if len(values) > 1 else 0.0,
        'min': min(values),
        'max': max(values),
    }


class Audit(Node):
    def __init__(self):
        super().__init__('odometry_audit')
        self.started = time.monotonic()
        self.arrivals = {}
        self.encoder = []
        self.odom = []
        self.filtered = []
        self.raw_gyro = []
        self.corrected_gyro = []
        self.bias = []
        self.create_subscription(Vector3Stamped, '/wheel/encoder_delta', self.on_encoder, 50)
        self.create_subscription(Odometry, '/odom', lambda msg: self.on_odom('odom', msg), 50)
        self.create_subscription(Odometry, '/odometry/filtered', lambda msg: self.on_odom('filtered', msg), 50)
        self.create_subscription(Imu, '/imu/mpu6050', lambda msg: self.on_imu('raw_imu', msg), qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu/corrected', lambda msg: self.on_imu('corrected_imu', msg), qos_profile_sensor_data)
        self.create_subscription(Float64, '/imu/gyro_bias_z', self.on_bias, 10)

    def touch(self, name):
        self.arrivals.setdefault(name, []).append(time.monotonic())

    def on_encoder(self, msg):
        self.touch('encoder')
        self.encoder.append((float(msg.vector.x), float(msg.vector.y), float(msg.vector.z)))

    def on_odom(self, name, msg):
        self.touch(name)
        entry = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y), yaw(msg),
                 float(msg.twist.twist.linear.x), float(msg.twist.twist.angular.z),
                 float(msg.pose.covariance[0]), float(msg.pose.covariance[35]))
        (self.odom if name == 'odom' else self.filtered).append(entry)

    def on_imu(self, name, msg):
        self.touch(name)
        target = self.raw_gyro if name == 'raw_imu' else self.corrected_gyro
        target.append(float(msg.angular_velocity.z))

    def on_bias(self, msg):
        self.touch('bias')
        self.bias.append(float(msg.data))

    def rate(self, name):
        values = self.arrivals.get(name, [])
        if len(values) < 2:
            return {'count': len(values), 'hz': 0.0, 'max_gap_s': None}
        gaps = [b - a for a, b in zip(values, values[1:])]
        return {'count': len(values), 'hz': (len(values) - 1) / (values[-1] - values[0]), 'max_gap_s': max(gaps)}

    def pose_report(self, values):
        if not values:
            return None
        first, last = values[0], values[-1]
        return {
            'start': {'x': first[0], 'y': first[1], 'yaw': first[2]},
            'end': {'x': last[0], 'y': last[1], 'yaw': last[2]},
            'delta_m': math.hypot(last[0] - first[0], last[1] - first[1]),
            'delta_yaw_deg': math.degrees(math.atan2(math.sin(last[2] - first[2]), math.cos(last[2] - first[2]))),
            'linear_velocity': describe([row[3] for row in values]),
            'angular_velocity': describe([row[4] for row in values]),
            'covariance_x': describe([row[5] for row in values]),
            'covariance_yaw': describe([row[6] for row in values]),
        }

    def report(self, duration):
        rates = {name: self.rate(name) for name in ('encoder', 'odom', 'filtered', 'raw_imu', 'corrected_imu', 'bias')}
        warnings = []
        minimum_rates = {'encoder': 7.0, 'odom': 7.0, 'filtered': 15.0, 'raw_imu': 20.0, 'corrected_imu': 20.0}
        for name, minimum in minimum_rates.items():
            if rates[name]['hz'] < minimum:
                warnings.append(f'{name}: {rates[name]["hz"]:.1f} Hz, expected at least {minimum:.1f} Hz')
        if self.encoder:
            invalid_dt = sum(1 for left, right, dt in self.encoder if not 0.001 < dt < 1.0 or not all(map(math.isfinite, (left, right, dt))))
            if invalid_dt:
                warnings.append(f'encoder: {invalid_dt} invalid samples')
        corrected = describe(self.corrected_gyro)
        if corrected and abs(corrected['mean']) > 0.01:
            warnings.append(f'corrected gyro bias remains {corrected["mean"]:.5f} rad/s')
        return {
            'duration_s': duration,
            'rates': rates,
            'encoder': {
                'left_ticks': describe([row[0] for row in self.encoder]),
                'right_ticks': describe([row[1] for row in self.encoder]),
                'interval_s': describe([row[2] for row in self.encoder]),
            },
            'wheel_odometry': self.pose_report(self.odom),
            'filtered_odometry': self.pose_report(self.filtered),
            'imu': {'raw_gyro_z': describe(self.raw_gyro), 'corrected_gyro_z': corrected, 'estimated_bias_z': describe(self.bias)},
            'ok': not warnings,
            'warnings': warnings,
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=15.0)
    args = parser.parse_args()
    rclpy.init()
    node = Audit()
    deadline = time.monotonic() + max(2.0, args.duration)
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    print(json.dumps(node.report(args.duration), ensure_ascii=False, indent=2))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
