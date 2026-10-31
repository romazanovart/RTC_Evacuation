"""Remove the MPU6050's temperature-dependent Z gyro bias while stationary."""

import copy
import math
import time

from geometry_msgs.msg import Vector3Stamped, Twist
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


def scale_yaw_measurement(rate, variance, scale):
    """Apply measured sensitivity AFTER removing bias in raw rad/s units."""
    if not math.isfinite(scale) or not .8 <= scale <= 1.2:
        raise ValueError('yaw_rate_scale must be finite and within [0.8,1.2]')
    return rate*scale, variance*scale*scale


class GyroBiasEstimator:
    """Estimate bias only from consecutive samples known to be stationary."""

    def __init__(self, samples=100, alpha=0.002, deadband=0.003):
        self.samples = samples
        self.alpha = alpha
        self.deadband = deadband
        self.count = 0
        self.bias = 0.0
        self.ready = False

    def update(self, value, stationary):
        if stationary and math.isfinite(value):
            if not self.ready:
                self.count += 1
                self.bias += (value - self.bias) / self.count
                self.ready = self.count >= self.samples
            else:
                self.bias += self.alpha * (value - self.bias)
        elif not self.ready:
            self.count = 0
            self.bias = 0.0
        corrected = value - self.bias if self.ready else value
        # Do not erase the slow tail of a real turn from the integrated angle.
        if stationary and abs(corrected) < self.deadband:
            corrected = 0.0
        return corrected


class AccelProcessor:
    """Remove the stationary gravity/bias vector and reject chassis vibration."""

    def __init__(self, samples=200, alpha=0.002, cutoff_hz=3.0,
                 deadband=0.04, limit=1.5):
        self.samples = samples
        self.alpha = alpha
        self.cutoff_hz = cutoff_hz
        self.deadband = deadband
        self.limit = limit
        self.count = 0
        self.bias = [0.0, 0.0, 0.0]
        self.filtered = [0.0, 0.0, 0.0]
        self.ready = False

    def update(self, values, stationary, dt):
        values = tuple(float(value) for value in values)
        finite = all(math.isfinite(value) for value in values)
        if stationary and finite:
            if not self.ready:
                self.count += 1
                for index, value in enumerate(values):
                    self.bias[index] += (value-self.bias[index]) / self.count
                self.ready = self.count >= self.samples
            else:
                for index, value in enumerate(values):
                    self.bias[index] += self.alpha * (value-self.bias[index])
        if not self.ready or not finite:
            self.filtered = [0.0, 0.0, 0.0]
            return (0.0, 0.0, 0.0)

        corrected = [value-bias for value, bias in zip(values, self.bias)]
        if stationary:
            # Wheel zero-velocity information is a reliable constraint and
            # prevents vibration from being integrated while stopped.
            corrected = [0.0, 0.0, 0.0]
        dt = min(0.1, max(0.001, dt))
        smoothing = 1.0-math.exp(-2.0*math.pi*self.cutoff_hz*dt)
        result = []
        for index, value in enumerate(corrected):
            self.filtered[index] += smoothing*(value-self.filtered[index])
            bounded = max(-self.limit, min(self.limit, self.filtered[index]))
            result.append(0.0 if abs(bounded) < self.deadband else bounded)
        return tuple(result)


class ImuBiasFilter(Node):
    def __init__(self):
        super().__init__('imu_bias_filter')
        defaults = dict(
            input_topic='/imu/mpu6050',
            output_topic='/imu/corrected',
            calibration_samples=300,
            stationary_delay=1.0,
            stationary_linear_threshold=0.001,
            stationary_angular_threshold=0.005,
            adaptation_alpha=0.002,
            # The straight-drive bag showed about 0.0014 rad/s accumulated
            # vibration with short peaks above 0.003. Apply only when stopped.
            deadband=0.003,
            yaw_rate_variance=0.0004,
            yaw_rate_scale=1.0,
            accel_calibration_samples=200,
            accel_adaptation_alpha=0.002,
            accel_cutoff_hz=3.0,
            accel_deadband=0.04,
            accel_limit=1.5,
            accel_x_variance=0.04,
            accel_y_variance=0.16,
            output_rate=50.0,
        )
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.yaw_scale=float(self.get_parameter('yaw_rate_scale').value)
        scale_yaw_measurement(0.,1.,self.yaw_scale)
        self.estimator = GyroBiasEstimator(
            samples=self.get_parameter('calibration_samples').value,
            alpha=self.get_parameter('adaptation_alpha').value,
            deadband=self.get_parameter('deadband').value,
        )
        self.accel = AccelProcessor(
            samples=self.get_parameter('accel_calibration_samples').value,
            alpha=self.get_parameter('accel_adaptation_alpha').value,
            cutoff_hz=self.get_parameter('accel_cutoff_hz').value,
            deadband=self.get_parameter('accel_deadband').value,
            limit=self.get_parameter('accel_limit').value,
        )
        self.last_odom = 0.0
        self.last_motion = time.monotonic()
        self.last_imu = time.monotonic()
        self.last_publish = 0.0
        self.last_report = 0.0
        self.last_input_stamp = -1
        self.calibration_reported = False
        self.pub = self.create_publisher(
            Imu, self.get_parameter('output_topic').value, qos_profile_sensor_data)
        self.bias_pub = self.create_publisher(Float64, '/imu/gyro_bias_z', 10)
        self.accel_bias_pub = self.create_publisher(
            Vector3Stamped, '/imu/accel_bias', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Twist, '/frob/motor_cmd_vel', self.on_command, 10)
        self.create_subscription(
            Imu, self.get_parameter('input_topic').value, self.on_imu,
            qos_profile_sensor_data)

    def on_command(self, msg):
        if abs(msg.linear.x) > 0.001 or abs(msg.angular.z) > 0.001:
            self.last_motion = time.monotonic()

    def on_odom(self, msg):
        self.last_odom = time.monotonic()
        twist = msg.twist.twist
        if (abs(twist.linear.x) > self.get_parameter('stationary_linear_threshold').value or
                abs(twist.angular.z) > self.get_parameter('stationary_angular_threshold').value):
            self.last_motion = time.monotonic()

    def on_imu(self, msg):
        now = time.monotonic()
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if stamp <= self.last_input_stamp:
            return
        self.last_input_stamp = stamp
        if not all(math.isfinite(v) for v in (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)):
            self.estimator.update(float('nan'), False)
            return
        stationary = (now - self.last_odom < 0.3 and
                      now - self.last_motion >= self.get_parameter('stationary_delay').value and
                      abs(msg.angular_velocity.z - self.estimator.bias) < 0.04)
        corrected = self.estimator.update(msg.angular_velocity.z, stationary)
        if self.estimator.ready and not self.calibration_reported:
            self.get_logger().info(
                f'Gyro Z stationary calibration ready: bias={self.estimator.bias:.7f} rad/s, scale={self.yaw_scale:.9f}')
            self.calibration_reported = True
        now = time.monotonic()
        acceleration = self.accel.update(
            (msg.linear_acceleration.x, msg.linear_acceleration.y,
             msg.linear_acceleration.z), stationary, now-self.last_imu)
        self.last_imu = now
        # An uncalibrated IMU can be the EKF's first message at boot. Even a
        # huge covariance then initializes its angular-rate uncertainty and
        # pollutes yaw covariance during integration. Let wheels initialize
        # the filter; publish gyro measurements only after stationary bias
        # calibration is complete.
        if not self.estimator.ready:
            return
        period = 1.0 / max(1.0, self.get_parameter('output_rate').value)
        if now - self.last_publish < period:
            return
        self.last_publish = now
        result = copy.deepcopy(msg)
        result.angular_velocity.z, yaw_variance = scale_yaw_measurement(
            corrected,self.get_parameter('yaw_rate_variance').value,self.yaw_scale)
        # X/Y are deliberately not fused. A realistic Z covariance prevents
        # robot_localization from treating this low-cost gyro as perfect.
        result.angular_velocity_covariance = [1.0e6, 0.0, 0.0,
                                               0.0, 1.0e6, 0.0,
                                               0.0, 0.0,
                                               yaw_variance]
        result.linear_acceleration.x = acceleration[0]
        result.linear_acceleration.y = acceleration[1]
        result.linear_acceleration.z = acceleration[2]
        if self.accel.ready:
            x_variance = self.get_parameter('accel_x_variance').value
            y_variance = self.get_parameter('accel_y_variance').value
        else:
            x_variance = y_variance = 1.0e6
        result.linear_acceleration_covariance = [x_variance, 0.0, 0.0,
                                                 0.0, y_variance, 0.0,
                                                 0.0, 0.0, 1.0e6]
        self.pub.publish(result)
        if self.estimator.ready and now - self.last_report >= 1.0:
            self.bias_pub.publish(Float64(data=self.estimator.bias))
            if self.accel.ready:
                bias = Vector3Stamped()
                bias.header = msg.header
                bias.vector.x, bias.vector.y, bias.vector.z = self.accel.bias
                self.accel_bias_pub.publish(bias)
            self.last_report = now


def main(args=None):
    rclpy.init(args=args)
    node = ImuBiasFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
