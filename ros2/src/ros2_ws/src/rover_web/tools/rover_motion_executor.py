#!/usr/bin/env python3
"""Closed-loop motion primitives for the FROB differential robot using ROS 2 odometry.

Commands are completed from fused encoder/IMU feedback, not by assuming that a fixed
number of seconds corresponds to a fixed distance or angle.
"""

from __future__ import annotations

import argparse
import copy
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
import yaml


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_odometry(message: Odometry) -> float:
    q = message.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def bounded_turn_time_limit(
    degrees: float,
    angular_speed: float,
    configured_limit: float,
) -> float:
    """Return a short physical deadline for a closed-loop turn."""
    expected = abs(math.radians(degrees)) / max(abs(angular_speed), 0.05)
    return min(float(configured_limit), max(15.0, expected * 3.5 + 3.0))


def straight_angular_command(
    steering_error: float,
    heading_error: float,
    yaw_rate: float,
    maximum: float,
    *,
    path_gain: float = 0.80,
    heading_gain: float = 2.20,
    yaw_damping: float = 0.20,
) -> float:
    """Return bounded course correction for a translating robot."""
    return clamp(
        path_gain * steering_error
        + heading_gain * heading_error
        - yaw_damping * yaw_rate,
        -maximum,
        maximum,
    )


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass
class MotionDefaults:
    linear_speed: float = 0.18
    approach_speed: float = 0.10
    position_tolerance: float = 0.07
    angular_speed: float = 0.32
    minimum_angular_speed: float = 0.10
    angle_tolerance_deg: float = 1.0
    turn_braking_decel_radps2: float = 0.60
    turn_acceleration_radps2: float = 0.45
    turn_brake_margin_deg: float = 0.15
    turn_correction_speed: float = 0.40
    turn_settle_rate_radps: float = 0.015
    turn_settle_time: float = 0.50
    turn_max_corrections: int = 3

    # Linear stopping and settled-position verification.
    move_braking_decel_mps2: float = 0.22
    move_acceleration_mps2: float = 0.18
    move_brake_margin_m: float = 0.010
    move_correction_speed: float = 0.10
    move_correction_approach_speed: float = 0.10
    move_settle_speed_mps: float = 0.015
    move_settle_rate_radps: float = 0.03
    move_settle_time: float = 0.50
    move_max_corrections: int = 1
    # Straight-line course controller.  The IMU heading term reacts as soon
    # as unequal motor startup begins rotating the chassis, while the path
    # term removes the small lateral displacement that remains.
    move_path_gain: float = 0.80
    move_heading_gain: float = 2.20
    move_yaw_damping: float = 0.20

    odom_timeout: float = 0.50
    # Sensors publish at about 20 Hz.  A 25 Hz controller is responsive while
    # avoiding the full-core busy polling caused by the old 50 Hz loop.
    command_rate_hz: float = 25.0
    maximum_step_time: float = 60.0


class MotionController(Node):
    def __init__(
        self,
        odom_topic: str,
        cmd_vel_topic: str,
        defaults: MotionDefaults,
    ) -> None:
        super().__init__("rover_motion_executor")

        self.defaults = defaults
        self.odom_topic = odom_topic
        self.require_fused_feedback = odom_topic == '/odometry/filtered'
        self.latest_imu_monotonic = 0.0
        self.latest_wheel_monotonic = 0.0
        self.imu_calibrated = False
        self._feedback_stamps = {}
        self._wheel_message = None
        self._wheel_alignment = None
        self._wheel_fallback = False
        self._fallback_previous = None
        self._imu_integrated_yaw = 0.0
        self._imu_last_stamp = None
        self._imu_last_rate = 0.0
        self._imu_yaw_offset = None
        if self.require_fused_feedback:
            self.imu_subscription = self.create_subscription(
                Imu, '/imu/corrected', self._imu_callback, qos_profile_sensor_data)
            self.wheel_subscription = self.create_subscription(
                Odometry, '/odom', self._wheel_callback, 1)
        self.publisher = self.create_publisher(
            Twist,
            cmd_vel_topic,
            10,
        )
        self.subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self._odom_callback,
            1,
        )

        self.latest_message: Odometry | None = None
        self.latest_odom_monotonic = 0.0

        self._last_raw_yaw: float | None = None
        self._unwrapped_yaw = 0.0
        self.origin: Pose2D | None = None

    def _odom_callback(self, message: Odometry) -> None:
        if self._wheel_fallback or not self._new_feedback('fused', message):
            return
        raw_yaw = yaw_from_odometry(message)
        if not all(math.isfinite(v) for v in (
            raw_yaw, message.pose.pose.position.x, message.pose.pose.position.y,
            message.twist.twist.angular.z, message.twist.twist.linear.x,
        )):
            return

        if self._last_raw_yaw is None:
            self._unwrapped_yaw = raw_yaw
        else:
            delta = normalize_angle(raw_yaw - self._last_raw_yaw)
            self._unwrapped_yaw += delta

        self._last_raw_yaw = raw_yaw
        self.latest_message = message
        if self._imu_last_stamp is not None:
            self._imu_yaw_offset = self._unwrapped_yaw - self._imu_integrated_yaw
        self.latest_odom_monotonic = time.monotonic()
        wheel = self._wheel_message
        if wheel is not None and self.latest_odom_monotonic - self.latest_wheel_monotonic < .15:
            angle = normalize_angle(raw_yaw - yaw_from_odometry(wheel))
            wx, wy = wheel.pose.pose.position.x, wheel.pose.pose.position.y
            self._wheel_alignment = (angle,
                message.pose.pose.position.x - math.cos(angle)*wx + math.sin(angle)*wy,
                message.pose.pose.position.y - math.sin(angle)*wx - math.cos(angle)*wy)

    def _new_feedback(self, source, message):
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        if not -0.1 <= age <= self.defaults.odom_timeout:
            return False
        if stamp <= self._feedback_stamps.get(source, -1):
            return False
        self._feedback_stamps[source] = stamp
        return True

    def _imu_callback(self, message):
        if not self._new_feedback('imu', message):
            return
        self.latest_imu_monotonic = time.monotonic()
        variance = message.angular_velocity_covariance[8]
        self.imu_calibrated = (
            math.isfinite(message.angular_velocity.z) and 0.0 < variance < 1.0)
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        rate = float(message.angular_velocity.z)
        if self.imu_calibrated:
            if self._imu_last_stamp is not None:
                dt = stamp - self._imu_last_stamp
                if 0 < dt <= self.defaults.odom_timeout:
                    self._imu_integrated_yaw += .5 * (self._imu_last_rate + rate) * dt
                elif self._wheel_fallback:
                    self.imu_calibrated = False
            self._imu_last_stamp = stamp
            self._imu_last_rate = rate

    def _wheel_callback(self, message):
        values = (message.pose.pose.position.x, message.pose.pose.position.y,
                  yaw_from_odometry(message), message.twist.twist.angular.z,
                  message.twist.twist.linear.x, message.twist.twist.linear.y)
        if self._new_feedback('wheel', message) and all(math.isfinite(v) for v in values):
            self.latest_wheel_monotonic = time.monotonic()
            self._wheel_message = message
            if self._wheel_fallback:
                self._apply_wheel_feedback()

    def _apply_wheel_feedback(self):
        message = copy.deepcopy(self._wheel_message)
        angle, tx, ty = self._wheel_alignment or (0., 0., 0.)
        x, y = message.pose.pose.position.x, message.pose.pose.position.y
        message.pose.pose.position.x = math.cos(angle)*x - math.sin(angle)*y + tx
        message.pose.pose.position.y = math.sin(angle)*x + math.cos(angle)*y + ty
        raw = normalize_angle(yaw_from_odometry(message) + angle)
        if self._imu_yaw_offset is None:
            self._imu_yaw_offset = raw - self._imu_integrated_yaw
        raw = normalize_angle(self._imu_integrated_yaw + self._imu_yaw_offset)
        message.twist.twist.angular.z = self._imu_last_rate
        wheel_yaw = yaw_from_odometry(self._wheel_message)
        previous = self._fallback_previous
        if previous is not None:
            px, py, pw, old_yaw, out_x, out_y = previous
            wheel_mid = pw + .5 * normalize_angle(wheel_yaw - pw)
            distance = (x-px)*math.cos(wheel_mid) + (y-py)*math.sin(wheel_mid)
            imu_mid = old_yaw + .5 * normalize_angle(raw - old_yaw)
            message.pose.pose.position.x = out_x + distance * math.cos(imu_mid)
            message.pose.pose.position.y = out_y + distance * math.sin(imu_mid)
        self._fallback_previous = (x, y, wheel_yaw, raw,
            message.pose.pose.position.x, message.pose.pose.position.y)
        if self._last_raw_yaw is None:
            self._unwrapped_yaw = raw
        else:
            self._unwrapped_yaw += normalize_angle(raw - self._last_raw_yaw)
        self._last_raw_yaw = raw
        self.latest_message = message
        self.latest_odom_monotonic = self.latest_wheel_monotonic

    def _feedback_problem(self):
        now = time.monotonic()
        if self.latest_message is None or now - self.latest_odom_monotonic > self.defaults.odom_timeout:
            available = (self.require_fused_feedback and self._wheel_message is not None
                         and now - self.latest_wheel_monotonic <= self.defaults.odom_timeout
                         and now - self.latest_imu_monotonic <= self.defaults.odom_timeout
                         and self.imu_calibrated
                         and (self.latest_message is None or self._wheel_alignment is not None))
            if not available:
                return 'Нет свежей одометрии или резервных датчиков'
            if not self._wheel_fallback:
                print('FEEDBACK_FALLBACK: Основная одометрия недоступна. Продолжаю: расстояние по энкодерам, поворот по IMU.', flush=True)
                self._wheel_fallback = True
            self._apply_wheel_feedback()
        if self.require_fused_feedback:
            if now - self.latest_wheel_monotonic > self.defaults.odom_timeout:
                return 'Нет свежих энкодеров'
            if now - self.latest_imu_monotonic > self.defaults.odom_timeout:
                return 'Нет свежей IMU'
            if not self.imu_calibrated:
                return 'IMU ещё не откалибрована на стоящем роботе'
        return None

    def wait_until_ready(self, timeout: float = 10.0) -> None:
        print("Ожидание одометрии...")
        deadline = time.monotonic() + timeout

        while self._feedback_problem() is not None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() >= deadline:
                raise RuntimeError(self._feedback_problem() or 'Истекло ожидание датчиков')

        print("Ожидание подписчика /cmd_vel...")
        deadline = time.monotonic() + timeout

        while self.publisher.get_subscription_count() < 1:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    "Подписчик команд скорости не найден"
                )

        self.origin = self.current_pose()
        print(
            "Начальная точка: "
            f"x={self.origin.x:+.3f} м, "
            f"y={self.origin.y:+.3f} м, "
            f"yaw={math.degrees(self.origin.yaw):+.2f}°"
        )

    def current_pose(self) -> Pose2D:
        if self.latest_message is None:
            raise RuntimeError("Одометрия ещё не получена")

        position = self.latest_message.pose.pose.position
        return Pose2D(
            x=position.x,
            y=position.y,
            yaw=self._unwrapped_yaw,
        )

    def current_angular_velocity(self) -> float:
        if self.latest_message is None:
            raise RuntimeError("Одометрия ещё не получена")
        return float(
            self.latest_message.twist.twist.angular.z
        )

    def current_body_velocity(self) -> tuple[float, float]:
        if self.latest_message is None:
            raise RuntimeError("Одометрия ещё не получена")
        velocity = self.latest_message.twist.twist.linear
        return float(velocity.x), float(velocity.y)

    def current_linear_speed(self) -> float:
        velocity_x, velocity_y = self.current_body_velocity()
        return math.hypot(velocity_x, velocity_y)

    def _check_odom_freshness(self) -> None:
        problem = self._feedback_problem()
        if problem:
            raise RuntimeError(problem)

    def _spin_feedback(self):
        # Drain a bounded batch without busy-polling a full CPU core.
        for _ in range(3):
            rclpy.spin_once(self, timeout_sec=0.0)

    def publish_stop(self, duration: float = 0.8) -> None:
        stop = Twist()
        period = 1.0 / self.defaults.command_rate_hz
        deadline = time.monotonic() + duration

        while time.monotonic() < deadline and rclpy.ok():
            self.publisher.publish(stop)
            self._spin_feedback()
            time.sleep(period)

    def _position_error(
        self,
        target: Pose2D,
    ) -> tuple[float, float, float]:
        pose = self.current_pose()
        error_x = target.x - pose.x
        error_y = target.y - pose.y
        return error_x, error_y, math.hypot(error_x, error_y)

    def _settle_after_move(
        self,
        target: Pose2D,
        *,
        timeout: float = 5.0,
    ) -> tuple[float, float, float, float]:
        """Command zero and wait until odometry reports a real stop."""
        period = 1.0 / self.defaults.command_rate_hz
        deadline = time.monotonic() + timeout
        stable_since: float | None = None
        last_report = 0.0

        while rclpy.ok():
            self.publisher.publish(Twist())
            self._spin_feedback()
            self._check_odom_freshness()

            pose = self.current_pose()
            _, _, distance = self._position_error(target)
            yaw_error = target.yaw - pose.yaw
            linear_speed = self.current_linear_speed()
            angular_rate = self.current_angular_velocity()
            now = time.monotonic()

            stopped = (
                linear_speed
                <= self.defaults.move_settle_speed_mps
                and abs(angular_rate)
                <= self.defaults.move_settle_rate_radps
            )

            if stopped:
                if stable_since is None:
                    stable_since = now
                elif (
                    now - stable_since
                    >= self.defaults.move_settle_time
                ):
                    return (
                        distance,
                        yaw_error,
                        linear_speed,
                        angular_rate,
                    )
            else:
                stable_since = None

            if now - last_report >= 0.20:
                print(
                    f"\rТорможение перемещения: "
                    f"ошибка {distance:.3f} м, "
                    f"курс {math.degrees(yaw_error):+.1f}°, "
                    f"v={linear_speed:.3f} м/с, "
                    f"ω={angular_rate:+.3f}",
                    end="",
                    flush=True,
                )
                last_report = now

            if now >= deadline:
                return (
                    distance,
                    yaw_error,
                    linear_speed,
                    angular_rate,
                )

            time.sleep(period)

        raise RuntimeError("ROS был остановлен")

    def move_to_pose(
        self,
        target: Pose2D,
        *,
        linear_speed: float | None = None,
        approach_speed: float | None = None,
        position_tolerance: float | None = None,
        maximum_time: float | None = None,
        label: str = "перемещение",
    ) -> None:
        requested_max_speed = (
            self.defaults.linear_speed
            if linear_speed is None
            else float(linear_speed)
        )
        requested_min_speed = (
            self.defaults.approach_speed
            if approach_speed is None
            else float(approach_speed)
        )
        tolerance = (
            self.defaults.position_tolerance
            if position_tolerance is None
            else float(position_tolerance)
        )
        heading_tolerance_deg = max(
            self.defaults.angle_tolerance_deg,
            5.0,
        )
        heading_tolerance_rad = math.radians(heading_tolerance_deg)
        configured_time_limit = min(60.0, (
            self.defaults.maximum_step_time
            if maximum_time is None
            else float(maximum_time)
        ))
        time_limit = configured_time_limit

        if not 0.0 < requested_min_speed <= requested_max_speed:
            raise ValueError(
                "approach_speed должен быть > 0 и <= linear_speed"
            )
        if tolerance <= 0.0:
            raise ValueError("position_tolerance должен быть > 0")
        if not all(math.isfinite(value) and value > 0 for value in (
            self.defaults.move_braking_decel_mps2, self.defaults.move_acceleration_mps2
        )):
            raise ValueError(
                "move_braking_decel_mps2 должен быть > 0"
            )

        total_start_time = time.monotonic()
        period = 1.0 / self.defaults.command_rate_hz

        print(
            f"{label}: цель x={target.x:+.3f}, "
            f"y={target.y:+.3f}, "
            f"yaw={math.degrees(target.yaw):+.2f}°"
        )

        for correction_index in range(
            self.defaults.move_max_corrections + 1
        ):
            pose = self.current_pose()
            _, _, initial_distance = self._position_error(target)

            if correction_index == 0:
                pass_max_speed = requested_max_speed
                pass_min_speed = requested_min_speed
                phase_name = label
            else:
                pass_max_speed = min(
                    requested_max_speed,
                    self.defaults.move_correction_speed,
                )
                pass_min_speed = min(
                    pass_max_speed,
                    self.defaults.move_correction_approach_speed,
                )
                phase_name = (
                    f"{label}: коррекция позиции "
                    f"{correction_index}/"
                    f"{self.defaults.move_max_corrections}"
                )
                print(
                    f"\n{phase_name}, остаток "
                    f"{initial_distance:.3f} м"
                )

            best_distance = initial_distance
            last_improvement_time = time.monotonic()
            last_report = 0.0
            best_alignment_error = math.pi
            ramp_speed = 0.0
            ramp_time = time.monotonic()

            while rclpy.ok():
                self._spin_feedback()
                self._check_odom_freshness()

                pose = self.current_pose()
                error_world_x = target.x - pose.x
                error_world_y = target.y - pose.y
                distance = math.hypot(
                    error_world_x,
                    error_world_y,
                )
                yaw_error = normalize_angle(target.yaw - pose.yaw)

                cosine = math.cos(pose.yaw)
                sine = math.sin(pose.yaw)
                error_body_x = (
                    cosine * error_world_x
                    + sine * error_world_y
                )
                error_body_y = (
                    -sine * error_world_x
                    + cosine * error_world_y
                )

                velocity_x, velocity_y = (
                    self.current_body_velocity()
                )
                linear_speed_now = math.hypot(
                    velocity_x,
                    velocity_y,
                )

                if distance > 1e-9:
                    direction_body_x = error_body_x / distance
                    direction_body_y = error_body_y / distance
                    speed_toward_target = (
                        velocity_x * direction_body_x
                        + velocity_y * direction_body_y
                    )
                else:
                    direction_body_x = 0.0
                    direction_body_y = 0.0
                    speed_toward_target = 0.0

                stopping_distance = (
                    max(0.0, speed_toward_target) ** 2
                    / (
                        2.0
                        * self.defaults.move_braking_decel_mps2
                    )
                )
                braking_threshold = (
                    stopping_distance
                    + self.defaults.move_brake_margin_m
                )

                # Stop before the target based on measured odometry
                # velocity, then assess the final settled pose.
                if (
                    distance <= tolerance
                    or (
                        speed_toward_target > 0.0
                        and distance <= braking_threshold
                        and linear_speed_now <= pass_min_speed
                    )
                ):
                    self.publisher.publish(Twist())
                    break

                speed_command = clamp(
                    1.0 * distance,
                    pass_min_speed,
                    pass_max_speed,
                )
                speed_command = min(speed_command, math.sqrt(
                    2.0 * self.defaults.move_braking_decel_mps2 * max(0.0, distance - tolerance)
                ))
                ramp_now = time.monotonic()
                dt = min(max(ramp_now - ramp_time, 0.0), 2.0 * period)
                ramp_time = ramp_now
                rate = (self.defaults.move_acceleration_mps2 if speed_command > ramp_speed
                        else self.defaults.move_braking_decel_mps2)
                ramp_speed += clamp(speed_command - ramp_speed, -rate * dt, rate * dt)
                speed_command = ramp_speed
                steering_error = math.atan2(error_body_y, error_body_x)
                reverse = abs(steering_error) > math.pi / 2
                if reverse:
                    steering_error = math.atan2(math.sin(steering_error - math.pi),
                                                math.cos(steering_error - math.pi))
                command_x = (-1 if reverse else 1) * speed_command * max(0.0, math.cos(steering_error))
                if abs(steering_error) > .45:
                    command_x = 0.0
                command_y = 0.0
                # Keep the requested route heading during translation.
                # Previously yaw_error was only reported in the log, so the
                # controller reacted late, after a visible lateral error.
                angular_command = straight_angular_command(
                    steering_error,
                    yaw_error,
                    self.current_angular_velocity(),
                    self.defaults.angular_speed,
                    path_gain=self.defaults.move_path_gain,
                    heading_gain=self.defaults.move_heading_gain,
                    yaw_damping=self.defaults.move_yaw_damping,
                )

                if abs(steering_error) > .45:
                    angular_command = math.copysign(
                        min(self.defaults.angular_speed, max(.24, abs(steering_error))),
                        steering_error)

                command = Twist()
                command.linear.x = command_x
                command.linear.y = command_y
                command.angular.z = angular_command
                self.publisher.publish(command)

                now = time.monotonic()

                if distance < best_distance - 0.003:
                    best_distance = distance
                    last_improvement_time = now

                if abs(steering_error) > .45:
                    if abs(steering_error) < best_alignment_error - math.radians(1):
                        best_alignment_error = abs(steering_error)
                        last_improvement_time = now
                else:
                    best_alignment_error = math.pi

                if now - last_improvement_time > 3.0:
                    raise RuntimeError(
                        f"{phase_name}: расстояние до цели "
                        "не уменьшается"
                    )

                if now - total_start_time > time_limit:
                    raise RuntimeError(
                        f"{label}: превышен лимит "
                        f"{time_limit:.1f} с"
                    )

                if now - last_report >= 0.20:
                    print(
                        f"\r{phase_name}: осталось "
                        f"{distance:.3f} м, "
                        f"курс "
                        f"{math.degrees(yaw_error):+.1f}°, "
                        f"v={linear_speed_now:.3f}, "
                        f"стоп≈{stopping_distance:.3f} м, "
                        f"cmd x={command_x:+.2f}, "
                        f"y={command_y:+.2f}",
                        end="",
                        flush=True,
                    )
                    last_report = now

                time.sleep(period)

            (
                settled_distance,
                settled_yaw_error,
                settled_speed,
                settled_rate,
            ) = self._settle_after_move(target)
            print(
                f"\nПосле полной остановки: "
                f"ошибка позиции {settled_distance:.3f} м, "
                f"курс "
                f"{math.degrees(settled_yaw_error):+.2f}°, "
                f"v={settled_speed:.3f} м/с, "
                f"ω={settled_rate:+.3f}"
            )

            # Correct heading separately when position is already good.
            if (
                settled_distance <= tolerance
                and abs(settled_yaw_error)
                > heading_tolerance_rad
            ):
                remaining_time = max(
                    3.0,
                    time_limit
                    - (time.monotonic() - total_start_time),
                )
                self.turn_relative(
                    degrees=math.degrees(settled_yaw_error),
                    angular_speed=min(
                        self.defaults.angular_speed,
                        self.defaults.turn_correction_speed,
                    ),
                    minimum_angular_speed=min(
                        self.defaults.minimum_angular_speed,
                        self.defaults.turn_correction_speed,
                    ),
                    angle_tolerance_deg=(
                        heading_tolerance_deg
                    ),
                    maximum_time=remaining_time,
                )
                (
                    settled_distance,
                    settled_yaw_error,
                    settled_speed,
                    settled_rate,
                ) = self._settle_after_move(target)
                print(
                    "После коррекции курса: "
                    f"ошибка позиции "
                    f"{settled_distance:.3f} м, "
                    f"курс "
                    f"{math.degrees(settled_yaw_error):+.2f}°"
                )

            if (
                settled_distance <= tolerance
                and abs(settled_yaw_error)
                <= heading_tolerance_rad
                and settled_speed
                <= self.defaults.move_settle_speed_mps
                and abs(settled_rate)
                <= self.defaults.move_settle_rate_radps
            ):
                print(
                    f"{label} завершено после полной остановки:\n"
                    f"  итоговая ошибка позиции: "
                    f"{settled_distance:.3f} м\n"
                    f"  итоговая ошибка курса: "
                    f"{math.degrees(settled_yaw_error):+.2f}°\n"
                    f"  итоговая скорость: "
                    f"{settled_speed:.3f} м/с"
                )
                self.publish_stop(0.5)
                return

        _, _, final_distance = self._position_error(target)
        final_yaw_error = target.yaw - self.current_pose().yaw
        raise RuntimeError(
            "Не удалось стабилизировать позицию после "
            f"{self.defaults.move_max_corrections} коррекций; "
            f"ошибка позиции {final_distance:.3f} м, "
            f"ошибка курса "
            f"{math.degrees(final_yaw_error):+.2f}°"
        )

    def move_relative(
        self,
        forward: float,
        left: float,
        **kwargs: Any,
    ) -> None:
        start = self.current_pose()
        cosine = math.cos(start.yaw)
        sine = math.sin(start.yaw)

        target = Pose2D(
            x=start.x + cosine * forward - sine * left,
            y=start.y + sine * forward + cosine * left,
            yaw=start.yaw,
        )

        self.move_to_pose(
            target,
            label=(
                f"движение forward={forward:+.3f} м, "
                f"left={left:+.3f} м"
            ),
            **kwargs,
        )

    def _settle_after_turn(
        self,
        target_yaw: float,
        tolerance_rad: float,
        *,
        timeout: float = 4.0,
    ) -> tuple[float, float]:
        """Command zero until the rover and EKF have actually settled."""
        period = 1.0 / self.defaults.command_rate_hz
        deadline = time.monotonic() + timeout
        stable_since: float | None = None
        last_report = 0.0

        while rclpy.ok():
            self.publisher.publish(Twist())
            self._spin_feedback()
            self._check_odom_freshness()

            pose = self.current_pose()
            angular_velocity = self.current_angular_velocity()
            error = target_yaw - pose.yaw
            now = time.monotonic()

            if (
                abs(angular_velocity)
                <= self.defaults.turn_settle_rate_radps
            ):
                if stable_since is None:
                    stable_since = now
                elif (
                    now - stable_since
                    >= self.defaults.turn_settle_time
                ):
                    return error, angular_velocity
            else:
                stable_since = None

            if now - last_report >= 0.20:
                print(
                    f"\rТорможение: ошибка "
                    f"{math.degrees(error):+.2f}°, "
                    f"ω={angular_velocity:+.3f} рад/с",
                    end="",
                    flush=True,
                )
                last_report = now

            if now >= deadline:
                return error, angular_velocity

            time.sleep(period)

        raise RuntimeError("ROS был остановлен")

    def turn_relative(
        self,
        degrees: float,
        *,
        angular_speed: float | None = None,
        minimum_angular_speed: float | None = None,
        angle_tolerance_deg: float | None = None,
        maximum_time: float | None = None,
    ) -> None:
        requested_max_speed = (
            self.defaults.angular_speed
            if angular_speed is None
            else abs(float(angular_speed))
        )
        requested_min_speed = (
            self.defaults.minimum_angular_speed
            if minimum_angular_speed is None
            else abs(float(minimum_angular_speed))
        )
        tolerance_deg = (
            self.defaults.angle_tolerance_deg
            if angle_tolerance_deg is None
            else abs(float(angle_tolerance_deg))
        )
        configured_time_limit = min(60.0, (
            self.defaults.maximum_step_time
            if maximum_time is None
            else float(maximum_time)
        ))
        time_limit = bounded_turn_time_limit(
            degrees,
            requested_max_speed,
            configured_time_limit,
        )

        if not 0.0 < requested_min_speed <= requested_max_speed:
            raise ValueError(
                "minimum_angular_speed должен быть > 0 "
                "и <= angular_speed"
            )

        start = self.current_pose()
        final_target_yaw = start.yaw + math.radians(degrees)
        coarse_degrees = 0.0
        coarse_target_yaw = (
            final_target_yaw
            - math.copysign(math.radians(coarse_degrees), degrees)
        )
        if not all(math.isfinite(value) and value > 0.0 for value in (
            self.defaults.turn_acceleration_radps2,
            self.defaults.turn_braking_decel_radps2,
        )):
            raise ValueError("Ускорение и торможение поворота должны быть положительными")
        tolerance_rad = math.radians(tolerance_deg)
        if not all(math.isfinite(v) for v in (degrees, tolerance_deg, time_limit)) or not 0 < tolerance_deg <= 15 or time_limit <= 0:
            raise ValueError('Угол и лимит времени должны быть конечными, допуск — (0, 15]°')
        self._check_odom_freshness()
        # Aim at the centre of the acceptance band, not its near edge. The
        # outer tolerance is checked only after actual coast-down has ended.
        control_tolerance = min(tolerance_rad * 0.35, math.radians(0.35))
        total_start_time = time.monotonic()
        period = 1.0 / self.defaults.command_rate_hz

        print(
            f"Поворот на {degrees:+.2f}°: "
            f"цель yaw={math.degrees(final_target_yaw):+.2f}°"
        )

        for correction_index in range(
            self.defaults.turn_max_corrections + 1
        ):
            target_yaw = (
                coarse_target_yaw
                if correction_index == 0 and coarse_degrees
                else final_target_yaw
            )
            pose = self.current_pose()
            initial_error = target_yaw - pose.yaw

            if abs(initial_error) <= tolerance_rad:
                settled_error, settled_rate = self._settle_after_turn(
                    target_yaw,
                    tolerance_rad,
                )
                print()
                if (
                    abs(settled_error) <= tolerance_rad
                    and abs(settled_rate)
                    <= self.defaults.turn_settle_rate_radps
                ):
                    if target_yaw != final_target_yaw:
                        continue
                    actual_turn = self.current_pose().yaw - start.yaw
                    print(
                        "Поворот завершён после полной остановки:\n"
                        f"  итоговая ошибка: "
                        f"{math.degrees(settled_error):+.2f}°\n"
                        f"  угол по {self.odom_topic}: "
                        f"{math.degrees(actual_turn):+.2f}°\n"
                        f"  итоговая ω: "
                        f"{settled_rate:+.3f} рад/с"
                    )
                    return

            if correction_index == 0:
                pass_max_speed = requested_max_speed
                pass_min_speed = requested_min_speed
                phase_name = "основной поворот"
            else:
                pass_max_speed = min(
                    requested_max_speed,
                    self.defaults.turn_correction_speed,
                )
                pass_min_speed = min(
                    max(requested_min_speed, 0.24),
                    pass_max_speed,
                )
                phase_name = (
                    f"коррекция {correction_index}/"
                    f"{self.defaults.turn_max_corrections}"
                )
                print(
                    f"\n{phase_name}: остаточная ошибка "
                    f"{math.degrees(initial_error):+.2f}°"
                )

            best_error = abs(initial_error)
            last_improvement_time = time.monotonic()
            last_report = 0.0
            ramp_speed = 0.0
            ramp_time = time.monotonic()
            progress_reference_error = abs(initial_error)
            progress_reference_time = ramp_time
            progress_window = 3.0

            while rclpy.ok():
                self._spin_feedback()
                self._check_odom_freshness()

                pose = self.current_pose()
                error = target_yaw - pose.yaw
                absolute_error = abs(error)
                angular_velocity = self.current_angular_velocity()

                # Estimate how much angle is still required to stop.
                stopping_angle = (
                    angular_velocity * angular_velocity
                    / (
                        2.0
                        * self.defaults.turn_braking_decel_radps2
                    )
                )
                braking_threshold = (
                    stopping_angle
                    + math.radians(
                        self.defaults.turn_brake_margin_deg
                    )
                )

                moving_toward_target = (
                    angular_velocity == 0.0
                    or math.copysign(1.0, angular_velocity)
                    == math.copysign(1.0, error)
                )

                # Stop before the target, then observe the actual coast-down.
                if (
                    absolute_error <= control_tolerance
                    or (
                        absolute_error <= tolerance_rad
                        and abs(angular_velocity) <= self.defaults.turn_settle_rate_radps
                    )
                    or (
                        moving_toward_target
                        and absolute_error <= braking_threshold
                    )
                    or (
                        angular_velocity != 0.0
                        and math.copysign(1.0, error)
                        != math.copysign(1.0, initial_error)
                    )
                ):
                    self.publisher.publish(Twist())
                    break

                speed = clamp(
                    1.6 * absolute_error,
                    pass_min_speed,
                    pass_max_speed,
                )
                remaining = max(0.0, absolute_error - control_tolerance)
                speed = min(speed, math.sqrt(
                    2.0 * self.defaults.turn_braking_decel_radps2 * remaining
                ))
                if correction_index > 0:
                    speed = max(speed, min(pass_max_speed, 0.24))
                ramp_now = time.monotonic()
                dt = min(max(ramp_now - ramp_time, 0.0), 2.0 * period)
                ramp_time = ramp_now
                rate = (
                    self.defaults.turn_acceleration_radps2
                    if speed > ramp_speed
                    else self.defaults.turn_braking_decel_radps2
                )
                ramp_speed += clamp(speed - ramp_speed, -rate * dt, rate * dt)
                speed = ramp_speed

                command = Twist()
                command.angular.z = math.copysign(speed, error)
                self.publisher.publish(command)

                now = time.monotonic()
                if absolute_error < best_error - math.radians(0.25):
                    best_error = absolute_error
                    last_improvement_time = now

                if now - last_improvement_time > 3.0:
                    raise RuntimeError(
                        f"{phase_name}: ошибка угла не уменьшается"
                    )

                if now - progress_reference_time >= progress_window:
                    progress = progress_reference_error - absolute_error
                    if progress < math.radians(0.75):
                        raise RuntimeError(
                            f"{phase_name}: энкодеры/IMU не "
                            "подтверждают поворот"
                        )
                    progress_reference_error = absolute_error
                    progress_reference_time = now
                    progress_window = 1.5

                if now - total_start_time > time_limit:
                    raise RuntimeError(
                        f"Поворот: превышен лимит "
                        f"{time_limit:.1f} с"
                    )

                if now - last_report >= 0.15:
                    print(
                        f"\r{phase_name}: осталось "
                        f"{math.degrees(error):+.2f}°, "
                        f"ω={angular_velocity:+.3f}, "
                        f"стоп≈"
                        f"{math.degrees(stopping_angle):.1f}°",
                        end="",
                        flush=True,
                    )
                    last_report = now

                time.sleep(period)

            settled_error, settled_rate = self._settle_after_turn(
                target_yaw,
                tolerance_rad,
            )
            print(
                f"\nПосле торможения: ошибка "
                f"{math.degrees(settled_error):+.2f}°, "
                f"ω={settled_rate:+.3f} рад/с"
            )

            if (
                abs(settled_error) <= tolerance_rad
                and abs(settled_rate)
                <= self.defaults.turn_settle_rate_radps
            ):
                if target_yaw != final_target_yaw:
                    print("Основной поворот завершён. Выполняю доводку 2°.")
                    continue
                final_pose = self.current_pose()
                actual_turn = final_pose.yaw - start.yaw
                print(
                    "Поворот завершён после полной остановки:\n"
                    f"  итоговая ошибка: "
                    f"{math.degrees(settled_error):+.2f}°\n"
                    f"  угол по {self.odom_topic}: "
                    f"{math.degrees(actual_turn):+.2f}°\n"
                    f"  итоговая ω: "
                    f"{settled_rate:+.3f} рад/с"
                )
                self.publish_stop(0.5)
                return

        final_error = final_target_yaw - self.current_pose().yaw
        raise RuntimeError(
            "Не удалось стабилизировать поворот после "
            f"{self.defaults.turn_max_corrections} коррекций; "
            f"остаточная ошибка "
            f"{math.degrees(final_error):+.2f}°"
        )

    def return_to_origin(
        self,
        *,
        restore_heading: bool = True,
        **kwargs: Any,
    ) -> None:
        if self.origin is None:
            raise RuntimeError("Начальная точка не сохранена")

        current = self.current_pose()
        target = Pose2D(
            x=self.origin.x,
            y=self.origin.y,
            yaw=(
                self.origin.yaw
                if restore_heading
                else current.yaw
            ),
        )
        self.move_to_pose(
            target,
            label="возврат в начальную точку",
            **kwargs,
        )

    def print_origin_error(self) -> None:
        if self.origin is None:
            return

        pose = self.current_pose()
        dx = pose.x - self.origin.x
        dy = pose.y - self.origin.y
        distance = math.hypot(dx, dy)
        yaw_error = pose.yaw - self.origin.yaw

        print(
            "\nИтоговая ошибка возврата по одометрии:\n"
            f"  dx: {dx:+.3f} м\n"
            f"  dy: {dy:+.3f} м\n"
            f"  расстояние: {distance:.3f} м\n"
            f"  yaw: {math.degrees(yaw_error):+.2f}°"
        )


def load_defaults(plan: dict[str, Any]) -> MotionDefaults:
    values = plan.get("defaults", {})
    return MotionDefaults(
        linear_speed=float(values.get("linear_speed", 0.18)),
        approach_speed=float(values.get("approach_speed", 0.10)),
        position_tolerance=float(
            values.get("position_tolerance", 0.07)
        ),
        angular_speed=float(values.get("angular_speed", 0.45)),
        minimum_angular_speed=float(
            values.get("minimum_angular_speed", 0.12)
        ),
        angle_tolerance_deg=float(
            values.get("angle_tolerance_deg", 1.0)
        ),
        turn_braking_decel_radps2=float(
            values.get("turn_braking_decel_radps2", 0.60)
        ),
        turn_acceleration_radps2=float(
            values.get("turn_acceleration_radps2", 0.45)
        ),
        turn_brake_margin_deg=float(
            values.get("turn_brake_margin_deg", 0.15)
        ),
        turn_correction_speed=float(
            values.get("turn_correction_speed", 0.40)
        ),
        turn_settle_rate_radps=float(
            values.get("turn_settle_rate_radps", 0.015)
        ),
        turn_settle_time=float(
            values.get("turn_settle_time", 0.50)
        ),
        turn_max_corrections=int(
            values.get("turn_max_corrections", 3)
        ),
        move_braking_decel_mps2=float(
            values.get("move_braking_decel_mps2", 0.22)
        ),
        move_acceleration_mps2=float(values.get("move_acceleration_mps2", 0.18)),
        move_brake_margin_m=float(
            values.get("move_brake_margin_m", 0.010)
        ),
        move_correction_speed=float(
            values.get("move_correction_speed", 0.10)
        ),
        move_correction_approach_speed=float(
            values.get(
                "move_correction_approach_speed",
                0.10,
            )
        ),
        move_settle_speed_mps=float(
            values.get("move_settle_speed_mps", 0.015)
        ),
        move_settle_rate_radps=float(
            values.get("move_settle_rate_radps", 0.03)
        ),
        move_settle_time=float(
            values.get("move_settle_time", 0.50)
        ),
        move_max_corrections=int(
            values.get("move_max_corrections", 1)
        ),
        move_path_gain=float(values.get("move_path_gain", 0.80)),
        move_heading_gain=float(values.get("move_heading_gain", 2.20)),
        move_yaw_damping=float(values.get("move_yaw_damping", 0.20)),
        odom_timeout=float(values.get("odom_timeout", 0.50)),
        command_rate_hz=float(
            values.get("command_rate_hz", 25.0)
        ),
        maximum_step_time=float(
            values.get("maximum_step_time", 60.0)
        ),
    )


def execute_plan(
    controller: MotionController,
    plan: dict[str, Any],
) -> None:
    steps = plan.get("steps")
    if not isinstance(steps, list) or not steps:
        raise ValueError("В плане отсутствует непустой список steps")

    for index, step in enumerate(steps, start=1):
        if not isinstance(step, dict):
            raise ValueError(f"Шаг {index} должен быть объектом")

        step_type = str(step.get("type", "")).strip()
        print(f"\n--- Шаг {index}/{len(steps)}: {step_type} ---")

        common: dict[str, Any] = {}
        if "linear_speed" in step:
            common["linear_speed"] = float(step["linear_speed"])
        if "approach_speed" in step:
            common["approach_speed"] = float(
                step["approach_speed"]
            )
        if "position_tolerance" in step:
            common["position_tolerance"] = float(
                step["position_tolerance"]
            )
        if "maximum_time" in step:
            common["maximum_time"] = float(step["maximum_time"])

        common["maximum_time"] = min(60.0, float(common.get("maximum_time", 60.0)))

        try:
            if step_type == "move":
                controller.move_relative(
                    forward=float(step.get("forward", 0.0)),
                    left=0.0,
                    **common,
                )

            elif step_type == "turn":
                controller.turn_relative(
                    degrees=float(step["degrees"]),
                    angular_speed=float(step.get("angular_speed", controller.defaults.angular_speed)),
                    maximum_time=common["maximum_time"],
                )

            else:
                raise ValueError(f"Неподдерживаемый тип шага {step_type!r}")
        except Exception as error:
            controller.publish_stop(0.5)
            print(f"\nШаг {index} остановлен: {error}. Переход к следующему шагу.")


def create_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Замкнутое управление перемещением и поворотом "
            "ровера по одометрии"
        )
    )
    parser.add_argument(
        "--odom-topic",
        default="/odometry/filtered",
        help="источник одометрии, по умолчанию энкодеры + IMU /odometry/filtered",
    )
    parser.add_argument(
        "--cmd-vel-topic",
        default="/cmd_vel",
        help="топик команд скорости",
    )

    subparsers = parser.add_subparsers(
        dest="command",
        required=True,
    )

    move = subparsers.add_parser(
        "move",
        help="переместиться относительно текущей ориентации",
    )
    move.add_argument("--forward", type=float, default=0.0)
    move.add_argument("--left", type=float, default=0.0)
    move.add_argument("--speed", type=float, default=0.18)
    move.add_argument("--approach-speed", type=float, default=None)
    move.add_argument("--position-tolerance", type=float, default=None)
    move.add_argument("--correction-speed", type=float, default=None)

    turn = subparsers.add_parser(
        "turn",
        help="повернуться на заданное число градусов",
    )
    turn.add_argument("degrees", type=float)
    turn.add_argument("--speed", type=float, default=0.32)
    turn.add_argument(
        "--tolerance-deg",
        type=float,
        default=1.0,
    )

    run = subparsers.add_parser(
        "run",
        help="выполнить последовательность из YAML",
    )
    run.add_argument("plan", type=Path)

    return parser


def main() -> None:
    args = create_parser().parse_args()

    if args.command == "run":
        plan = yaml.safe_load(
            args.plan.read_text(encoding="utf-8")
        )
        if not isinstance(plan, dict):
            raise SystemExit("YAML-план должен содержать объект")
        defaults = load_defaults(plan)
    else:
        plan = None
        defaults = MotionDefaults()

    rclpy.init()
    controller = MotionController(
        odom_topic=args.odom_topic,
        cmd_vel_topic=args.cmd_vel_topic,
        defaults=defaults,
    )

    failed = False
    try:
        controller.wait_until_ready()
        controller.publish_stop(0.5)

        if args.command == "move":
            if args.correction_speed is not None:
                controller.defaults.move_correction_speed = args.correction_speed
                controller.defaults.move_correction_approach_speed = args.correction_speed
            controller.move_relative(
                forward=args.forward,
                left=args.left,
                linear_speed=args.speed,
                approach_speed=args.approach_speed,
                position_tolerance=args.position_tolerance,
            )

        elif args.command == "turn":
            controller.turn_relative(
                degrees=args.degrees,
                angular_speed=args.speed,
                angle_tolerance_deg=args.tolerance_deg,
            )

        elif args.command == "run":
            execute_plan(controller, plan)
            controller.print_origin_error()

    except KeyboardInterrupt:
        print("\nОстановлено пользователем")

    except Exception as error:
        failed = True
        print(f"\nАварийная остановка: {error}")

    finally:
        print("\nОстановка моторов...")
        controller.publish_stop(1.0)
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
