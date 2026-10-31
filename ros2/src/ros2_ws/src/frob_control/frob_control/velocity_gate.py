"""Single owner of the private motor command with stop, expiry and slew limits."""
import json
import math
import time

from frob_interfaces.srv import SetControlMode
from geometry_msgs.msg import Twist, Vector3Stamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class VelocityGate(Node):
    def __init__(self):
        super().__init__('velocity_gate')
        self.declare_parameter('output_topic', '/frob/motor_cmd_vel')
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 2.5)
        self.declare_parameter('linear_acceleration', 1.2)
        self.declare_parameter('linear_deceleration', 2.0)
        self.declare_parameter('angular_acceleration', 4.0)
        self.declare_parameter('angular_deceleration', 6.0)
        self.declare_parameter('manual_timeout', .65)
        self.declare_parameter('navigation_timeout', .4)
        self.declare_parameter('immediate_manual_stop', True)
        self.declare_parameter('straight_hold_enabled', True)
        self.declare_parameter('straight_hold_min_linear', .04)
        self.declare_parameter('straight_hold_command_deadband', .005)
        self.declare_parameter('straight_hold_kp', 2.20)
        self.declare_parameter('straight_hold_kd', .20)
        self.declare_parameter('straight_hold_max_angular', .32)
        self.declare_parameter('straight_hold_odom_timeout', .30)
        self.declare_parameter('encoder_feedback_timeout', .45)
        self.declare_parameter('encoder_motion_timeout', .80)
        self.declare_parameter('encoder_motion_grace', .80)
        self.mode, self.stopped, self.reason = 'manual', False, ''
        self.latest = {}
        self.max_input_gap = {'manual': 0.0, 'navigation': 0.0}
        self.stale_events = 0
        self.was_stale = False
        self.field_heartbeat = 0.
        self.heading_yaw = 0.
        self.heading_rate = 0.
        self.heading_seen = 0.
        self.straight_heading = None
        self.straight_heading_mode = None
        self.encoder_seen = 0.
        self.encoder_motion_seen = 0.
        self.motion_command_since = None
        self.output = Twist()
        self.last_tick = time.monotonic()
        self.pub = self.create_publisher(Twist, self.get_parameter('output_topic').value, 10)
        self.status = self.create_publisher(String, '/control/state', 10)
        inputs = [('manual', '/cmd_vel'),
                  ('navigation', '/frob/nav_cmd_vel')]
        for mode, topic in inputs:
            self.create_subscription(Twist, topic, lambda msg, m=mode: self.receive(m, msg), 1)
        self.create_subscription(Bool, '/field/heartbeat', self.heartbeat, 1)
        self.create_subscription(
            Odometry, '/odometry/filtered', self.odometry, 1)
        self.create_subscription(
            Vector3Stamped, '/wheel/encoder_delta', self.encoders, 10)
        self.create_service(SetControlMode, '/control/set_mode', self.set_mode)
        self.create_service(Trigger, '/control/stop', self.stop)
        self.create_service(Trigger, '/control/reset_stop', self.reset)
        self.create_timer(.05, self.tick)

    def heartbeat(self, msg):
        if msg.data:
            self.field_heartbeat = time.monotonic()

    @staticmethod
    def angle_error(target, current):
        return math.atan2(math.sin(target-current), math.cos(target-current))

    def odometry(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w*q.z + q.x*q.y),
            1.0 - 2.0 * (q.y*q.y + q.z*q.z))
        yaw_rate = float(msg.twist.twist.angular.z)
        if math.isfinite(yaw) and math.isfinite(yaw_rate):
            self.heading_yaw = yaw
            self.heading_rate = yaw_rate
            self.heading_seen = time.monotonic()

    def encoders(self, msg):
        now = time.monotonic()
        self.encoder_seen = now
        if abs(float(msg.vector.x)) + abs(float(msg.vector.y)) >= 1.0:
            self.encoder_motion_seen = now

    def apply_straight_heading_hold(self, target, fresh, now):
        """Keep the start heading while a fresh command requests straight travel."""
        if not bool(self.get_parameter('straight_hold_enabled').value):
            self.straight_heading = None
            return
        min_linear = float(self.get_parameter('straight_hold_min_linear').value)
        deadband = float(
            self.get_parameter('straight_hold_command_deadband').value)
        odom_timeout = float(
            self.get_parameter('straight_hold_odom_timeout').value)
        straight = (
            fresh
            and abs(target.linear.x) >= min_linear
            and abs(target.angular.z) <= deadband
        )
        odom_fresh = self.heading_seen > 0. and now-self.heading_seen <= odom_timeout
        if straight and odom_fresh:
            if self.straight_heading is None or self.straight_heading_mode != self.mode:
                self.straight_heading = self.heading_yaw
                self.straight_heading_mode = self.mode
            error = self.angle_error(self.straight_heading, self.heading_yaw)
            correction = (
                float(self.get_parameter('straight_hold_kp').value) * error
                - float(self.get_parameter('straight_hold_kd').value) * self.heading_rate
            )
            limit = abs(float(
                self.get_parameter('straight_hold_max_angular').value))
            target.angular.z = max(-limit, min(limit, correction))
        elif fresh:
            # A deliberate stop or turn starts a new straight-line reference.
            self.straight_heading = None
            self.straight_heading_mode = None

    def receive(self, mode, msg):
        if not all(math.isfinite(x) for x in (msg.linear.x, msg.angular.z)):
            self.stopped, self.reason = True, 'non-finite velocity'
            return
        now = time.monotonic()
        previous = self.latest.get(mode)
        if previous:
            self.max_input_gap[mode] = max(
                self.max_input_gap[mode], now - previous[0])
        self.latest[mode] = (now, msg)

    def set_mode(self, request, response):
        invalid = request.mode not in ('none', 'manual', 'navigation')
        blocked = self.stopped and request.mode != 'none'
        if invalid or blocked:
            response.success, response.message = False, 'Stop latched or invalid mode'
        else:
            self.mode = request.mode
            self.latest.clear()
            self.was_stale = False
            self.publish_zero()
            response.success, response.message = True, self.mode
        return response

    def stop(self, request, response):
        self.stopped, self.mode, self.reason = True, 'none', 'operator stop'
        self.latest.clear()
        self.publish_zero()
        response.success, response.message = True, 'Stop latched'
        return response

    def reset(self, request, response):
        self.stopped, self.mode, self.reason = False, 'none', ''
        self.latest.clear()
        self.publish_zero()
        response.success, response.message = True, 'Reset; select a new mode to move'
        return response

    @staticmethod
    def approach(current, target, rate, dt):
        step = max(0., rate)*max(0., dt)
        return max(target, current-step) if target < current else min(target, current+step)

    def publish_zero(self):
        self.motion_command_since = None
        self.output = Twist()
        self.straight_heading = None
        self.straight_heading_mode = None
        self.pub.publish(self.output)

    def tick(self):
        now = time.monotonic()
        dt = min(.2, max(.001, now-self.last_tick))
        self.last_tick = now
        target = Twist()
        stamp, command = self.latest.get(self.mode, (0., Twist()))
        if self.mode == 'navigation' and now-self.field_heartbeat > 1.:
            self.stopped, self.reason = True, 'field server heartbeat lost'
        timeout = (self.get_parameter('manual_timeout').value
                   if self.mode == 'manual'
                   else self.get_parameter('navigation_timeout').value)
        fresh = stamp > 0. and now-stamp < timeout
        stale = self.mode != 'none' and not fresh
        if stale and not self.was_stale:
            self.stale_events += 1
        self.was_stale = stale
        if not self.stopped and self.mode != 'none':
            if fresh:
                max_linear = float(self.get_parameter('max_linear_speed').value)
                max_angular = float(self.get_parameter('max_angular_speed').value)
                target.linear.x = max(-max_linear, min(max_linear, command.linear.x))
                target.angular.z = max(-max_angular, min(max_angular, command.angular.z))
            self.apply_straight_heading_hold(target, fresh, now)
            requested_motion = (
                fresh
                and (abs(target.linear.x) >= .02 or abs(target.angular.z) >= .05)
            )
            if requested_motion:
                if self.motion_command_since is None:
                    self.motion_command_since = now
                    self.encoder_motion_seen = now
                grace = float(self.get_parameter('encoder_motion_grace').value)
                if now - self.motion_command_since >= grace:
                    feedback_timeout = float(
                        self.get_parameter('encoder_feedback_timeout').value)
                    motion_timeout = float(
                        self.get_parameter('encoder_motion_timeout').value)
                    if self.encoder_seen <= 0. or now - self.encoder_seen > feedback_timeout:
                        self.stopped, self.mode, self.reason = (
                            True, 'none', 'encoder feedback lost')
                    elif now - self.encoder_motion_seen > motion_timeout:
                        self.stopped, self.mode, self.reason = (
                            True, 'none', 'encoder motion missing')
            else:
                self.motion_command_since = None

            if self.stopped:
                self.latest.clear()
                self.publish_zero()
                return
            explicit_manual_stop = (
                self.mode == 'manual'
                and fresh
                and bool(self.get_parameter('immediate_manual_stop').value)
                and abs(target.linear.x) < 1e-6
                and abs(target.angular.z) < 1e-6
            )
            if explicit_manual_stop:
                self.motion_command_since = None
                self.output = Twist()
            else:
                linear_falling = abs(target.linear.x) < abs(self.output.linear.x)
                angular_falling = abs(target.angular.z) < abs(self.output.angular.z)
                linear_name = 'linear_deceleration' if linear_falling else 'linear_acceleration'
                angular_name = (
                    'angular_deceleration'
                    if angular_falling else 'angular_acceleration'
                )
                linear_rate = self.get_parameter(linear_name).value
                angular_rate = self.get_parameter(angular_name).value
                self.output.linear.x = self.approach(
                    self.output.linear.x, target.linear.x, linear_rate, dt)
                self.output.angular.z = self.approach(
                    self.output.angular.z, target.angular.z, angular_rate, dt)
        else:
            self.motion_command_since = None
            # A latched stop or disabled mode is immediate. A stale command is
            # decelerated above, avoiding a step change on short Wi-Fi gaps.
            self.output = Twist()
        self.pub.publish(self.output)
        state = {'mode': self.mode, 'stopped': self.stopped, 'reason': self.reason,
                 'linear': round(self.output.linear.x, 4),
                 'angular': round(self.output.angular.z, 4),
                 'command_age': None if stamp <= 0. else round(now-stamp, 3),
                 'stale': stale,
                 'stale_events': self.stale_events,
                 'max_input_gap': round(self.max_input_gap.get(self.mode, 0.), 3)}
        self.status.publish(String(data=json.dumps(state)))


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = VelocityGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
