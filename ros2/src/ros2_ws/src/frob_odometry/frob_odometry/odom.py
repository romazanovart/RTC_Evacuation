import math
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped, TransformStamped
from tf2_ros import TransformBroadcaster


def integrate_arc(x, y, yaw, left, right, base):
    angle = (right - left) / base
    distance = (right + left) / 2.0
    half = angle / 2.0
    scale = math.sin(half) / half if abs(half) > 1e-9 else 1.0
    return (x + distance * scale * math.cos(yaw + half),
            y + distance * scale * math.sin(yaw + half),
            math.atan2(math.sin(yaw + angle), math.cos(yaw + angle)))


class WheelOdometry(Node):
    def __init__(self):
        super().__init__('wheel_odometry')
        defaults = dict(meters_per_tick=0.0005513, left_meters_per_tick=0.0005525,
                        right_meters_per_tick=0.0005501, wheel_base=0.18,
                        publish_tf=False, odom_frame_id='odom',
                        base_frame_id='base_footprint')
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.left_scale = float(self.get_parameter('left_meters_per_tick').value)
        self.right_scale = float(self.get_parameter('right_meters_per_tick').value)
        self.base = float(self.get_parameter('wheel_base').value)
        if min(self.left_scale, self.right_scale, self.base) <= 0:
            raise ValueError('Wheel scales and wheel base must be positive')
        self.x = self.y = self.yaw = 0.0
        self.last_stamp = -1
        self.distance_variance = 1e-6
        self.yaw_variance = 1e-6
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self) if self.get_parameter('publish_tf').value else None
        self.create_subscription(Vector3Stamped, '/wheel/encoder_delta', self.on_pair, 20)

    def on_pair(self, message):
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        dt = message.vector.z
        if stamp <= self.last_stamp:
            return
        self.last_stamp = stamp
        if not all(math.isfinite(v) for v in (dt, message.vector.x, message.vector.y)) or not 0.001 < dt < 1.0:
            self.get_logger().warning('Encoder interval invalid; sample rejected')
            return
        left = message.vector.x * self.left_scale
        right = message.vector.y * self.right_scale
        if max(abs(left), abs(right)) / dt > 1.5:
            self.get_logger().warning('Implausible encoder jump rejected')
            return
        self.x, self.y, self.yaw = integrate_arc(self.x, self.y, self.yaw, left, right, self.base)
        distance = (left + right) / 2.0
        angle = (right - left) / self.base
        quant = (self.left_scale ** 2 + self.right_scale ** 2) / 12.0
        self.distance_variance += quant / 4.0 + (0.02 * abs(distance)) ** 2
        self.yaw_variance += quant / self.base ** 2 + (0.05 * abs(angle)) ** 2
        odom = Odometry()
        odom.header.stamp = message.header.stamp
        odom.header.frame_id = self.get_parameter('odom_frame_id').value
        odom.child_frame_id = self.get_parameter('base_frame_id').value
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom.twist.twist.linear.x = distance / dt
        odom.twist.twist.angular.z = angle / dt
        for i, value in enumerate((self.distance_variance, self.distance_variance, 1e6, 1e6, 1e6, self.yaw_variance)):
            odom.pose.covariance[i * 7] = value
        vx_var = max(0.0004, quant / (4 * dt * dt) + (0.05 * abs(distance / dt)) ** 2)
        wz_var = max(0.003, quant / (self.base * dt) ** 2 + (0.1 * abs(angle / dt)) ** 2)
        for i, value in enumerate((vx_var, 0.001, 1e6, 1e6, 1e6, wz_var)):
            odom.twist.covariance[i * 7] = value
        self.pub.publish(odom)
        if self.tf:
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = odom.child_frame_id
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.rotation = odom.pose.pose.orientation
            self.tf.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = WheelOdometry()
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
