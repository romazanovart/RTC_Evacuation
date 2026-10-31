import numpy as np
import warnings
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy
import tf2_ros

class LidarFilter(Node):
    def __init__(self):
        super().__init__('lidar_filter')
        
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )
        
        self.filtered_pub = self.create_publisher(LaserScan, '/scan/filtered', 10)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Параметры трансформации
        self.declare_parameter('source_frame', 'laser')  # Фрейм лидара из сообщения
        self.declare_parameter('target_frame', 'base_footprint')  # Целевой фрейм
        self.declare_parameter('filter_radius', 0.15)  # Радиус в метрах
        self.declare_parameter('usable_range', 6.0)
        self.declare_parameter('outlier_distance', 0.12)
        self._sensor_offset = None

    def scan_callback(self, msg):
        if self._sensor_offset is None:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.get_parameter('source_frame').value,
                    self.get_parameter('target_frame').value,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.2)
                )
                self._sensor_offset = (
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                )
            except Exception as e:
                self.get_logger().warning(f"Transform error: {str(e)}")
                return

        tx, ty = self._sensor_offset
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        usable_range = min(float(msg.range_max), self.get_parameter('usable_range').value)
        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= usable_range)
        samples = np.where(valid, ranges, np.nan)
        neighborhood = np.stack([np.roll(samples, shift) for shift in (-2, -1, 0, 1, 2)])
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', RuntimeWarning)
            median = np.nanmedian(neighborhood, axis=0)
        neighbors = np.sum(np.isfinite(neighborhood), axis=0)
        outlier_distance = self.get_parameter('outlier_distance').value
        outliers = valid & (neighbors >= 3) & (np.abs(ranges - median) > outlier_distance)
        filtered_ranges = ranges.copy()
        filtered_ranges[outliers] = median[outliers]
        filtered_ranges[~valid] = np.inf
        radius = self.get_parameter('filter_radius').value
        radius_sq = radius ** 2
        angles = msg.angle_min + np.arange(ranges.size) * msg.angle_increment
        x = filtered_ranges * np.cos(angles)
        y = filtered_ranges * np.sin(angles)
        inside_robot = valid & (((x - tx) ** 2 + (y - ty) ** 2) < radius_sq)
        filtered_ranges[inside_robot] = np.inf

        filtered_scan = LaserScan()
        filtered_scan.header = msg.header
        filtered_scan.angle_min = msg.angle_min
        filtered_scan.angle_max = msg.angle_max
        filtered_scan.angle_increment = msg.angle_increment
        filtered_scan.time_increment = msg.time_increment
        filtered_scan.scan_time = msg.scan_time
        filtered_scan.range_min = msg.range_min
        filtered_scan.range_max = msg.range_max
        filtered_scan.ranges = filtered_ranges.tolist()
        filtered_scan.intensities = msg.intensities
        
        self.filtered_pub.publish(filtered_scan)

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = LidarFilter()
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
