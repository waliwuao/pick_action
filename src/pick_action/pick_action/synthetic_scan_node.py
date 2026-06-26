"""Publish a synthetic LaserScan for development without LiDAR hardware."""

import math
import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class SyntheticScanNode(Node):
    """Generate a repeatable scan with equally spaced targets."""

    def __init__(self) -> None:
        super().__init__('synthetic_spear_scan')
        self.declare_parameter('topic_name', '/scan')
        self.declare_parameter('frame_id', 'base_laser')
        self.declare_parameter('target_count', 3)
        self.declare_parameter('target_x_m', 0.2)
        self.declare_parameter('target_pitch_m', 0.08)
        self.declare_parameter('points_per_target', 5)
        self.declare_parameter('noise_std_m', 0.002)
        self._publisher = self.create_publisher(
            LaserScan, self.get_parameter('topic_name').value, 10
        )
        self._timer = self.create_timer(0.1, self._publish)
        self._random = random.Random(27)
        self.get_logger().info('Publishing synthetic targets on /scan')

    def _publish(self) -> None:
        angle_min = -math.pi
        angle_increment = math.radians(0.167)
        sample_count = int(round(2.0 * math.pi / angle_increment))

        message = LaserScan()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.get_parameter('frame_id').value
        message.angle_min = angle_min
        message.angle_increment = angle_increment
        message.angle_max = angle_min + (sample_count - 1) * angle_increment
        message.time_increment = 1.0 / (10.0 * sample_count)
        message.scan_time = 0.1
        message.range_min = 0.03
        message.range_max = 25.0
        message.ranges = [math.inf] * sample_count
        message.intensities = [0.0] * sample_count

        count = int(self.get_parameter('target_count').value)
        target_x = float(self.get_parameter('target_x_m').value)
        pitch = float(self.get_parameter('target_pitch_m').value)
        points_per_target = int(self.get_parameter('points_per_target').value)
        noise_std = float(self.get_parameter('noise_std_m').value)

        first_y = -0.5 * (count - 1) * pitch
        half_span = points_per_target // 2
        for target_index in range(count):
            target_y = first_y + target_index * pitch
            bearing = math.atan2(target_y, target_x)
            center_index = int(round((bearing - angle_min) / angle_increment))
            nominal_range = math.hypot(target_x, target_y)
            for offset in range(-half_span, half_span + 1):
                index = center_index + offset
                if 0 <= index < sample_count:
                    message.ranges[index] = max(
                        message.range_min,
                        nominal_range + self._random.gauss(0.0, noise_std),
                    )
                    message.intensities[index] = 180.0

        self._publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SyntheticScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
