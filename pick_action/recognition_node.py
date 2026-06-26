"""ROS 2 node for multi-frame stable target recognition."""

import json
import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .core import ProcessorConfig, scan_to_points
from .temporal_recognition import (
    TemporalGrid,
    arrangement_alignment_error,
    arrangement_line_equation,
    arrangement_spacing_metrics,
    compensate_polar_coordinates,
    compensate_target_centers,
    select_target_components,
    split_targets_have_plausible_parent,
)


class RecognitionNode(Node):
    """Recognize stable, aligned target regions over a frame window."""

    def __init__(self) -> None:
        super().__init__('spear_recognition')
        defaults = ProcessorConfig()
        self.declare_parameter('input_topic', '/scan')
        for name, value in defaults.__dict__.items():
            self.declare_parameter(name, value)
        self.declare_parameter('expected_count', 3)
        self.declare_parameter('window_frames', 10)
        self.declare_parameter('grid_resolution_m', 0.005)
        self.declare_parameter('minimum_occupancy', 0.08)
        self.declare_parameter('maximum_component_span_m', 0.15)
        self.declare_parameter('maximum_spacing_deviation_ratio', -1.0)
        self.declare_parameter(
            'center_offset_along_m',
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        self.declare_parameter(
            'center_offset_normal_m',
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        self.declare_parameter(
            'polar_range_coefficients_m',
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        self.declare_parameter(
            'polar_angle_coefficients_rad',
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        self.declare_parameter('marker_scale_m', 0.04)
        self.declare_parameter('circle_radius_m', 0.045)
        self.declare_parameter('dataset_label', 'LIVE /scan')

        values = {
            name: self.get_parameter(name).value
            for name in defaults.__dict__
        }
        self._config = ProcessorConfig(**values)
        self._grids = self._new_grids()
        self._json_pub = self.create_publisher(
            String, 'spear_recognition/result', 10
        )
        self._marker_pub = self.create_publisher(
            MarkerArray, 'spear_recognition/markers', 10
        )
        self._subscription = self.create_subscription(
            LaserScan,
            self.get_parameter('input_topic').value,
            self._callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f'Expected targets: {self.get_parameter("expected_count").value}; '
            f'window: {self.get_parameter("window_frames").value} frames'
        )

    def _new_grids(self):
        return [
            (
                'standard',
                TemporalGrid(
                    resolution_m=float(
                        self.get_parameter('grid_resolution_m').value
                    )
                ),
                float(self.get_parameter('minimum_occupancy').value),
                2,
            ),
            ('near_split', TemporalGrid(resolution_m=0.003), 0.60, 2),
            ('ultra_near_split', TemporalGrid(resolution_m=0.002), 0.50, 1),
        ]

    def _callback(self, scan: LaserScan) -> None:
        points = scan_to_points(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
            self._config,
            scan.intensities,
        )
        for _, grid, _, _ in self._grids:
            grid.add_frame(points)
        if self._grids[0][1].frame_count < int(
            self.get_parameter('window_frames').value
        ):
            return

        expected_count = int(self.get_parameter('expected_count').value)
        selected_profile = 'none'
        components = []
        targets = []
        standard_components = []
        maximum_span = float(
            self.get_parameter('maximum_component_span_m').value
        )
        profile_solutions = []
        profile_penalties = {
            'standard': 0.0,
            'near_split': 0.0005,
            'ultra_near_split': 0.0010,
        }
        for profile, grid, occupancy, connection_radius in self._grids:
            profile_components = grid.stable_components(
                minimum_occupancy=occupancy,
                connection_radius_cells=connection_radius,
            )
            if profile == 'standard':
                standard_components = profile_components
            profile_targets = select_target_components(
                profile_components,
                expected_count=expected_count,
                maximum_component_span_m=maximum_span,
                maximum_spacing_deviation_ratio=float(
                    self.get_parameter(
                        'maximum_spacing_deviation_ratio'
                    ).value
                ),
                alignment_axis='auto',
            )
            if (
                profile in ('near_split', 'ultra_near_split')
                and profile_targets
                and not split_targets_have_plausible_parent(
                    standard_components,
                    profile_targets,
                    expected_count,
                    maximum_span,
                )
            ):
                continue
            if len(profile_targets) == expected_count:
                score = (
                    arrangement_alignment_error(profile_targets)
                    + profile_penalties[profile]
                )
                profile_solutions.append(
                    (score, profile, profile_components, profile_targets)
                )
        if profile_solutions:
            _, selected_profile, components, targets = min(profile_solutions)
        raw_targets = targets
        if (
            len(raw_targets) == expected_count
            and len(raw_targets)
            == len(self.get_parameter('center_offset_along_m').value)
        ):
            targets = compensate_target_centers(
                raw_targets,
                self.get_parameter('center_offset_along_m').value,
                self.get_parameter('center_offset_normal_m').value,
            )
        if len(targets) == expected_count:
            targets = compensate_polar_coordinates(
                targets,
                self.get_parameter('polar_range_coefficients_m').value,
                self.get_parameter('polar_angle_coefficients_rad').value,
            )
        self._publish(
            scan,
            components,
            raw_targets,
            targets,
            expected_count,
            selected_profile,
        )
        self._grids = self._new_grids()

    def _publish(
        self,
        scan,
        components,
        raw_targets,
        targets,
        expected_count,
        selected_profile,
    ) -> None:
        line = arrangement_line_equation(targets) if len(targets) >= 2 else None
        spacing = (
            arrangement_spacing_metrics(targets)
            if len(targets) >= 2 else None
        )
        message = String()
        message.data = json.dumps(
            {
                'frame_id': scan.header.frame_id,
                'expected_count': expected_count,
                'recognized_count': len(targets),
                'profile': selected_profile,
                'status': (
                    'recognized'
                    if len(targets) == expected_count else 'not_separable'
                ),
                'stable_component_count': len(components),
                'arrangement_line_2d': (
                    {
                        'form': 'a*x + b*y + c = 0',
                        'a': round(line[0], 6),
                        'b': round(line[1], 6),
                        'c': round(line[2], 6),
                    }
                    if line is not None else None
                ),
                'mean_adjacent_spacing_m': (
                    round(spacing[0], 6) if spacing is not None else None
                ),
                'maximum_spacing_deviation_ratio': (
                    round(spacing[1], 4) if spacing is not None else None
                ),
                'targets': [
                    {
                        'id': index,
                        'x_m': round(target.x_m, 4),
                        'y_m': round(target.y_m, 4),
                        'raw_component_x_m': round(raw_target.x_m, 4),
                        'raw_component_y_m': round(raw_target.y_m, 4),
                        'span_x_m': round(target.span_x_m, 4),
                        'span_y_m': round(target.span_y_m, 4),
                    }
                    for index, (raw_target, target) in enumerate(
                        zip(raw_targets, targets)
                    )
                ],
            },
            ensure_ascii=False,
        )
        self._json_pub.publish(message)

        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        scale = float(self.get_parameter('marker_scale_m').value)

        dataset = Marker()
        dataset.header = scan.header
        dataset.ns = 'dataset_label'
        dataset.id = 0
        dataset.type = Marker.TEXT_VIEW_FACING
        dataset.action = Marker.ADD
        dataset.pose.position.x = 0.0
        dataset.pose.position.y = -0.06
        dataset.pose.position.z = 0.12
        dataset.pose.orientation.w = 1.0
        dataset.scale.z = 0.035
        dataset.color.r = 0.2
        dataset.color.g = 0.8
        dataset.color.b = 1.0
        dataset.color.a = 1.0
        dataset.text = str(self.get_parameter('dataset_label').value)
        markers.markers.append(dataset)
        target_ids = {id(target) for target in targets}
        for index, component in enumerate(components):
            candidate = Marker()
            candidate.header = scan.header
            candidate.ns = 'stable_candidates'
            candidate.id = index
            candidate.type = Marker.SPHERE
            candidate.action = Marker.ADD
            candidate.pose.position.x = component.x_m
            candidate.pose.position.y = component.y_m
            candidate.pose.orientation.w = 1.0
            candidate.scale.x = scale * 0.35
            candidate.scale.y = scale * 0.35
            candidate.scale.z = scale * 0.35
            candidate.color.r = 1.0
            candidate.color.g = 0.85
            candidate.color.b = 0.0
            candidate.color.a = 0.8 if id(component) not in target_ids else 0.0
            markers.markers.append(candidate)

        radius = float(self.get_parameter('circle_radius_m').value)
        for index, target in enumerate(targets):
            circle = Marker()
            circle.header = scan.header
            circle.ns = 'recognized_target_circles'
            circle.id = index
            circle.type = Marker.LINE_STRIP
            circle.action = Marker.ADD
            circle.pose.orientation.w = 1.0
            circle.scale.x = 0.006
            circle.color.r = 1.0
            circle.color.g = 0.05
            circle.color.b = 0.05
            circle.color.a = 1.0
            for point_index in range(37):
                angle = 2.0 * math.pi * point_index / 36.0
                point = Point()
                point.x = target.x_m + radius * math.cos(angle)
                point.y = target.y_m + radius * math.sin(angle)
                point.z = 0.01
                circle.points.append(point)
            markers.markers.append(circle)

            center = Marker()
            center.header = scan.header
            center.ns = 'recognized_target_centers'
            center.id = index
            center.type = Marker.SPHERE
            center.action = Marker.ADD
            center.pose.position.x = target.x_m
            center.pose.position.y = target.y_m
            center.pose.position.z = 0.01
            center.pose.orientation.w = 1.0
            center.scale.x = scale * 0.35
            center.scale.y = scale * 0.35
            center.scale.z = scale * 0.35
            center.color.r = 1.0
            center.color.g = 0.05
            center.color.b = 0.05
            center.color.a = 1.0
            markers.markers.append(center)

            label = Marker()
            label.header = scan.header
            label.ns = 'recognized_target_labels'
            label.id = index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = target.x_m
            label.pose.position.y = target.y_m
            label.pose.position.z = 0.06
            label.pose.orientation.w = 1.0
            label.scale.z = 0.035
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = (
                f'ID {index}\n'
                f'x={target.x_m:.3f} y={target.y_m:.3f}'
            )
            markers.markers.append(label)
        self._marker_pub.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RecognitionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
