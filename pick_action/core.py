"""ROS-independent scan filtering and clustering algorithms."""

from dataclasses import dataclass
import math
from statistics import median
from typing import Iterable, List, Sequence


@dataclass(frozen=True)
class ProcessorConfig:
    """Parameters controlling scan filtering and clustering."""

    range_min_m: float = 0.03
    range_max_m: float = 0.43
    angle_min_deg: float = -170.0
    angle_max_deg: float = -10.0
    x_min_m: float = -0.43
    x_max_m: float = 0.43
    y_min_m: float = -0.43
    y_max_m: float = -0.02
    neighbor_base_gap_m: float = 0.015
    neighbor_gap_scale: float = 3.0
    min_cluster_points: int = 2
    max_cluster_points: int = 80
    min_cluster_width_m: float = 0.0
    max_cluster_width_m: float = 0.15
    range_calibration_scale: float = 1.0
    range_calibration_offset_m: float = 0.0
    range_calibration_cos_m: float = 0.0
    range_calibration_sin_m: float = 0.0
    range_calibration_cos2_m: float = 0.0


@dataclass(frozen=True)
class ScanPoint:
    """One valid point from a LaserScan."""

    index: int
    angle_rad: float
    range_m: float
    x_m: float
    y_m: float
    intensity: float = 0.0


@dataclass(frozen=True)
class Detection:
    """One clustered target in the LiDAR frame."""

    target_id: int
    x_m: float
    y_m: float
    range_m: float
    bearing_rad: float
    point_count: int
    width_m: float


def normalize_angle(angle_rad: float) -> float:
    """Normalize an angle to [-pi, pi)."""
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def angle_in_window(angle_rad: float, minimum_deg: float, maximum_deg: float) -> bool:
    """Return whether angle is inside a possibly wrapping degree interval."""
    angle_deg = math.degrees(normalize_angle(angle_rad))
    minimum = math.degrees(normalize_angle(math.radians(minimum_deg)))
    maximum = math.degrees(normalize_angle(math.radians(maximum_deg)))
    if minimum <= maximum:
        return minimum <= angle_deg <= maximum
    return angle_deg >= minimum or angle_deg <= maximum


def calibrate_range(
    raw_range_m: float,
    angle_rad: float,
    config: ProcessorConfig,
) -> float:
    """Apply the fitted radial calibration in the LiDAR scan plane."""
    return (
        config.range_calibration_scale * raw_range_m
        + config.range_calibration_offset_m
        + config.range_calibration_cos_m * math.cos(angle_rad)
        + config.range_calibration_sin_m * math.sin(angle_rad)
        + config.range_calibration_cos2_m * math.cos(2.0 * angle_rad)
    )


def scan_to_points(
    ranges: Sequence[float],
    angle_min_rad: float,
    angle_increment_rad: float,
    sensor_range_min_m: float,
    sensor_range_max_m: float,
    config: ProcessorConfig,
    intensities: Sequence[float] = (),
) -> List[ScanPoint]:
    """Convert a polar scan to Cartesian points and apply configured ROI."""
    points: List[ScanPoint] = []

    for index, raw_range_m in enumerate(ranges):
        if (
            not math.isfinite(raw_range_m)
            or not sensor_range_min_m <= raw_range_m <= sensor_range_max_m
        ):
            continue

        angle_rad = angle_min_rad + index * angle_increment_rad
        if not angle_in_window(
            angle_rad, config.angle_min_deg, config.angle_max_deg
        ):
            continue

        range_m = calibrate_range(raw_range_m, angle_rad, config)
        if not config.range_min_m <= range_m <= config.range_max_m:
            continue

        x_m = range_m * math.cos(angle_rad)
        y_m = range_m * math.sin(angle_rad)
        if not (
            config.x_min_m <= x_m <= config.x_max_m
            and config.y_min_m <= y_m <= config.y_max_m
        ):
            continue

        intensity = float(intensities[index]) if index < len(intensities) else 0.0
        points.append(
            ScanPoint(index, angle_rad, range_m, x_m, y_m, intensity)
        )

    return points


def _point_distance(first: ScanPoint, second: ScanPoint) -> float:
    return math.hypot(second.x_m - first.x_m, second.y_m - first.y_m)


def split_clusters(
    points: Sequence[ScanPoint],
    angle_increment_rad: float,
    config: ProcessorConfig,
) -> List[List[ScanPoint]]:
    """Split angularly ordered scan points using a range-adaptive gap."""
    if not points:
        return []

    clusters: List[List[ScanPoint]] = [[points[0]]]
    for point in points[1:]:
        previous = clusters[-1][-1]
        average_range = 0.5 * (previous.range_m + point.range_m)
        skipped_beams = max(1, point.index - previous.index)
        beam_gap = average_range * abs(angle_increment_rad) * skipped_beams
        threshold = config.neighbor_base_gap_m + config.neighbor_gap_scale * beam_gap

        if point.index - previous.index <= 2 and _point_distance(previous, point) <= threshold:
            clusters[-1].append(point)
        else:
            clusters.append([point])

    return clusters


def cluster_width(cluster: Sequence[ScanPoint]) -> float:
    """Return maximum end-to-end extent of an angularly ordered cluster."""
    if len(cluster) < 2:
        return 0.0
    return _point_distance(cluster[0], cluster[-1])


def filter_clusters(
    clusters: Iterable[Sequence[ScanPoint]], config: ProcessorConfig
) -> List[List[ScanPoint]]:
    """Reject clusters outside configured point-count and width limits."""
    accepted: List[List[ScanPoint]] = []
    for cluster in clusters:
        width_m = cluster_width(cluster)
        if not config.min_cluster_points <= len(cluster) <= config.max_cluster_points:
            continue
        if not config.min_cluster_width_m <= width_m <= config.max_cluster_width_m:
            continue
        accepted.append(list(cluster))
    return accepted


def clusters_to_detections(
    clusters: Iterable[Sequence[ScanPoint]],
    sort_axis: str = 'x',
    sort_ascending: bool = True,
) -> List[Detection]:
    """Estimate robust cluster centers and assign IDs on a configured axis."""
    centers = []
    for cluster in clusters:
        x_m = float(median(point.x_m for point in cluster))
        y_m = float(median(point.y_m for point in cluster))
        centers.append((x_m, y_m, len(cluster), cluster_width(cluster)))

    axis_index = 0 if sort_axis.lower() == 'x' else 1
    centers.sort(
        key=lambda center: center[axis_index],
        reverse=not sort_ascending,
    )

    detections: List[Detection] = []
    for target_id, (x_m, y_m, point_count, width_m) in enumerate(centers):
        detections.append(
            Detection(
                target_id=target_id,
                x_m=x_m,
                y_m=y_m,
                range_m=math.hypot(x_m, y_m),
                bearing_rad=math.atan2(y_m, x_m),
                point_count=point_count,
                width_m=width_m,
            )
        )
    return detections


def process_scan(
    ranges: Sequence[float],
    angle_min_rad: float,
    angle_increment_rad: float,
    sensor_range_min_m: float,
    sensor_range_max_m: float,
    config: ProcessorConfig,
    intensities: Sequence[float] = (),
    sort_axis: str = 'x',
    sort_ascending: bool = True,
) -> tuple[List[ScanPoint], List[Detection]]:
    """Run ROI filtering, adaptive clustering, and center estimation."""
    points = scan_to_points(
        ranges,
        angle_min_rad,
        angle_increment_rad,
        sensor_range_min_m,
        sensor_range_max_m,
        config,
        intensities,
    )
    clusters = split_clusters(points, angle_increment_rad, config)
    accepted = filter_clusters(clusters, config)
    return points, clusters_to_detections(
        accepted,
        sort_axis=sort_axis,
        sort_ascending=sort_ascending,
    )
