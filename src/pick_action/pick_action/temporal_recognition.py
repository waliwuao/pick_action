"""Temporal spatial voting for stable target recognition."""

from dataclasses import dataclass
from itertools import combinations
import math
from statistics import median
from typing import Dict, Iterable, List, Sequence, Set, Tuple

from .core import ScanPoint


Cell = Tuple[int, int]


@dataclass(frozen=True)
class StableComponent:
    """A spatially stable connected component accumulated over many frames."""

    x_m: float
    y_m: float
    cell_count: int
    span_x_m: float
    span_y_m: float
    peak_occupancy: float

    @property
    def range_m(self) -> float:
        return math.hypot(self.x_m, self.y_m)

    @property
    def max_span_m(self) -> float:
        return max(self.span_x_m, self.span_y_m)


class TemporalGrid:
    """Count whether each spatial cell is occupied in each frame."""

    def __init__(self, resolution_m: float = 0.005) -> None:
        self.resolution_m = resolution_m
        self.frame_count = 0
        self._cell_frames: Dict[Cell, int] = {}

    def add_frame(self, points: Iterable[ScanPoint]) -> None:
        """Add one frame, counting a cell at most once per frame."""
        cells = {
            (
                round(point.x_m / self.resolution_m),
                round(point.y_m / self.resolution_m),
            )
            for point in points
        }
        self.frame_count += 1
        for cell in cells:
            self._cell_frames[cell] = self._cell_frames.get(cell, 0) + 1

    def stable_components(
        self,
        minimum_occupancy: float = 0.15,
        minimum_cells: int = 4,
        connection_radius_cells: int = 2,
    ) -> List[StableComponent]:
        """Return connected regions repeatedly observed across the recording."""
        if self.frame_count == 0:
            return []

        threshold = max(2, math.ceil(minimum_occupancy * self.frame_count))
        stable: Set[Cell] = {
            cell for cell, count in self._cell_frames.items()
            if count >= threshold
        }
        components: List[List[Cell]] = []
        while stable:
            stack = [stable.pop()]
            component: List[Cell] = []
            while stack:
                cell = stack.pop()
                component.append(cell)
                for dx in range(
                    -connection_radius_cells, connection_radius_cells + 1
                ):
                    for dy in range(
                        -connection_radius_cells, connection_radius_cells + 1
                    ):
                        neighbor = (cell[0] + dx, cell[1] + dy)
                        if neighbor in stable:
                            stable.remove(neighbor)
                            stack.append(neighbor)
            if len(component) >= minimum_cells:
                components.append(component)

        result: List[StableComponent] = []
        for component in components:
            xs = [cell[0] * self.resolution_m for cell in component]
            ys = [cell[1] * self.resolution_m for cell in component]
            peak = max(
                self._cell_frames[cell] / self.frame_count for cell in component
            )
            result.append(
                StableComponent(
                    x_m=float(median(xs)),
                    y_m=float(median(ys)),
                    cell_count=len(component),
                    span_x_m=max(xs) - min(xs),
                    span_y_m=max(ys) - min(ys),
                    peak_occupancy=peak,
                )
            )
        return sorted(result, key=lambda component: component.x_m)


def estimate_arrangement_axis(
    components: Sequence[StableComponent],
) -> Tuple[float, float]:
    """Estimate the target-row unit axis from component centers."""
    if len(components) < 2:
        return (1.0, 0.0)
    mean_x = sum(item.x_m for item in components) / len(components)
    mean_y = sum(item.y_m for item in components) / len(components)
    covariance_xx = sum(
        (item.x_m - mean_x) ** 2 for item in components
    )
    covariance_yy = sum(
        (item.y_m - mean_y) ** 2 for item in components
    )
    covariance_xy = sum(
        (item.x_m - mean_x) * (item.y_m - mean_y)
        for item in components
    )
    angle = 0.5 * math.atan2(
        2.0 * covariance_xy,
        covariance_xx - covariance_yy,
    )
    axis = (math.cos(angle), math.sin(angle))
    # Deterministic sign: prefer +X, then +Y for an almost vertical row.
    if axis[0] < 0.0 or (abs(axis[0]) < 1e-9 and axis[1] < 0.0):
        axis = (-axis[0], -axis[1])
    return axis


def compensate_target_centers(
    components: Sequence[StableComponent],
    along_offsets_m: Sequence[float],
    normal_offsets_m: Sequence[float],
) -> List[StableComponent]:
    """Move observed surface centers to calibrated object centers.

    Offsets use the fitted target-row frame, so a small rotation of the row
    rotates the compensation with it.
    """
    if not components:
        return []
    if (
        len(along_offsets_m) != len(components)
        or len(normal_offsets_m) != len(components)
    ):
        raise ValueError('center-offset count must match target count')

    axis_x, axis_y = estimate_arrangement_axis(components)
    normal_x, normal_y = -axis_y, axis_x
    corrected = []
    for component, along, normal in zip(
        components, along_offsets_m, normal_offsets_m
    ):
        corrected.append(
            StableComponent(
                x_m=(
                    component.x_m
                    + float(along) * axis_x
                    + float(normal) * normal_x
                ),
                y_m=(
                    component.y_m
                    + float(along) * axis_y
                    + float(normal) * normal_y
                ),
                cell_count=component.cell_count,
                span_x_m=component.span_x_m,
                span_y_m=component.span_y_m,
                peak_occupancy=component.peak_occupancy,
            )
        )
    return corrected


def compensate_polar_coordinates(
    components: Sequence[StableComponent],
    range_coefficients_m: Sequence[float],
    angle_coefficients_rad: Sequence[float],
    range_center_m: float = 0.2,
    range_scale_m: float = 0.15,
    angle_center_rad: float = -math.pi / 2.0,
    angle_scale_rad: float = math.pi / 2.0,
) -> List[StableComponent]:
    """Apply a quadratic correction in measured range and bearing.

    Coefficients follow ``1, v, u, v², u*v, u²``, where ``u`` and ``v`` are
    normalized range and bearing. Apply this after target-normal compensation.
    """
    if len(range_coefficients_m) != 6 or len(angle_coefficients_rad) != 6:
        raise ValueError('polar compensation requires six coefficients')
    if range_scale_m <= 0.0 or angle_scale_rad <= 0.0:
        raise ValueError('polar compensation scales must be positive')

    corrected = []
    for component in components:
        range_m = component.range_m
        angle_rad = math.atan2(component.y_m, component.x_m)
        u = (range_m - range_center_m) / range_scale_m
        v = (angle_rad - angle_center_rad) / angle_scale_rad
        features = (1.0, v, u, v * v, u * v, u * u)
        corrected_range_m = range_m + sum(
            float(coefficient) * feature
            for coefficient, feature in zip(range_coefficients_m, features)
        )
        corrected_angle_rad = angle_rad + sum(
            float(coefficient) * feature
            for coefficient, feature in zip(angle_coefficients_rad, features)
        )
        corrected.append(
            StableComponent(
                x_m=corrected_range_m * math.cos(corrected_angle_rad),
                y_m=corrected_range_m * math.sin(corrected_angle_rad),
                cell_count=component.cell_count,
                span_x_m=component.span_x_m,
                span_y_m=component.span_y_m,
                peak_occupancy=component.peak_occupancy,
            )
        )
    return corrected


def select_target_components(
    components: Sequence[StableComponent],
    expected_count: int,
    maximum_component_span_m: float = 0.12,
    maximum_spacing_deviation_ratio: float = 0.35,
    alignment_axis: str = 'auto',
) -> List[StableComponent]:
    """Choose a compact, aligned subset matching the expected target count."""
    candidates = [
        component for component in components
        if component.max_span_m <= maximum_component_span_m
    ]
    if expected_count <= 0 or len(candidates) < expected_count:
        return []

    if expected_count == 1:
        # A compact target is preferred over a long wall edge or chair part.
        chosen = min(
            candidates,
            key=lambda item: (
                item.max_span_m,
                -item.peak_occupancy,
                -item.cell_count,
            ),
        )
        return [chosen]

    best_subset = None
    best_score = math.inf
    for subset in combinations(candidates, expected_count):
        axis_name = alignment_axis.lower()
        if axis_name == 'auto':
            axis_x, axis_y = estimate_arrangement_axis(subset)
            normal_x, normal_y = -axis_y, axis_x
            along = sorted(
                item.x_m * axis_x + item.y_m * axis_y for item in subset
            )
            across = [
                item.x_m * normal_x + item.y_m * normal_y for item in subset
            ]
        elif axis_name == 'x':
            axis_x, axis_y = 1.0, 0.0
            along = sorted(item.x_m for item in subset)
            across = [item.y_m for item in subset]
        else:
            axis_x, axis_y = 0.0, 1.0
            along = sorted(item.y_m for item in subset)
            across = [item.x_m for item in subset]

        # Approximate collinearity is only a soft cue. Do not impose a
        # pairwise Y/across-distance limit: each object keeps its own Y and
        # one displaced object must not invalidate an otherwise valid set.
        across_center = median(across)
        across_residual = (
            sum(abs(value - across_center) for value in across)
            / expected_count
        )
        compactness = sum(item.max_span_m for item in subset) / expected_count
        ranges = [item.range_m for item in subset]
        radial_spread = max(ranges) - min(ranges)
        spans = [item.max_span_m for item in subset]
        shape_spread = max(spans) - min(spans)
        spacing_penalty = 0.0
        if expected_count >= 3:
            spacings = [
                along[index] - along[index - 1]
                for index in range(1, len(along))
            ]
            mean_spacing = sum(spacings) / len(spacings)
            if mean_spacing <= 0.0:
                continue
            maximum_deviation_ratio = max(
                abs(value - mean_spacing) / mean_spacing
                for value in spacings
            )
            if (
                maximum_spacing_deviation_ratio >= 0.0
                and maximum_deviation_ratio
                > maximum_spacing_deviation_ratio
            ):
                continue
            spacing_penalty = (
                sum(abs(value - mean_spacing) for value in spacings)
                / len(spacings)
            )

        # Alignment and spacing are soft ranking cues. A negative spacing
        # threshold disables hard rejection while preserving this score.
        score = (
            2.0 * across_residual
            + 2.0 * spacing_penalty
            + 2.0 * radial_spread
            + shape_spread
            + compactness
        )
        if score < best_score:
            best_score = score
            best_subset = list(subset)

    if best_subset is None:
        return []
    axis_name = alignment_axis.lower()
    if axis_name == 'auto':
        axis_x, axis_y = estimate_arrangement_axis(best_subset)
        return sorted(
            best_subset,
            key=lambda item: item.x_m * axis_x + item.y_m * axis_y,
        )
    if axis_name == 'x':
        return sorted(best_subset, key=lambda item: item.x_m)
    return sorted(best_subset, key=lambda item: item.y_m)


def split_targets_have_plausible_parent(
    standard_components: Sequence[StableComponent],
    split_targets: Sequence[StableComponent],
    expected_count: int,
    maximum_component_span_m: float,
) -> bool:
    """Reject fine-scale targets carved out of an unrelated long structure.

    The fine grid is only a fallback for a target group that was merged at the
    standard scale. All split target centers must therefore lie inside one
    standard component whose total span is plausible for ``expected_count``
    targets.
    """
    if expected_count < 2 or len(split_targets) != expected_count:
        return False

    tolerance_m = 0.005

    # Case 1: all fine-grid targets were separated from one plausible merged
    # standard-grid region.
    span_limit = maximum_component_span_m * expected_count
    for parent in standard_components:
        if parent.max_span_m > span_limit:
            continue
        half_x = 0.5 * parent.span_x_m + tolerance_m
        half_y = 0.5 * parent.span_y_m + tolerance_m
        if all(
            abs(target.x_m - parent.x_m) <= half_x
            and abs(target.y_m - parent.y_m) <= half_y
            for target in split_targets
        ):
            return True

    # Case 2: the standard grid contains several target regions and one or
    # more of them are merged pairs. Every fine target must be covered by a
    # compact standard component, and no parent may manufacture more than two
    # targets.
    assignments: Dict[int, int] = {}
    for target in split_targets:
        matching = []
        for index, parent in enumerate(standard_components):
            if parent.max_span_m > 2.0 * maximum_component_span_m:
                continue
            half_x = 0.5 * parent.span_x_m + tolerance_m
            half_y = 0.5 * parent.span_y_m + tolerance_m
            if (
                abs(target.x_m - parent.x_m) <= half_x
                and abs(target.y_m - parent.y_m) <= half_y
            ):
                matching.append((
                    math.hypot(
                        target.x_m - parent.x_m,
                        target.y_m - parent.y_m,
                    ),
                    index,
                ))
        if not matching:
            return False
        _, parent_index = min(matching)
        assignments[parent_index] = assignments.get(parent_index, 0) + 1

    counts = sorted(assignments.values())
    return (
        math.ceil(expected_count / 2) <= len(counts) < expected_count
        and 2 in counts
        and all(count in (1, 2) for count in counts)
    )


def arrangement_line_equation(
    components: Sequence[StableComponent],
) -> Tuple[float, float, float]:
    """Return the fitted 2D row line as normalized ``a*x + b*y + c = 0``."""
    if not components:
        raise ValueError('at least one component is required')
    mean_x = sum(item.x_m for item in components) / len(components)
    mean_y = sum(item.y_m for item in components) / len(components)
    axis_x, axis_y = estimate_arrangement_axis(components)
    normal_x, normal_y = -axis_y, axis_x
    offset = -(normal_x * mean_x + normal_y * mean_y)
    return normal_x, normal_y, offset


def arrangement_alignment_error(
    components: Sequence[StableComponent],
) -> float:
    """Return mean absolute distance from the fitted target-row centerline."""
    if len(components) < 2:
        return 0.0
    axis_x, axis_y = estimate_arrangement_axis(components)
    normal_x, normal_y = -axis_y, axis_x
    across = [
        item.x_m * normal_x + item.y_m * normal_y
        for item in components
    ]
    center = median(across)
    return sum(abs(value - center) for value in across) / len(across)


def arrangement_spacing_metrics(
    components: Sequence[StableComponent],
) -> Tuple[float, float]:
    """Return mean adjacent spacing and maximum relative spacing deviation."""
    if len(components) < 2:
        raise ValueError('at least two components are required')
    axis_x, axis_y = estimate_arrangement_axis(components)
    along = sorted(
        item.x_m * axis_x + item.y_m * axis_y for item in components
    )
    spacings = [
        along[index] - along[index - 1]
        for index in range(1, len(along))
    ]
    mean_spacing = sum(spacings) / len(spacings)
    if mean_spacing <= 0.0:
        return 0.0, math.inf
    maximum_deviation_ratio = max(
        abs(value - mean_spacing) / mean_spacing for value in spacings
    )
    return mean_spacing, maximum_deviation_ratio
