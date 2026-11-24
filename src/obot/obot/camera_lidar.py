"""Display-only LiDAR geometry and thread-safe scan snapshots."""

from dataclasses import dataclass
import math
import statistics
import threading
import time


@dataclass(frozen=True)
class LidarConfig:
    center_half_angle_deg: float = 2.0
    robot_width_m: float = 0.5334
    safety_margin_m: float = 0.0762
    warning_distance_m: float = 1.524
    danger_distance_m: float = 0.6096
    stale_timeout_s: float = 0.5
    front_bumper_x_m: float = 0.30572

    def __post_init__(self):
        values = (
            self.center_half_angle_deg, self.robot_width_m, self.safety_margin_m,
            self.warning_distance_m, self.danger_distance_m,
            self.stale_timeout_s, self.front_bumper_x_m,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError('LiDAR parameters must be finite')
        if not (0 < self.center_half_angle_deg < 90
                and self.robot_width_m > 0 and self.safety_margin_m >= 0
                and 0 < self.danger_distance_m <= self.warning_distance_m
                and self.stale_timeout_s > 0 and self.front_bumper_x_m >= 0):
            raise ValueError('Invalid LiDAR geometry, thresholds, or timeout')


@dataclass(frozen=True)
class Hazard:
    region: str
    distance_m: float
    severity: str


@dataclass(frozen=True)
class LidarReading:
    center_m: float | None
    left_m: float | None
    right_m: float | None
    center_nearest_m: float | None
    coverage_uncertain: bool


@dataclass(frozen=True)
class DisplayDistance:
    distance_m: float | None
    severity: str | None
    held: bool = False


@dataclass(frozen=True)
class DisplayReading:
    front: DisplayDistance
    left: DisplayDistance
    right: DisplayDistance
    center_nearest: DisplayDistance
    coverage_uncertain: bool


@dataclass(frozen=True)
class LidarSnapshot:
    status: str
    reading: DisplayReading | None = None


def scan_points(scan, transform):
    """Transform valid returns and assess coarse forward angular coverage."""
    t = transform.translation
    q = transform.rotation
    values = (t.x, t.y, t.z, q.x, q.y, q.z, q.w,
              scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Non-finite LiDAR transform or scan metadata')
    if (scan.range_min < 0 or scan.range_max <= scan.range_min
            or scan.angle_increment == 0):
        raise ValueError('Invalid LiDAR scan metadata')
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if norm < 1e-9:
        raise ValueError('Invalid LiDAR transform quaternion')
    x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
    r00 = 1 - 2 * (y*y + z*z)
    r01 = 2 * (x*y - z*w)
    r10 = 2 * (x*y + z*w)
    r11 = 1 - 2 * (x*x + z*z)
    points = []
    # Ten-degree bins cover the forward half-plane in base_link. A valid
    # return in each bin is only a coarse availability check, not proof of
    # obstacle-free coverage.
    covered_bins = [False] * 18
    for index, distance in enumerate(scan.ranges):
        if (not math.isfinite(distance) or distance <= 0
                or not scan.range_min <= distance <= scan.range_max):
            continue
        angle = scan.angle_min + index * scan.angle_increment
        cos_angle, sin_angle = math.cos(angle), math.sin(angle)
        sx, sy = distance * cos_angle, distance * sin_angle
        dx = r00 * cos_angle + r01 * sin_angle
        dy = r10 * cos_angle + r11 * sin_angle
        if dx > 0:
            bearing = math.atan2(dy, dx)
            bin_index = min(17, int((bearing + math.pi / 2) * 18 / math.pi))
            covered_bins[bin_index] = True
        points.append((t.x + r00*sx + r01*sy,
                       t.y + r10*sx + r11*sy))
    return points, not all(covered_bins)


def center_distance(points, config):
    half_angle = math.radians(config.center_half_angle_deg)
    distances = [
        max(0.0, x - config.front_bumper_x_m)
        for x, y in points
        if x >= config.front_bumper_x_m and x > 0
        and abs(math.atan2(y, x)) <= half_angle
    ]
    return statistics.median(distances) if distances else None


def corridor_distances(points, config):
    half_width = config.robot_width_m / 2 + config.safety_margin_m
    nearest = {}
    for x, y in points:
        if x < config.front_bumper_x_m or x <= 0 or abs(y) > half_width:
            continue
        distance = x - config.front_bumper_x_m
        region = 'LEFT' if y > half_width / 3 else (
            'RIGHT' if y < -half_width / 3 else 'CENTER')
        nearest[region] = min(distance, nearest.get(region, distance))
    return nearest


def corridor_hazards(points, config):
    nearest = corridor_distances(points, config)
    return tuple(
        Hazard(region, nearest[region],
               'red' if nearest[region] < config.danger_distance_m else 'yellow')
        for region in ('LEFT', 'CENTER', 'RIGHT')
        if region in nearest and nearest[region] <= config.warning_distance_m
    )


def process_scan(scan, transform, config):
    points, coverage_uncertain = scan_points(scan, transform)
    regions = corridor_distances(points, config)
    return LidarReading(center_distance(points, config), regions.get('LEFT'),
                        regions.get('RIGHT'), regions.get('CENTER'),
                        coverage_uncertain)


class _DistanceFilter:
    def __init__(self, config):
        self.config = config
        self.value = None
        self.last_valid = None
        self.severity = None
        self.present = False

    def update(self, distance, stamp):
        self.present = distance is not None
        if distance is None:
            return
        if self.last_valid is None or stamp - self.last_valid > min(0.25, self.config.stale_timeout_s):
            self.value = distance
            self.severity = None
        else:
            self.value = min(distance, self.value + 0.25 * (distance - self.value))
        self.last_valid = stamp
        danger, warning = self.config.danger_distance_m, self.config.warning_distance_m
        severity = 'red' if distance < danger else 'yellow' if distance <= warning else 'green'
        if self.severity == 'red' and self.value < danger + 0.1:
            severity = 'red'
        elif self.severity in ('red', 'yellow') and severity == 'green' and self.value <= warning + 0.1:
            severity = 'yellow'
        self.severity = severity

    def snapshot(self, now):
        if self.last_valid is None or not 0 <= now - self.last_valid <= min(
                0.25, self.config.stale_timeout_s):
            return DisplayDistance(None, None)
        return DisplayDistance(self.value, self.severity, not self.present)


class LidarState:
    """Store immutable readings with monotonic freshness checks."""

    def __init__(self, config, clock=time.monotonic):
        self.config = config
        self.timeout = config.stale_timeout_s
        self.clock = clock
        self.lock = threading.Lock()
        self.stamp = None
        self.filters = {name: _DistanceFilter(config) for name in (
            'center_m', 'left_m', 'right_m', 'center_nearest_m')}
        self.coverage_uncertain = True
        self.error = None

    def put(self, reading, stamp):
        with self.lock:
            if self.stamp is not None and stamp <= self.stamp:
                return
            if self.stamp is None or stamp - self.stamp > self.timeout:
                self.filters = {name: _DistanceFilter(self.config) for name in self.filters}
            self.stamp = stamp
            for name, distance_filter in self.filters.items():
                distance_filter.update(getattr(reading, name), stamp)
            self.coverage_uncertain = reading.coverage_uncertain
            self.error = None

    def unavailable(self, reason):
        with self.lock:
            self.stamp = None
            self.error = reason

    def snapshot(self):
        now = self.clock()
        with self.lock:
            if self.error:
                return LidarSnapshot(self.error)
            if self.stamp is None or not 0 <= now - self.stamp <= self.timeout:
                return LidarSnapshot('stale')
            front, left, right, center_nearest = (
                self.filters[name].snapshot(now)
                for name in ('center_m', 'left_m', 'right_m', 'center_nearest_m'))
            return LidarSnapshot('ok', DisplayReading(
                front, left, right, center_nearest, self.coverage_uncertain))
