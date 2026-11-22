"""Thread-safe telemetry and display-only pose calculations for view2cams."""

import math
import threading
import time


def yaw_from_quaternion(q):
    values = (q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Non-finite quaternion')
    norm = math.sqrt(sum(v * v for v in values))
    if norm < 1e-9:
        raise ValueError('Zero quaternion')
    x, y, z, w = (v / norm for v in values)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def relative_pose(pose, origin):
    x, y, yaw = pose
    if origin is None:
        return pose
    ox, oy, oyaw = origin
    dx, dy = x - ox, y - oy
    angle = yaw - oyaw
    return (math.cos(oyaw) * dx + math.sin(oyaw) * dy,
            -math.sin(oyaw) * dx + math.cos(oyaw) * dy,
            math.atan2(math.sin(angle), math.cos(angle)))


def wheel_speeds(msg, left_joint, right_joint):
    """Convert joint velocities to operator-facing RPM."""
    values = {}
    for label, name, sign in (('left', left_joint, -1), ('right', right_joint, 1)):
        try:
            index = msg.name.index(name)
            velocity = msg.velocity[index]
            values[label] = sign * velocity * 60 / (2 * math.pi) if (
                math.isfinite(velocity)) else None
        except (ValueError, IndexError):
            values[label] = None
    return values


class Telemetry:
    def __init__(self, timeout, source='odom', clock=time.monotonic):
        self.timeout = timeout
        self.clock = clock
        self.lock = threading.Lock()
        self.values = {}
        self.source = source
        self.origins = {'odom': None, 'map': None}
        self.notice = None

    def put(self, key, value):
        with self.lock:
            self.values[key] = (self.clock(), value)

    def snapshot(self):
        now = self.clock()
        with self.lock:
            result = {
                key: value if 0 <= now - stamp <= self.timeout else None
                for key, (stamp, value) in self.values.items()
            }
            result['source'] = self.source
            pose = result.get('pose_' + self.source)
            result['position'] = (
                relative_pose(pose, self.origins[self.source])
                if pose is not None else None)
            result['notice'] = (
                self.notice[1] if self.notice and now < self.notice[0]
                else None)
            return result

    def select(self, source):
        if source not in self.origins:
            raise ValueError('Unknown position source')
        with self.lock:
            self.source = source

    def show_notice(self, message):
        with self.lock:
            self.notice = (self.clock() + 3.0, message)

    def reset(self):
        now = self.clock()
        with self.lock:
            entry = self.values.get('pose_' + self.source)
            if (entry is None or entry[1] is None
                    or not 0 <= now - entry[0] <= self.timeout):
                self.notice = (now + 3.0, 'RESET: TF unavailable')
                return False
            self.origins[self.source] = entry[1]
            self.notice = (now + 3.0, 'Origin reset (display only)')
            return True
