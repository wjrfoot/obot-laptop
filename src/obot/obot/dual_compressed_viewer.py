#!/usr/bin/env python3

from dataclasses import dataclass
import math
import threading
import time
import uuid

import cv2
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import numpy as np
from obot_msgs.srv import ClickDrive
from PyQt5.QtCore import QEvent, QObject, Qt
from PyQt5.QtWidgets import QApplication, QWidget
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, JointState, LaserScan
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .camera_controls import control_metrics, fit_text, push_button as _push_button
from .camera_dashboard import render_dashboard
from .camera_lidar import (
    center_distance, corridor_distances, corridor_obstacle_blobs, LidarConfig,
    LidarReading, LidarState, scan_points,
)
from .camera_spray import draw_spray_panel, SprayControl, SpraySnapshot
from .camera_telemetry import Telemetry, wheel_speeds, yaw_from_quaternion
from .camera_units import display_linear, meters
from .fisheye_bearing import FisheyeBearing


OVERLAY_DEFAULTS = {
    'rear_scale': 0.5,
    'center_marker_enabled': True,
    'center_offset_fraction': 0.0,
    'center_color': [0, 255, 0],
    'center_thickness': 2,
    'center_height_fraction': 0.20,
    'clearance_enabled': True,
    'clearance_offset_fraction': 0.25,
    'clearance_color': [0, 0, 255],
    'clearance_thickness': 2,
    'clearance_height_fraction': 0.20,
}

TELEMETRY_DEFAULTS = {
    'telemetry.enabled': True,
    'telemetry.refresh_hz': 5.0,
    'telemetry.stale_timeout_s': 2.0,
    'telemetry.command_topic': '/diff_drive_controller/cmd_vel',
    'telemetry.odom_topic': '/diff_drive_controller/odom',
    'telemetry.joint_states_topic': '/joint_states',
    'telemetry.odom_frame': 'odom',
    'telemetry.map_frame': 'map',
    'telemetry.robot_frame': 'base_link',
    'telemetry.default_position_source': 'odom',
    'telemetry.left_joint': 'left_drive_wheel_joint',
    'telemetry.right_joint': 'right_drive_wheel_joint',
    'telemetry.left_voltage_topic': '/obot2/watchdog/bus_voltage/left',
    'telemetry.right_voltage_topic': '/obot2/watchdog/bus_voltage/right',
    'telemetry.health_topic': '/obot2/watchdog/health',
}

LIDAR_DEFAULTS = {
    'lidar.enabled': True,
    'lidar.topic': '/scan',
    'lidar.center_half_angle_deg': 2.0,
    'lidar.robot_width_m': 0.5334,
    'lidar.safety_margin_m': 0.0762,
    'lidar.warning_distance_m': 2.0,
    'lidar.danger_distance_m': 1.0,
    'lidar.stale_timeout_s': 0.5,
    'lidar.front_bumper_x_m': 0.30572,
}

DIRECTIONAL_DEFAULTS = {
    'directional.min_speed_mps': 0.2,
    'directional.max_speed_mps': 2.5,
    'directional.default_speed_mps': 0.2,
    'directional.linear_limit_mps': 0.2,
    'directional.angular_limit_rps': 0.35,
    'directional.wheel_separation_m': 0.486,
}

SPEED_PRESET_DEFAULTS = {
    'speed.slow_mps': 0.20,
    'speed.medium_mps': 0.50,
    'speed.fast_mps': 1.00,
}

GO_DEFAULTS = {
    'go.target_standoff_m': 1.0,
    'obstacle.min_diameter_fraction': 0.015,
    'obstacle.max_diameter_fraction': 0.05,
}

MOTION_DEFAULTS = {
    'motion.final_approach_distance_m': 1.0,
    'motion.final_approach_speed_mps': 0.20,
}

# Lets ints and floats (and lists of either) be set from the command line.
_FLEXIBLE = ParameterDescriptor(dynamic_typing=True)


class OverlayConfig:
    """Validated rear-camera scale and front-camera marker settings."""

    def __init__(self, params, warn=None):
        self._warn = warn or (lambda text: None)
        self.rear_scale = self._number(
            params, 'rear_scale', 1e-3, 1.0)
        self.center_enabled = bool(params['center_marker_enabled'])
        self.center_offset = self._number(
            params, 'center_offset_fraction', -1.0, 1.0)
        self.center_bgr = self._color(params, 'center_color')
        self.center_thickness = self._thickness(
            params, 'center_thickness')
        self.center_height = self._number(
            params, 'center_height_fraction', 0.0, 1.0)
        self.clearance_enabled = bool(params['clearance_enabled'])
        self.clearance_offset = self._number(
            params, 'clearance_offset_fraction', 0.0, 1.0)
        self.clearance_bgr = self._color(params, 'clearance_color')
        self.clearance_thickness = self._thickness(
            params, 'clearance_thickness')
        self.clearance_height = self._number(
            params, 'clearance_height_fraction', 0.0, 1.0)

    def _fallback(self, name, why):
        default = OVERLAY_DEFAULTS[name]
        self._warn(f"Invalid '{name}': {why}; using default {default}")
        return default

    def _number(self, params, name, lower, upper):
        try:
            value = float(params[name])
        except (TypeError, ValueError):
            return self._fallback(name, 'not a number')
        if not math.isfinite(value) or not lower <= value <= upper:
            return self._fallback(
                name, f'must be between {lower} and {upper}')
        return value

    def _thickness(self, params, name):
        try:
            value = int(params[name])
        except (TypeError, ValueError):
            return self._fallback(name, 'not an integer')
        if value < 1:
            return self._fallback(name, 'must be at least 1')
        return value

    def _color(self, params, name):
        try:
            rgb = [int(c) for c in params[name]]
            if len(rgb) != 3 or any(not 0 <= c <= 255 for c in rgb):
                raise ValueError
        except (TypeError, ValueError):
            rgb = self._fallback(name, 'need three values from 0 to 255')
        return (rgb[2], rgb[1], rgb[0])

    @property
    def show_clearance(self):
        return self.clearance_enabled and self.clearance_offset > 0.0


@dataclass
class TargetTrackerState:
    tracker: object | None = None
    pixel_x: float = 0.0
    pixel_y: float = 0.0
    bearing_deg: float = 0.0
    status: str = 'idle'


class TargetTrackingWorker:
    """Track against only the newest camera frame on a dedicated thread."""

    def __init__(self, tracker_factory, bearing_fn, stale_timeout_s=0.75):
        self._tracker_factory = tracker_factory
        self._bearing_fn = bearing_fn
        self._stale_timeout_s = stale_timeout_s
        self._condition = threading.Condition()
        self._frame = None
        self._frame_sequence = 0
        self._selection_sequence = 0
        self._selection = None
        self._last_frame_time = 0.0
        self._state = TargetTrackerState()
        self._stopping = False
        self._thread = threading.Thread(
            target=self._run, name='obot-target-tracker', daemon=True)
        self._thread.start()

    def submit_frame(self, frame):
        with self._condition:
            self._frame = frame
            self._frame_sequence += 1
            self._last_frame_time = time.monotonic()
            self._condition.notify()

    def select(self, pixel_x, pixel_y, bearing_deg):
        with self._condition:
            self._selection = (float(pixel_x), float(pixel_y), float(bearing_deg))
            self._selection_sequence += 1
            self._state = TargetTrackerState(
                pixel_x=float(pixel_x), pixel_y=float(pixel_y),
                bearing_deg=float(bearing_deg), status='initializing')
            self._condition.notify()

    def clear(self):
        with self._condition:
            self._selection = None
            self._selection_sequence += 1
            self._state = TargetTrackerState()
            self._condition.notify()

    def snapshot(self):
        with self._condition:
            state = TargetTrackerState(
                pixel_x=self._state.pixel_x,
                pixel_y=self._state.pixel_y,
                bearing_deg=self._state.bearing_deg,
                status=self._state.status)
            if (state.status == 'tracking'
                    and time.monotonic() - self._last_frame_time > self._stale_timeout_s):
                state.status = 'lost'
            return state

    def close(self):
        with self._condition:
            self._stopping = True
            self._condition.notify()
        self._thread.join(timeout=0.1)

    def _run(self):
        tracker = None
        consumed_frame_sequence = 0
        consumed_selection_sequence = 0
        while True:
            with self._condition:
                self._condition.wait_for(
                    lambda: self._stopping
                    or self._frame_sequence != consumed_frame_sequence
                    or self._selection_sequence != consumed_selection_sequence)
                if self._stopping:
                    return
                frame = self._frame
                frame_sequence = self._frame_sequence
                selection_sequence = self._selection_sequence
                selection_changed = selection_sequence != consumed_selection_sequence
                selection = self._selection
            if selection_changed:
                consumed_selection_sequence = selection_sequence
                tracker = None
                consumed_frame_sequence = frame_sequence
                if selection is None:
                    continue
                if frame is None:
                    self._publish(
                        selection_sequence,
                        TargetTrackerState(
                            pixel_x=selection[0], pixel_y=selection[1],
                            bearing_deg=selection[2], status='lost'))
                    continue
                pixel_x, pixel_y, bearing_deg = selection
                tracker = self._tracker_factory() if self._tracker_factory else None
                if tracker is None:
                    self._publish(
                        selection_sequence,
                        TargetTrackerState(
                            pixel_x=pixel_x, pixel_y=pixel_y,
                            bearing_deg=bearing_deg, status='lost'))
                    continue
                box_size = max(20, min(frame.shape[1], frame.shape[0]) // 12)
                x0 = int(max(0, min(
                    frame.shape[1] - box_size, pixel_x - box_size // 2)))
                y0 = int(max(0, min(
                    frame.shape[0] - box_size, pixel_y - box_size // 2)))
                try:
                    initialized = tracker.init(
                        frame, (x0, y0, box_size, box_size))
                    if initialized is False:
                        raise RuntimeError('tracker initialization rejected')
                    self._publish(
                        selection_sequence,
                        TargetTrackerState(
                            pixel_x=pixel_x, pixel_y=pixel_y,
                            bearing_deg=bearing_deg, status='tracking'))
                except Exception:
                    tracker = None
                    self._publish(
                        selection_sequence,
                        TargetTrackerState(
                            pixel_x=pixel_x, pixel_y=pixel_y,
                            bearing_deg=bearing_deg, status='lost'))
                continue
            consumed_frame_sequence = frame_sequence
            if tracker is None or selection is None:
                continue
            try:
                ok, box = tracker.update(frame)
                if not ok:
                    raise RuntimeError('tracker lost target')
                x, y, width, height = box
                pixel_x = min(max(0.0, x + width / 2), frame.shape[1] - 1.0)
                pixel_y = min(max(0.0, y + height / 2), frame.shape[0] - 1.0)
                bearing_deg = self._bearing_fn(pixel_x, pixel_y)
                state = TargetTrackerState(
                    pixel_x=pixel_x, pixel_y=pixel_y,
                    bearing_deg=bearing_deg, status='tracking')
            except Exception:
                tracker = None
                state = TargetTrackerState(
                    pixel_x=selection[0], pixel_y=selection[1],
                    bearing_deg=selection[2], status='lost')
            self._publish(selection_sequence, state)

    def _publish(self, selection_sequence, state):
        with self._condition:
            if selection_sequence == self._selection_sequence:
                self._state = state


def _tracker_factory():
    if hasattr(cv2, 'TrackerCSRT_create'):
        return cv2.TrackerCSRT_create
    if hasattr(cv2, 'legacy') and hasattr(cv2.legacy, 'TrackerMOSSE_create'):
        return cv2.legacy.TrackerMOSSE_create
    if hasattr(cv2, 'legacy') and hasattr(cv2.legacy, 'TrackerKCF_create'):
        return cv2.legacy.TrackerKCF_create
    return None


def _marker(img, x, thickness, height_fraction, bgr):
    """Draw one short vertical bar centered on the image midline."""
    h, w = img.shape[:2]
    half = int(round(h * height_fraction / 2.0))
    left = x - thickness // 2
    right = left + thickness
    if half <= 0 or right <= 0 or left >= w:
        return
    mid = h // 2
    img[max(0, mid - half):min(h, mid + half + 1),
        max(0, left):min(w, right)] = bgr


def draw_markers(img, cfg):
    """Draw the center and clearance markers in place on the front image."""
    w = img.shape[1]
    center_x = int(round(w / 2.0 + cfg.center_offset * w))
    if cfg.center_enabled:
        _marker(img, center_x, cfg.center_thickness,
                cfg.center_height, cfg.center_bgr)
    if cfg.show_clearance:
        delta = int(round(cfg.clearance_offset * w))
        for x in (center_x - delta, center_x + delta):
            _marker(img, x, cfg.clearance_thickness,
                    cfg.clearance_height, cfg.clearance_bgr)


def lidar_labels(snapshot, units='m'):
    """Return compact status labels without left/right distance text."""
    if snapshot.status != 'ok':
        return [('LIDAR STALE' if snapshot.status == 'stale'
                 else 'LIDAR ' + snapshot.status.upper(), (0, 180, 255))]
    reading = snapshot.reading
    colors = {'red': (0, 0, 255), 'yellow': (0, 255, 255),
              'green': (70, 220, 70), None: (180, 180, 180)}
    value = reading.front
    labels = []
    near = reading.center_nearest
    front = value
    rank = {None: 0, 'green': 1, 'yellow': 2, 'red': 3}
    if (near.distance_m is not None and near.severity in ('red', 'yellow')
            and (front.distance_m is None
                 or near.distance_m + 0.05 < front.distance_m
                 or rank[near.severity] > rank[front.severity])):
        prefix = '~' if near.held else ''
        labels.append((f'NEAR: {prefix}{display_linear(near.distance_m, units):.1f} {units}',
                       colors[near.severity]))
    if reading.coverage_uncertain:
        labels.append(('LIDAR COVERAGE UNKNOWN', (0, 180, 255)))
    return labels


def draw_turn_indicator(img, bearing):
    direction = 'RIGHT' if bearing >= 0 else 'LEFT'
    text = f'TURN {direction} {abs(bearing):.0f}'
    scale, thickness = 0.4, 1
    (width, height), baseline = cv2.getTextSize(
        text, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)
    x, y = 7, 7 + height
    cv2.rectangle(img, (4, 4), (x + width + 7, y + baseline + 2), (0, 0, 0), -1)
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                scale, (0, 255, 255), thickness, cv2.LINE_AA)
    cv2.circle(img, (x + width + 3, y - height + 2), 2, (0, 255, 255), 1)


def draw_lidar(img, snapshot, top=4, units='m'):
    """Draw compact labels at the upper right, without adding mouse hitboxes."""
    height, width = img.shape[:2]
    if width < 100 or height < 60:
        return
    labels = lidar_labels(snapshot, units)
    if not labels:
        return
    available = max(1, height // 3 - top)
    scale = min(0.6, max(0.25, width / 1000), max(0.25, available / (25 * len(labels))))
    thickness = 1 if scale < 0.5 else 2
    sizes = [cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)[0]
             for text, _ in labels]
    row = max(size[1] for size in sizes) + 7
    for i, ((text, color), (text_width, text_height)) in enumerate(
            zip(labels, sizes)):
        x = max(0, width - text_width - 10)
        y = top + i * row
        cv2.rectangle(img, (x, y), (width - 1, min(height - 1, y + row - 1)),
                      (0, 0, 0), -1)
        cv2.putText(img, text, (x + 4, y + text_height + 2),
                    cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)


def scale_rear(rear, front_w, front_h, scale):
    """Fit the rear image in scale*front size, keeping its aspect ratio."""
    box_w = max(1, int(round(front_w * scale)))
    box_h = max(1, int(round(front_h * scale)))
    rh, rw = rear.shape[:2]
    factor = min(box_w / rw, box_h / rh)
    size = (max(1, int(round(rw * factor))),
            max(1, int(round(rh * factor))))
    if size != (rw, rh):
        rear = cv2.resize(rear, size, interpolation=cv2.INTER_AREA)
    return rear, box_w, box_h


def compose(front, rear, scale):
    """Place the scaled rear image to the right of the full-size front."""
    fh, fw = front.shape[:2]
    rear, box_w, _ = scale_rear(rear, fw, fh, scale)
    canvas = np.zeros((fh, fw + box_w, 3), dtype=np.uint8)
    canvas[:, :fw] = front
    canvas[:rear.shape[0], fw:fw + rear.shape[1]] = rear
    return canvas


def maximize_window(name):
    """Maximize an OpenCV Qt window without switching to fullscreen."""
    app = QApplication.instance()
    if app is None:
        return False
    for window in app.topLevelWidgets():
        if window.windowTitle() == name:
            window.showMaximized()
            return True
    return False


def display_sized_image(image, window_name):
    """Scale the composite to the displayed image area before drawing text."""
    _, _, width, height = cv2.getWindowImageRect(window_name)
    if width <= 0 or height <= 0:
        return image
    image_h, image_w = image.shape[:2]
    factor = min(width / image_w, height / image_h)
    if factor == 1:
        return image
    size = (max(1, round(image_w * factor)),
            max(1, round(image_h * factor)))
    return cv2.resize(image, size, interpolation=cv2.INTER_LINEAR)


def directional_speeds(selected, settings):
    linear = min(selected, settings['linear_limit_mps'])
    angular = min(2.0 * linear / settings['wheel_separation_m'],
                  settings['angular_limit_rps'])
    return linear, angular


def validate_directional_settings(settings):
    if (not all(math.isfinite(value) and value > 0 for value in settings.values())
            or settings['min_speed_mps'] >= settings['max_speed_mps']):
        raise ValueError('Invalid directional speeds or wheel separation')


def window_has_focus(name):
    app = QApplication.instance()
    if app is None:
        return False
    return any(widget.windowTitle() == name and widget.isActiveWindow()
               for widget in app.topLevelWidgets())


class MotionWindowEvents(QObject):
    def __init__(self, viewer):
        super().__init__()
        self.viewer = viewer

    def eventFilter(self, watched, event):
        if (event.type() == QEvent.KeyPress and isinstance(watched, QWidget)
                and watched.window().windowTitle() in (
                    self.viewer.combined_window, self.viewer.window_left,
                    self.viewer.window_right)
                and event.key() == Qt.Key_Space):
            if not event.isAutoRepeat():
                self.viewer._universal_stop()
            return True
        if (event.type() in (QEvent.Leave, QEvent.WindowDeactivate, QEvent.Close)
                and isinstance(watched, QWidget)):
            window = (self.viewer.combined_window
                      if self.viewer.show_side_by_side else self.viewer.window_left)
            if watched.window().windowTitle() == window:
                self.viewer._spray_down = None
                self.viewer._manual_stop()
        return False


class MovementRequest:
    """Advance the stop barrier and one-use permit without blocking the GUI."""

    def __init__(self, viewer, client, request):
        self.viewer = viewer
        self.client = client
        self.request = request
        self.started = time.monotonic()
        self.error = None
        self.response = None
        if viewer._motion_session_ready:
            self.stage = 'prepare'
            self.future = self._prepare()
        else:
            self.stage = 'startup_stop'
            stop = ClickDrive.Request()
            stop.command = 'STOP'
            self.future = client.call_async(stop)

    def _prepare(self):
        prepare = ClickDrive.Request()
        prepare.command = 'PREPARE'
        prepare.client_id = self.request.client_id
        prepare.sequence = self.request.sequence
        prepare.permit_command = self.request.command
        prepare.permit_id = self.request.permit_command
        return self.client.call_async(prepare)

    def done(self):
        if self.response is not None or self.error is not None:
            return True
        if not self.future.done():
            return False
        try:
            response = self.future.result()
            if not response.accepted or self.stage == 'movement':
                self.response = response
                return True
            if (self.stage == 'prepare' and self.request.permit_command
                    and response.message != 'Turn continuation permit issued'):
                self.response = ClickDrive.Response()
                self.response.message = 'Motion controller cannot confirm turn continuation'
                return True
            if time.monotonic() - self.started > 0.2:
                self.response = ClickDrive.Response()
                self.response.message = 'Movement request timed out before authorization'
                return True
            if self.stage == 'startup_stop':
                self.viewer._motion_session_ready = True
                self.stage = 'prepare'
                self.future = self._prepare()
            else:
                self.request.permit_id = response.permit_id
                self.stage = 'movement'
                self.future = self.client.call_async(self.request)
        except Exception as exc:
            self.error = exc
            return True
        return False

    def result(self):
        if self.error is not None:
            raise self.error
        return self.response


class DualCompressedViewer(Node):

    def __init__(self):
        super().__init__('dual_compressed_viewer')

        # Camera topics on OBot.
        self.declare_parameter(
            'topic_left',
            '/front_camera/image_raw/compressed'
        )
        self.declare_parameter(
            'topic_right',
            '/rear_camera/image_raw/compressed'
        )

        self.declare_parameter('window_left', 'Front Camera')
        self.declare_parameter('window_right', 'Rear Camera')
        self.declare_parameter('show_side_by_side', True)
        self.declare_parameter('combined_window', 'OBot Cameras')
        self.declare_parameter('max_display_hz', 30.0)
        self.declare_parameter('waitkey_ms', 1)

        for name, default in OVERLAY_DEFAULTS.items():
            self.declare_parameter(name, default, _FLEXIBLE)

        self.overlay = OverlayConfig(
            {name: self.get_parameter(name).value
             for name in OVERLAY_DEFAULTS},
            warn=self.get_logger().warn)

        for name, default in TELEMETRY_DEFAULTS.items():
            self.declare_parameter(name, default)
        self.declare_parameter('operator.units', 'm')
        self.operator_units = self.get_parameter('operator.units').value
        if self.operator_units not in ('m', 'ft'):
            raise ValueError('operator.units must be m or ft')
        for name, default in GO_DEFAULTS.items():
            self.declare_parameter(name, default)
        self.target_standoff_m = float(
            self.get_parameter('go.target_standoff_m').value)
        if not math.isfinite(self.target_standoff_m) or self.target_standoff_m < 0:
            raise ValueError('go.target_standoff_m must be finite and nonnegative')
        self.obstacle_min_fraction = float(
            self.get_parameter('obstacle.min_diameter_fraction').value)
        self.obstacle_max_fraction = float(
            self.get_parameter('obstacle.max_diameter_fraction').value)
        if not (math.isfinite(self.obstacle_min_fraction)
                and math.isfinite(self.obstacle_max_fraction)
                and 0 < self.obstacle_min_fraction <= self.obstacle_max_fraction <= 0.05):
            raise ValueError('Obstacle diameter fractions must satisfy 0 < min <= max <= 0.05')
        for name, default in MOTION_DEFAULTS.items():
            self.declare_parameter(name, default)
        self.final_approach_distance_m = float(
            self.get_parameter('motion.final_approach_distance_m').value)
        self.final_approach_speed_mps = float(
            self.get_parameter('motion.final_approach_speed_mps').value)
        if (not math.isfinite(self.final_approach_distance_m)
                or self.final_approach_distance_m < 0):
            raise ValueError('motion.final_approach_distance_m must be finite and nonnegative')
        if (not math.isfinite(self.final_approach_speed_mps)
                or self.final_approach_speed_mps <= 0):
            raise ValueError('motion.final_approach_speed_mps must be finite and positive')
        settings = {name.removeprefix('telemetry.'):
                    self.get_parameter(name).value
                    for name in TELEMETRY_DEFAULTS}
        self.telemetry = None
        self._panel = None
        self._panel_time = 0.0
        self._panel_hits = {}
        self._panel_origin = (0, 0)
        self._source_menu = False
        self._rear_window_ready = False
        for name, default in LIDAR_DEFAULTS.items():
            self.declare_parameter(name, default)
        lidar_settings = {name.removeprefix('lidar.'):
                          self.get_parameter(name).value
                          for name in LIDAR_DEFAULTS}
        self.lidar = None
        if settings['enabled'] or lidar_settings['enabled']:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        if settings['enabled']:
            self._init_telemetry(settings)
        if lidar_settings['enabled']:
            self._init_lidar(lidar_settings)

        self.topic_left = self.get_parameter('topic_left').value
        self.topic_right = self.get_parameter('topic_right').value
        self.window_left = self.get_parameter('window_left').value
        self.window_right = self.get_parameter('window_right').value

        self.show_side_by_side = bool(
            self.get_parameter('show_side_by_side').value
        )

        self.combined_window = self.get_parameter(
            'combined_window'
        ).value

        self.max_display_hz = float(
            self.get_parameter('max_display_hz').value
        )

        self.waitkey_ms = int(
            self.get_parameter('waitkey_ms').value
        )

        if self.waitkey_ms < 1:
            self.waitkey_ms = 1

        # Click-to-bearing: display only, no robot motion.
        self.declare_parameter(
            'click.calibration_file',
            '/home/ros/obot_camera_calibration/front/fisheye_calibration.json'
        )
        self.bearing_camera = FisheyeBearing(
            self.get_parameter('click.calibration_file').value
        )
        self.tracking_worker = TargetTrackingWorker(
            _tracker_factory(), self.bearing_camera.bearing_deg)
        self.selected_target = TargetTrackerState()
        self.click_state = 'idle'  # idle, selected, turning, ready, driving, stopping
        self._click_saw_turn_motion = False
        self._click_saw_drive_motion = False
        self.click_message = ''
        self._go_requested_distance_m = None
        self._go_start_pose = None
        self._pending_go = None
        self._pressed_click = None
        self._latest_odom_pose = None
        self._latest_odom_time = 0.0
        self._latest_scan_time = 0.0
        self._latest_obstacles = ()
        self._bearing_x_lookup = None
        self.click_enable_motion = False
        self.declare_parameter('click.enable_motion', False)
        self.click_enable_motion = bool(self.get_parameter('click.enable_motion').value)
        self._click_client = self.create_client(ClickDrive, '/click_drive')
        for name, default in DIRECTIONAL_DEFAULTS.items():
            self.declare_parameter(name, default)
        self.declare_parameter('directional.enable_motion', True)
        self.directional_enable_motion = bool(
            self.get_parameter('directional.enable_motion').value)
        self.directional_settings = {
            name.removeprefix('directional.'): float(self.get_parameter(name).value)
            for name in DIRECTIONAL_DEFAULTS
        }
        d = self.directional_settings
        validate_directional_settings(d)
        for name, default in SPEED_PRESET_DEFAULTS.items():
            self.declare_parameter(name, default)
        self.speed_presets = {
            name.split('.')[1].removesuffix('_mps').upper():
            float(self.get_parameter(name).value)
            for name in SPEED_PRESET_DEFAULTS
        }
        if not all(math.isfinite(speed) and speed > 0
                   for speed in self.speed_presets.values()):
            raise ValueError('Speed presets must be positive and finite')
        self.selected_speed_preset = 'SLOW'
        self.directional_speed = min(d['max_speed_mps'], max(
            d['min_speed_mps'], self.speed_presets['SLOW']))
        self._viewer_id = uuid.uuid4().hex
        self._viewer_sequence = 0
        self._motion_result_id = None
        self._active_go = None
        self._motion_session_ready = False
        self._manual_client = self.create_client(ClickDrive, '/click_drive')
        self._manual_button = None
        self._manual_future = None
        self._manual_kind = None
        self._manual_future_started = 0.0
        self._manual_owned = False
        self._manual_last_send = 0.0
        self._directional_hits = {}
        self._click_future = None
        self._click_request_kind = None
        self._click_last_status = 0.0
        self._click_turn_started = 0.0
        self._click_button_hits = {}
        self._fixed_go_hits = {}
        self._spray_panel = None
        self._spray_hits = {}
        self._spray_down = None
        try:
            self.spray = SprayControl(self)
        except ModuleNotFoundError as exc:
            if exc.name != 'obot_valve_interfaces':
                raise
            self.spray = None
            self.get_logger().error(
                'Spray controls unavailable: source the robot workspace '
                'containing obot_valve_interfaces before starting the viewer')
        self.front_display_size = None
        self.front_native_size = None
        self._control_bottom = 0

        # Latest decoded camera images.
        self.last_left = None
        self.last_right = None

        self.decode_fail_left = 0
        self.decode_fail_right = 0

        self.left_msg_count = 0
        self.right_msg_count = 0

        self.last_draw_time = 0.0

        # Protect camera images because ROS callbacks execute in another
        # thread while the OpenCV GUI remains in the main thread.
        self._image_lock = threading.Lock()

        # The OpenCV window is deliberately created here.
        #
        # Creating it before the first imshow() allows us to detect when
        # the user closes it. Otherwise imshow() simply recreates a
        # closed window on the next frame.
        cv2.namedWindow(
            self.combined_window,
            cv2.WINDOW_NORMAL
        )
        cv2.setMouseCallback(self.combined_window, self._on_mouse)
        app = QApplication.instance()
        self._motion_window_events = None
        if app is None:
            self.directional_enable_motion = False
            self.get_logger().warning(
                'Directional motion disabled: Qt window events unavailable')
        else:
            self._motion_window_events = MotionWindowEvents(self)
            app.installEventFilter(self._motion_window_events)

        # resizeWindow() will be called once after the first real
        # composite image is available. This restores the natural
        # initial image size while still allowing the window to be
        # resized or maximized afterward.
        self._window_sized = False
        self._window_maximize_attempted = False

        self.sub_left = self.create_subscription(
            CompressedImage,
            self.topic_left,
            self.on_left,
            qos_profile_sensor_data
        )

        self.sub_right = self.create_subscription(
            CompressedImage,
            self.topic_right,
            self.on_right,
            qos_profile_sensor_data
        )

        self.get_logger().info(
            f'Subscribing front: {self.topic_left}'
        )
        self.get_logger().info(
            f'Subscribing rear : {self.topic_right}'
        )
        self.get_logger().info(
            "Press 'q' or close the window to quit."
        )

    def _init_telemetry(self, settings):
        timeout = float(settings['stale_timeout_s'])
        refresh = float(settings['refresh_hz'])
        source = settings['default_position_source'].lower()
        if (not math.isfinite(timeout) or timeout <= 0
                or not math.isfinite(refresh) or refresh <= 0
                or source not in ('odom', 'map')):
            raise ValueError('Invalid telemetry timeout, refresh rate, or source')
        self.telemetry = Telemetry(timeout, source)
        self._clear_lock = threading.Lock()
        self._clear_future = None
        self._clear_started = 0.0
        self._clear_client = self.create_client(Trigger, '/obot/clear_path')
        self.create_timer(0.25, self._check_clear_timeout)
        self.telemetry_settings = settings
        self._panel_period = 1.0 / refresh
        self._missing_joints = set()
        self._tf_seen = {}
        self.telemetry_subscriptions = [
            self.create_subscription(TwistStamped, settings['command_topic'],
                                     self._on_command, 10),
            self.create_subscription(Odometry, settings['odom_topic'],
                                     self._on_odom, 10),
            self.create_subscription(JointState, settings['joint_states_topic'],
                                     self._on_joints, qos_profile_sensor_data),
            self.create_subscription(Float32, settings['left_voltage_topic'],
                                     lambda msg: self._on_voltage('left', msg), 10),
            self.create_subscription(Float32, settings['right_voltage_topic'],
                                     lambda msg: self._on_voltage('right', msg), 10),
            self.create_subscription(String, settings['health_topic'],
                                     self._on_health, 10),
        ]
        self.create_timer(1.0 / refresh, self._refresh_poses)

    def _init_lidar(self, settings):
        self.lidar_config = LidarConfig(**{
            name: settings[name] for name in LidarConfig.__dataclass_fields__
        })
        if not isinstance(settings['topic'], str) or not settings['topic']:
            raise ValueError('LiDAR topic must not be empty')
        self.lidar = LidarState(self.lidar_config)
        self._lidar_last_error = None
        self.lidar_subscription = self.create_subscription(
            LaserScan, settings['topic'], self._on_scan, qos_profile_sensor_data)
        self.get_logger().info(f"Subscribing LiDAR: {settings['topic']}")

    def _on_scan(self, msg):
        received = time.monotonic()
        try:
            stamp = Time.from_msg(msg.header.stamp)
            if not msg.header.frame_id or stamp.nanoseconds <= 0:
                raise ValueError('Scan has no frame ID or timestamp')
            age = (self.get_clock().now().nanoseconds - stamp.nanoseconds) * 1e-9
            if age > self.lidar_config.stale_timeout_s:
                self.lidar.unavailable('stale')
                if self._lidar_last_error != 'stale':
                    self.get_logger().warning(f'LiDAR scan is stale (age {age:.3f}s)')
                    self._lidar_last_error = 'stale'
                return
            if age < 0:
                raise ValueError(f'Scan timestamp is in the future (age {age:.3f}s)')
            transform = self.tf_buffer.lookup_transform(
                'base_link', msg.header.frame_id, stamp)
            points, coverage_uncertain = scan_points(msg, transform.transform)
            regions = corridor_distances(points, self.lidar_config)
            reading = LidarReading(
                center_distance(points, self.lidar_config),
                regions.get('LEFT'),
                regions.get('RIGHT'),
                regions.get('CENTER'),
                coverage_uncertain)
            self.lidar.put(reading, received - age)
            self._latest_scan_time = received - age
            self._latest_obstacles = corridor_obstacle_blobs(
                points, self.lidar_config,
                self.lidar_config.warning_distance_m,
                self.lidar_config.danger_distance_m)
            if self._lidar_last_error is not None:
                self.get_logger().info('LiDAR scan processing recovered')
                self._lidar_last_error = None
        except (TransformException, ValueError) as exc:
            status = 'TF unavailable' if isinstance(exc, TransformException) else 'unavailable'
            self.lidar.unavailable(status)
            self._latest_obstacles = ()
            if self._lidar_last_error != status:
                self.get_logger().warning(f'LiDAR {status}: {exc}')
                self._lidar_last_error = status

    def _on_command(self, msg):
        values = (msg.twist.linear.x, msg.twist.angular.z)
        self.telemetry.put('command', values if all(map(math.isfinite, values)) else None)

    def _on_odom(self, msg):
        values = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        self.telemetry.put('actual', values if all(map(math.isfinite, values)) else None)
        position = msg.pose.pose.position
        try:
            yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        except ValueError:
            return
        pose = (position.x, position.y, yaw)
        if all(math.isfinite(value) for value in pose):
            self._latest_odom_pose = pose
            self._latest_odom_time = time.monotonic()

    def _on_joints(self, msg):
        wheels = wheel_speeds(msg, self.telemetry_settings['left_joint'],
                              self.telemetry_settings['right_joint'])
        missing = {side for side, value in wheels.items() if value is None}
        if missing != self._missing_joints and missing:
            self.get_logger().warning(
                'Missing or invalid wheel joint velocities: ' + ', '.join(sorted(missing)))
        self._missing_joints = missing
        self.telemetry.put('wheels', wheels)

    def _on_voltage(self, side, msg):
        self.telemetry.put('voltage_' + side,
                           msg.data if math.isfinite(msg.data) else None)

    def _on_health(self, msg):
        self.telemetry.put('health', msg.data if msg.data in (
            'NORMAL', 'RECOVERING', 'FAULT', 'NO_DATA') else None)

    def _clear_path(self):
        with self._clear_lock:
            if self._clear_future is not None:
                self.telemetry.show_notice('Clear pending')
                return
            if not self._clear_client.service_is_ready():
                self.telemetry.show_notice('Path unavailable')
                return
            try:
                future = self._clear_client.call_async(Trigger.Request())
            except RuntimeError as exc:
                self.get_logger().warning(f'Clear path request failed: {exc}')
                self.telemetry.show_notice('Clear failed')
                return
            self._clear_future = future
            self._clear_started = time.monotonic()
        future.add_done_callback(self._clear_done)

    def _clear_done(self, future):
        with self._clear_lock:
            if future is not self._clear_future:
                return
            self._clear_future = None
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().warning(f'Clear path request failed: {exc}')
            self.telemetry.show_notice('Clear failed')
            return
        self.telemetry.show_notice(
            'Path cleared' if result is not None and result.success
            else 'Clear failed')

    def _check_clear_timeout(self):
        with self._clear_lock:
            if (self._clear_future is None
                    or time.monotonic() - self._clear_started < 5.0):
                return
            future = self._clear_future
            self._clear_future = None
        self._clear_client.remove_pending_request(future)
        self.telemetry.show_notice('Clear timeout')

    def _refresh_poses(self):
        for source in ('odom', 'map'):
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.telemetry_settings[source + '_frame'],
                    self.telemetry_settings['robot_frame'], Time())
                stamp = (transform.header.stamp.sec
                         + transform.header.stamp.nanosec * 1e-9)
                now_ros = self.get_clock().now().nanoseconds * 1e-9
                now = time.monotonic()
                old_stamp, first_seen = self._tf_seen.get(source, (None, now))
                if stamp != old_stamp:
                    first_seen = now
                self._tf_seen[source] = (stamp, first_seen)
                if (stamp <= 0 or not 0 <= now_ros - stamp <= self.telemetry.timeout
                        or now - first_seen > self.telemetry.timeout):
                    self.telemetry.put('pose_' + source, None)
                    continue
                trans = transform.transform.translation
                pose = (trans.x, trans.y,
                        yaw_from_quaternion(transform.transform.rotation))
                if not all(map(math.isfinite, pose)):
                    raise ValueError('Non-finite transform')
                self.telemetry.put('pose_' + source, pose)
            except (TransformException, ValueError):
                self.telemetry.put('pose_' + source, None)

    def _fresh_center_distance(self):
        if self.lidar is None:
            return None
        snapshot = self.lidar.snapshot()
        if snapshot.status != 'ok':
            return None
        front = snapshot.reading.front
        if front.distance_m is None or front.held:
            return None
        return front.distance_m

    def _go_distance_from_lidar(self):
        center = self._fresh_center_distance()
        if center is None:
            return None
        distance = center - self.target_standoff_m
        if distance <= 0.0:
            return None
        return distance

    def _drive_progress_m(self):
        if self._go_start_pose is None or self._latest_odom_pose is None:
            return None
        if time.monotonic() - self._latest_odom_time > 0.6:
            return None
        sx, sy, syaw = self._go_start_pose
        ox, oy, _ = self._latest_odom_pose
        dx, dy = ox - sx, oy - sy
        return max(0.0, dx * math.cos(syaw) + dy * math.sin(syaw))

    def _go_remaining_m(self):
        if self.click_state != 'driving' or self._go_requested_distance_m is None:
            return None
        progress = self._drive_progress_m()
        if progress is None:
            return None
        return max(0.0, self._go_requested_distance_m - progress)

    def _go_button_text(self):
        distance = self._go_distance_from_lidar()
        if distance is None:
            return 'GO --'
        return f'GO {display_linear(distance, self.operator_units):.1f} {self.operator_units}'

    def _build_bearing_lookup(self):
        if self.front_native_size is None:
            return
        width, height = self.front_native_size
        if width <= 2 or self.bearing_camera.width <= 2:
            return
        samples = []
        row = min(height - 1, max(0, height // 2))
        for x in range(0, width, max(1, width // 80)):
            try:
                bearing = self.bearing_camera.bearing_deg(x, row)
            except ValueError:
                continue
            samples.append((bearing, x))
        samples.sort(key=lambda item: item[0])
        if len(samples) >= 2:
            self._bearing_x_lookup = samples

    def _bearing_to_pixel(self, bearing_deg):
        table = self._bearing_x_lookup
        if table is None or len(table) < 2:
            return None
        if bearing_deg < table[0][0] or bearing_deg > table[-1][0]:
            return None
        for (left_bearing, left_x), (right_bearing, right_x) in zip(table, table[1:]):
            if left_bearing <= bearing_deg <= right_bearing:
                if right_bearing == left_bearing:
                    return int(round((left_x + right_x) / 2))
                ratio = (bearing_deg - left_bearing) / (right_bearing - left_bearing)
                return int(round(left_x + ratio * (right_x - left_x)))
        return None

    def _update_target_tracking(self, _frame=None):
        state = self.tracking_worker.snapshot()
        if state.status == 'lost' and self.selected_target.status != 'lost':
            self.click_message = 'Target tracking lost'
        self.selected_target = state

    def _draw_obstacle_indicators(self, image):
        if not self._latest_obstacles:
            return
        if time.monotonic() - self._latest_scan_time > self.lidar_config.stale_timeout_s:
            return
        if self._bearing_x_lookup is None:
            self._build_bearing_lookup()
        for blob in self._latest_obstacles:
            x = self._bearing_to_pixel(math.degrees(blob.bearing_rad))
            if x is None:
                continue
            width = image.shape[1]
            fraction = max(0.0, min(1.0, 1.0 - blob.nearest_m /
                                    self.lidar_config.warning_distance_m))
            diameter = width * (self.obstacle_min_fraction + fraction * (
                self.obstacle_max_fraction - self.obstacle_min_fraction))
            radius = max(1, int((diameter - 2) / 2))
            y = max(radius + 3, 55)
            color = (0, 0, 255) if blob.severity == 'red' else (0, 255, 255)
            cv2.circle(image, (x, y), radius, color, 2)
            distance = display_linear(blob.nearest_m, self.operator_units)
            label = f'{distance:.1f} {self.operator_units}'
            size, baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
            lx = min(max(0, x + radius + 4), max(0, width - size[0] - 1))
            ly = min(image.shape[0] - baseline - 1, y + size[1] // 2)
            cv2.putText(image, label, (lx, ly), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, color, 1, cv2.LINE_AA)

    def _click_request(self, command, angle=0.0, distance=0.0, continuation_of=''):
        """Nonblocking ROS service call; results processed in draw()."""
        if self._click_future is not None:
            return False
        if not self._click_client.service_is_ready():
            self.click_message = 'ClickDrive service unavailable'
            return False
        request = self._viewer_request(command)
        request.angle_deg = float(angle)
        request.distance_m = float(distance)
        request.permit_command = continuation_of
        if command in ('TURN', 'DRIVE'):
            self._click_future = MovementRequest(self, self._click_client, request)
            self._motion_result_id = f'{request.client_id}:{request.sequence}'
        else:
            self._click_future = self._click_client.call_async(request)
        self._click_request_kind = command
        return True

    def _viewer_request(self, command):
        request = ClickDrive.Request()
        self._viewer_sequence += 1
        request.client_id = self._viewer_id
        request.sequence = self._viewer_sequence
        request.command = command
        request.linear_speed_mps, request.angular_speed_rps = directional_speeds(
            self.directional_speed, self.directional_settings)
        return request

    def _manual_send(self, command):
        if not self._manual_client.service_is_ready():
            self.click_message = 'Motion service unavailable'
            return False
        request = self._viewer_request(command)
        if command == 'MANUAL':
            request.linear_direction, request.angular_direction = {
                'FORWARD': (1, 0), 'REVERSE': (-1, 0),
                'LEFT': (0, 1), 'RIGHT': (0, -1),
            }[self._manual_button]
        if command in ('MANUAL_BEGIN', 'MANUAL'):
            future = MovementRequest(self, self._manual_client, request)
        else:
            future = self._manual_client.call_async(request)
        if command != 'MANUAL_END':
            self._manual_future = future
            self._manual_kind = command
            self._manual_future_started = time.monotonic()
        return True

    def _manual_stop(self):
        if self._manual_button is not None or self._manual_owned:
            self._manual_button = None
            self._manual_owned = False
            self._manual_send('MANUAL_END')
        self._manual_future = None
        self._manual_kind = None

    def _clear_selected_target(self):
        self.tracking_worker.clear()
        self.selected_target = TargetTrackerState()

    def _manual_press(self, name):
        self._manual_stop()
        self._pending_go = None
        self._click_future = None
        self._click_request_kind = None
        self._clear_selected_target()
        self.click_state = 'idle'
        self._go_requested_distance_m = None
        self._go_start_pose = None
        self.click_message = 'Manual takeover requested'
        if not self.directional_enable_motion:
            self.click_message = 'TEST MODE: motion disabled'
            return
        self._manual_button = name
        if not self._manual_send('MANUAL_BEGIN'):
            self._manual_button = None

    def _manual_tick(self):
        if self._manual_button is None:
            return
        window = self.combined_window if self.show_side_by_side else self.window_left
        if not window_has_focus(window):
            self.click_message = 'Manual stopped: window not focused'
            self._manual_stop()
            return
        now = time.monotonic()
        if self._manual_future is not None:
            if self._manual_future.done():
                future, kind = self._manual_future, self._manual_kind
                self._manual_future = None
                try:
                    response = future.result()
                except Exception as exc:
                    self.click_message = f'Manual service error: {exc}'
                    self._manual_stop()
                    return
                if not response.accepted:
                    self._pending_go = None
                    self.click_message = f'Manual rejected: {response.message}'
                    self._manual_stop()
                    return
                if kind == 'MANUAL_BEGIN':
                    self._manual_owned = True
                    self.click_message = 'Manual control'
            elif now - self._manual_future_started > 0.2:
                self.click_message = 'Manual service timeout'
                self._manual_stop()
                return
        if (self._manual_owned and self._manual_future is None
                and now - self._manual_last_send >= 0.08):
            if self._manual_send('MANUAL'):
                self._manual_last_send = now
            else:
                self._manual_stop()

    def _click_turn(self):
        if self._manual_button is not None:
            return
        if self.click_state != 'selected' or self.selected_target.status == 'idle':
            return
        if not self.click_enable_motion:
            self.click_message = 'CLICK MOTION DISABLED'
            return
        if self.selected_target.status not in ('selected', 'tracking'):
            self.click_message = 'Target unavailable or tracking lost'
            return
        if abs(self.selected_target.bearing_deg) < 0.1:
            self._clear_selected_target()
            self.click_state = 'ready'
            self.click_message = 'Turn complete'
            return
        if self._click_request('TURN', self.selected_target.bearing_deg):
            self._pending_go = None
            self.click_state = 'turning'
            self._click_turn_started = time.monotonic()
            self._click_saw_turn_motion = False
            self.click_message = 'TURN requested...'

    def _click_cancel(self):
        self._manual_stop()
        self._pending_go = None
        self._clear_selected_target()
        self._click_future = None
        self._click_request_kind = None
        self.click_state = 'stopping'
        self.click_message = 'STOP pending...'
        self._click_request('STOP')

    def _universal_stop(self):
        """Request motion and spray cancellation without waiting for the GUI."""
        self._manual_stop()
        self._pending_go = None
        self._clear_selected_target()
        self._click_future = None
        self._click_request_kind = None
        self.click_state = 'stopping'
        self._go_requested_distance_m = None
        self._go_start_pose = None
        self.click_message = 'STOP pending...'
        self._click_request('STOP')
        if self.spray is not None:
            self.spray.cancel_all()

    def _request_drive(self, distance, continuation_of=''):
        if self._manual_button is not None:
            return
        if self.click_state not in ('idle', 'selected', 'ready') or self._click_future is not None:
            return
        if not math.isfinite(distance) or distance < 0.1:
            self.click_message = 'Distance must be finite and at least 0.1 m'
            return
        if not self.click_enable_motion:
            self.click_message = 'CLICK MOTION DISABLED'
            return
        if self._click_request('DRIVE', distance=distance, continuation_of=continuation_of):
            self.click_state = 'driving'
            self._click_saw_drive_motion = False
            self._go_requested_distance_m = distance
            self._go_start_pose = self._latest_odom_pose
            self.click_message = (
                f'DRIVE {display_linear(distance, self.operator_units):.2f}'
                f' {self.operator_units} requested...')

    def _begin_go(self, distance):
        if (self._manual_button is not None
                or self.click_state not in ('idle', 'selected', 'ready')):
            return
        if not self.click_enable_motion:
            self.click_message = 'CLICK MOTION DISABLED'
            return
        target = self.selected_target
        if self.click_state == 'selected' and target.status not in ('selected', 'tracking'):
            self.click_message = 'Target unavailable or tracking lost'
            return
        if self.click_state == 'selected' and abs(target.bearing_deg) >= 0.1:
            self._pending_go = ('lidar', None) if distance is None else ('fixed', distance)
            if self._click_request('TURN', target.bearing_deg):
                self.click_state = 'turning'
                self._click_saw_turn_motion = False
                self.click_message = 'Turning before drive'
            else:
                self._pending_go = None
            return
        self._start_go_drive(distance)

    def _start_go_drive(self, distance, continuation_of=''):
        if distance is None:
            distance = self._go_distance_from_lidar()
            if distance is None:
                self.click_message = 'GO unavailable: center LiDAR invalid or stale'
                return
        self._request_drive(distance, continuation_of=continuation_of)

    def _click_go(self):
        was_ready = self.click_state in ('idle', 'selected', 'ready')
        self._begin_go(None)
        if was_ready and self.click_state in ('turning', 'driving'):
            self._active_go = 'GO'

    def _click_go_fixed(self, distance_units):
        try:
            distance = meters(float(distance_units), self.operator_units)
        except ValueError:
            self.click_message = 'Invalid fixed distance'
            return
        was_ready = self.click_state in ('idle', 'selected', 'ready')
        self._begin_go(distance)
        if was_ready and self.click_state in ('turning', 'driving'):
            self._active_go = str(int(distance_units))

    def _click_poll(self):
        """Poll the GUI-thread request without blocking camera drawing."""
        future = self._click_future
        if future is not None and future.done():
            kind = self._click_request_kind
            self._click_future = None
            self._click_request_kind = None
            try:
                response = future.result()
                if not response.accepted:
                    if kind == 'TURN':
                        self._pending_go = None
                    self.click_message = f'{kind} rejected: {response.message}'
                    if kind == 'TURN' and self.click_state == 'turning':
                        self.click_state = 'selected'
                    elif kind == 'DRIVE' and self.click_state == 'driving':
                        self.click_state = 'ready'
                elif kind in ('TURN', 'DRIVE'):
                    self.click_message = response.message
                    if kind == 'TURN':
                        self._click_saw_turn_motion = True
                    else:
                        self._click_saw_drive_motion = True
                elif kind == 'STOP':
                    self.click_message = 'STOP accepted'
                    self.click_state = 'idle'
                    self._clear_selected_target()
                    self._go_requested_distance_m = None
                    self._go_start_pose = None
                elif kind == 'STATUS' and self.click_state in ('turning', 'driving'):
                    if response.state != 'idle':
                        if self.click_state == 'turning':
                            self._click_saw_turn_motion = True
                        else:
                            self._click_saw_drive_motion = True
                    elif self.click_state == 'turning' and self._click_saw_turn_motion:
                        self.click_state = 'ready'
                        pending, self._pending_go = self._pending_go, None
                        expected = f'result=success, request={self._motion_result_id},'
                        if not response.message.startswith(expected):
                            self.click_message = 'Turn failed or completion unconfirmed'
                        else:
                            self._clear_selected_target()
                            self.click_message = 'Turn complete'
                            if pending is not None:
                                self._start_go_drive(pending[1], self._motion_result_id)
                    elif self.click_state == 'driving' and self._click_saw_drive_motion:
                        self.click_state = 'idle'
                        self._clear_selected_target()
                        self._go_requested_distance_m = None
                        self._go_start_pose = None
                        expected = f'result=success, request={self._motion_result_id},'
                        self.click_message = ('Drive complete' if response.message.startswith(
                            expected) else 'Drive failed or completion unconfirmed')
            except Exception as exc:
                self._pending_go = None
                self.click_message = f'{kind} service error: {exc}'
                if kind in ('TURN', 'DRIVE') and self.click_state in ('turning', 'driving'):
                    self.click_state = 'stopping'
        if self.click_state == 'stopping' and self._click_future is None:
            self._click_request('STOP')
        if (self.click_state in ('turning', 'driving') and self._click_future is None
                and time.monotonic() - self._click_last_status >= 0.35):
            if self._click_request('STATUS'):
                self._click_last_status = time.monotonic()

    def _click_drive_buttons(self, canvas, front_w, front_h):
        """Draw motion controls in the first two panels below the camera."""
        self._click_button_hits = {}
        self._directional_hits = {}
        self._fixed_go_hits = {}
        w = min(front_w, canvas.shape[1])
        top = front_h
        h = canvas.shape[0] - top
        if w < 300 or h < 75:
            return
        first, second = w // 3, 2 * w // 3
        cv2.rectangle(canvas, (0, top), (w - 1, top + h - 1), (30, 30, 30), -1)
        for x in (first, second):
            cv2.line(canvas, (x, top), (x, top + h - 1), (130, 130, 130), 1)
        margin, gap, header, button_h = control_metrics(first, h)
        row_top = top + header
        fit_text(canvas, 'CLICK-TO-DRIVE', (margin, top, first - margin, row_top))
        angle = '--'
        if self.selected_target.status != 'idle':
            angle = f'{self.selected_target.bearing_deg:+.0f} deg'
        left, right = margin, first - margin
        mid = (left + right) // 2
        turn_box = (left, row_top, mid - gap, row_top + button_h)
        stop_box = (mid, row_top, right, row_top + button_h)
        can_turn = (self.selected_target.status in ('selected', 'tracking')
                    and self.click_state == 'selected')
        _push_button(canvas, turn_box, 'TURN', enabled=can_turn or self.click_state == 'turning',
                     active=self.click_state == 'turning', pressed=self._pressed_click == 'TURN')
        _push_button(canvas, stop_box, 'STOP', pressed=self._pressed_click == 'STOP')
        self._click_button_hits['TURN'] = turn_box
        self._click_button_hits['STOP'] = stop_box
        can_go = self.click_state in ('idle', 'selected', 'ready')
        go_enabled = can_go and (self.click_state == 'selected'
                                 or self._go_distance_from_lidar() is not None)
        go_y = row_top + button_h + gap
        go_width = right - left
        usable = go_width - 2 * gap
        fixed_width = 3 * usable // 10
        go_box = (left + 2 * fixed_width + 2 * gap, go_y, right, go_y + button_h)
        active_go = self.click_state in ('turning', 'driving') and self._active_go == 'GO'
        _push_button(canvas, go_box, self._go_button_text(),
                     enabled=go_enabled or active_go, primary=True,
                     active=active_go, pressed=self._pressed_click == 'GO')
        self._click_button_hits['GO'] = go_box
        for index, (label, amount) in enumerate((('GO-2', 2), ('GO-4', 4))):
            bx0 = left + index * (fixed_width + gap)
            bx1 = bx0 + fixed_width
            box = (bx0, go_y, bx1, go_y + button_h)
            active = self.click_state in ('turning', 'driving') and self._active_go == str(amount)
            _push_button(canvas, box, label, enabled=can_go or active, active=active,
                         pressed=self._pressed_click == str(amount))
            self._fixed_go_hits[str(amount)] = box
        info_y = go_y + button_h + gap
        info_bottom = top + h - margin
        info_h = max(1, min(20, (info_bottom - info_y) // 4))
        cv2.rectangle(canvas, (left, info_y),
                      (right - 1, min(info_bottom, info_y + 4 * info_h) - 1),
                      (90, 90, 90), 1)
        tracking = ' TRACK LOST' if self.selected_target.status == 'lost' else ''
        direction = (' RIGHT' if self.selected_target.bearing_deg > 0 else ' LEFT'
                     ) if self.selected_target.status != 'idle' else ''
        fit_text(canvas, 'Target: ' + angle + direction + tracking,
                 (left, info_y, right, info_y + info_h), scale=0.55)
        remaining = self._go_remaining_m() if self.click_state == 'driving' else None
        message = (f'Remaining {display_linear(remaining, self.operator_units):.1f} '
                   f'{self.operator_units}' if remaining is not None else self.click_message)
        words = message.split()
        lines = ['']
        for word in words:
            candidate = (lines[-1] + ' ' + word).strip()
            width = cv2.getTextSize(candidate, cv2.FONT_HERSHEY_SIMPLEX, 0.48, 1)[0][0]
            if width > right - left - 8 and lines[-1] and len(lines) < 3:
                lines.append(word)
            else:
                lines[-1] = candidate
        for index, line in enumerate(lines):
            y0 = info_y + (index + 1) * info_h
            y1 = min(info_bottom, y0 + info_h)
            fit_text(canvas, line, (left, y0, right, y1), (0, 255, 255), scale=0.48)

        actual, _ = directional_speeds(self.directional_speed, self.directional_settings)
        speed_label = f'{display_linear(actual, self.operator_units):.2f} {self.operator_units}/s'
        fit_text(canvas, f'DIRECTIONAL  {speed_label}',
                 (first + margin, top, second - margin, row_top))
        dleft, dright = first + margin, second - margin
        dw = dright - dleft
        for key, col, row, vx, vy in (
                ('FORWARD', 1, 0, 0, -1), ('LEFT', 0, 1, -1, 0),
                ('RIGHT', 2, 1, 1, 0), ('REVERSE', 1, 2, 0, 1)):
            bx0 = dleft + col * dw // 3
            bx1 = dleft + (col + 1) * dw // 3 - gap
            by = row_top + row * (button_h + gap)
            box = (bx0, by, bx1, by + button_h)
            _push_button(canvas, box, '', enabled=True)
            self._directional_hits[key] = box
            cx, cy = (box[0] + box[2]) // 2, (box[1] + box[3]) // 2
            length = min(12, button_h // 3, (bx1 - bx0) // 3)
            cv2.arrowedLine(canvas, (cx - length * vx, cy - length * vy),
                            (cx + length * vx, cy + length * vy),
                            (240, 240, 240), 3, cv2.LINE_AA, tipLength=0.45)
        speed_top = row_top + 3 * (button_h + gap)
        for index, name in enumerate(('SLOW', 'MEDIUM', 'FAST')):
            bx0 = dleft + index * dw // 3
            bx1 = dleft + (index + 1) * dw // 3 - (gap if index < 2 else 0)
            box = (bx0, speed_top, bx1, speed_top + button_h)
            _push_button(canvas, box, name, enabled=True,
                         active=(name == self.selected_speed_preset))
            self._directional_hits[name] = box

    def _click_handle_button(self, x, y):
        for name, (x0, y0, x1, y1) in self._click_button_hits.items():
            if x0 <= x < x1 and y0 <= y < y1:
                if name == 'TURN':
                    self._click_turn()
                elif name == 'GO':
                    self._click_go()
                else:
                    self._click_cancel()
                return True
        for amount, (x0, y0, x1, y1) in self._fixed_go_hits.items():
            if x0 <= x < x1 and y0 <= y < y1:
                self._click_go_fixed(float(amount))
                return True
        return False

    def _select_front_target(self, x, y):
        """Select a point in displayed front-image coordinates."""
        if self._manual_button is not None:
            return
        if self.front_display_size is None or self.front_native_size is None:
            return

        display_w, display_h = self.front_display_size
        native_w, native_h = self.front_native_size

        if not (0 <= x < display_w and 0 <= y < display_h):
            return

        # Map the displayed image back to the camera's native pixels.
        pixel_x = (x + 0.5) * native_w / display_w - 0.5
        pixel_y = (y + 0.5) * native_h / display_h - 0.5
        pixel_x = min(max(pixel_x, 0.0), native_w - 1.0)
        pixel_y = min(max(pixel_y, 0.0), native_h - 1.0)

        try:
            bearing = self.bearing_camera.bearing_deg(pixel_x, pixel_y)
        except ValueError as exc:
            self.get_logger().warning(str(exc))
            return

        if self.click_state in ('turning', 'driving', 'stopping'):
            return
        self.tracking_worker.select(pixel_x, pixel_y, bearing)
        self.selected_target = TargetTrackerState(
            pixel_x=pixel_x, pixel_y=pixel_y,
            bearing_deg=bearing, status='initializing')
        self.click_state = 'selected'
        self.click_message = ('CLICK MOTION DISABLED' if not self.click_enable_motion
                              else 'Select TURN or GO')

        direction = 'RIGHT' if bearing > 0 else 'LEFT'
        self.get_logger().info(
            f'TARGET pixel=({pixel_x:.1f}, {pixel_y:.1f}) '
            f'bearing={bearing:+.2f} deg ({direction})'
        )

    def _handle_motion_mouse(self, event, x, y, flags):
        hits = self._directional_hits
        if event == cv2.EVENT_LBUTTONUP:
            if self._manual_button is not None:
                self._manual_stop()
                return True
        if event == cv2.EVENT_MOUSEMOVE:
            if self._manual_button is not None:
                box = hits.get(self._manual_button)
                if (not flags & cv2.EVENT_FLAG_LBUTTON or box is None
                        or not (box[0] <= x < box[2] and box[1] <= y < box[3])):
                    self._manual_stop()
                return True
        if event == cv2.EVENT_LBUTTONDOWN:
            for name, (x0, y0, x1, y1) in hits.items():
                if x0 <= x < x1 and y0 <= y < y1:
                    if name in self.speed_presets:
                        self.selected_speed_preset = name
                        d = self.directional_settings
                        self.directional_speed = min(
                            d['max_speed_mps'], max(
                                d['min_speed_mps'], self.speed_presets[name]))
                    else:
                        self._manual_press(name)
                    return True
        return False

    def _track_click_press(self, event, x, y):
        if event == cv2.EVENT_LBUTTONDOWN:
            self._pressed_click = next(
                (name for name, box in (*self._click_button_hits.items(),
                                        *self._fixed_go_hits.items())
                 if box[0] <= x < box[2] and box[1] <= y < box[3]), None)
        elif event in (cv2.EVENT_LBUTTONUP, cv2.EVENT_MOUSEMOVE):
            box = (self._click_button_hits.get(self._pressed_click)
                   or self._fixed_go_hits.get(self._pressed_click))
            if (event == cv2.EVENT_LBUTTONUP or box is None
                    or not (box[0] <= x < box[2] and box[1] <= y < box[3])):
                self._pressed_click = None

    def _handle_spray_mouse(self, event, x, y):
        panel = self._spray_panel
        if panel is None:
            return False
        inside = panel[0] <= x < panel[2] and panel[1] <= y < panel[3]
        if event == cv2.EVENT_LBUTTONDOWN:
            self._spray_down = next(
                (name for name, box in self._spray_hits.items()
                 if box[0] <= x < box[2] and box[1] <= y < box[3]), None)
            return inside
        if event == cv2.EVENT_LBUTTONUP:
            name, self._spray_down = self._spray_down, None
            box = self._spray_hits.get(name)
            if (self.spray is not None and box is not None
                    and box[0] <= x < box[2] and box[1] <= y < box[3]):
                if name == 'cancel':
                    self.spray.cancel()
                else:
                    if self.spray.mode_enabled(name):
                        self.spray.start({'circle': 0, 'square': 1, 'triangle': 2}[name])
            return inside or name is not None
        return inside

    def _draw_spray(self, canvas):
        snapshot = (self.spray.snapshot() if self.spray is not None else None)
        if snapshot is None:
            snapshot = SpraySnapshot(
                state='UNKNOWN', active=False, remaining=None,
                message='DEPENDENCY MISSING',
                durations={'circle': None, 'square': None, 'triangle': None})
        self._spray_panel, self._spray_hits = draw_spray_panel(
            canvas, (2 * self.front_display_size[0] // 3, self.front_display_size[1],
                     self.front_display_size[0], canvas.shape[0]),
            snapshot, available=self.spray is not None)

    def _on_front_mouse(self, event, x, y, _flags, _userdata):
        """Mouse callback for the separate front-camera window."""
        self._track_click_press(event, x, y)
        if self._handle_motion_mouse(event, x, y, _flags):
            return
        if self._handle_spray_mouse(event, x, y):
            return
        if event == cv2.EVENT_LBUTTONUP:
            if self.front_display_size and y >= self.front_display_size[1]:
                self._click_handle_button(x, y)
                return
            if not self._click_handle_button(x, y):
                self._select_front_target(x, y)

    def _on_mouse(self, event, x, y, _flags, _userdata):
        self._track_click_press(event, x, y)
        if self._handle_motion_mouse(event, x, y, _flags):
            return
        if self._handle_spray_mouse(event, x, y):
            return
        if event != cv2.EVENT_LBUTTONUP:
            return

        # Front-camera clicks select targets.
        if self.front_display_size is not None:
            fw, fh = self.front_display_size
            if 0 <= x < fw and fh <= y < self._control_bottom:
                self._click_handle_button(x, y)
                return
            if 0 <= x < fw and 0 <= y < fh:
                if not self._click_handle_button(x, y):
                    self._select_front_target(x, y)
                return

        self._on_dashboard_mouse(x, y)

    def _on_rear_mouse(self, event, x, y, _flags, _userdata):
        if event == cv2.EVENT_LBUTTONUP:
            self._on_dashboard_mouse(x, y)

    def _on_dashboard_mouse(self, x, y):
        # Existing dashboard controls remain unchanged.
        if not self.telemetry:
            return

        px, py = self._panel_origin
        x, y = x - px, y - py
        for action, (x0, y0, x1, y1) in self._panel_hits.items():
            if x0 <= x < x1 and y0 <= y < y1:
                if action == 'source':
                    self._source_menu = not self._source_menu
                elif action in ('odom', 'map'):
                    self.telemetry.select(action)
                    self._source_menu = False
                elif action == 'reset':
                    self.telemetry.reset()
                elif action == 'clear':
                    self._clear_path()
                self._panel_time = 0.0
                return

    def _draw_dashboard(self, canvas, x, y):
        panel_height = canvas.shape[0] - y
        panel_width = canvas.shape[1] - x
        if panel_height <= 0 or panel_width <= 0:
            return
        now = time.monotonic()
        if (self._panel is None
                or self._panel.shape[:2] != (panel_height, panel_width)
                or now - self._panel_time >= self._panel_period):
            data = self.telemetry.snapshot()
            data['source_menu'] = self._source_menu
            data['units'] = self.operator_units
            self._panel, self._panel_hits = render_dashboard(
                panel_width, panel_height, data)
            self._panel_time = now
        canvas[y:, x:] = self._panel
        self._panel_origin = (x, y)

    @staticmethod
    def _decode(msg: CompressedImage):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        return cv2.imdecode(buf, cv2.IMREAD_COLOR)

    def on_left(self, msg: CompressedImage):
        img = self._decode(msg)

        if img is None:
            self.decode_fail_left += 1
            self.get_logger().warn(
                'Failed to decode FRONT image'
            )
            return

        with self._image_lock:
            self.last_left = img
        self.tracking_worker.submit_frame(img)

        self.left_msg_count += 1

    def on_right(self, msg: CompressedImage):
        img = self._decode(msg)

        if img is None:
            self.decode_fail_right += 1
            self.get_logger().warn(
                'Failed to decode REAR image'
            )
            return

        with self._image_lock:
            self.last_right = img

        self.right_msg_count += 1

    def draw(self):
        self._manual_tick()
        now = time.time()

        if self.max_display_hz > 0.0:
            period = 1.0 / self.max_display_hz

            if (now - self.last_draw_time) < period:
                return

        self.last_draw_time = now

        with self._image_lock:
            left = (
                None
                if self.last_left is None
                else self.last_left.copy()
            )

            right = (
                None
                if self.last_right is None
                else self.last_right.copy()
            )

        # Wait for at least one camera before displaying anything.
        if left is None and right is None:
            return

        if left is None:
            left = np.zeros(
                (240, 320, 3),
                dtype=np.uint8
            )

            cv2.putText(
                left,
                'NO FRONT IMAGE',
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )

        if right is None:
            right = np.zeros(
                (240, 320, 3),
                dtype=np.uint8
            )

            cv2.putText(
                right,
                'NO REAR IMAGE',
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )

        if self.lidar:
            draw_lidar(left, self.lidar.snapshot(),
                       top=40 if self.selected_target.status != 'idle' else 4,
                       units=self.operator_units)
            self._draw_obstacle_indicators(left)
        draw_markers(left, self.overlay)

        self._update_target_tracking(left)
        if self.selected_target.status != 'idle':
            tx, ty = self.selected_target.pixel_x, self.selected_target.pixel_y
            cx, cy = round(tx), round(ty)
            if self.selected_target.status != 'lost':
                cv2.drawMarker(
                    left, (cx, cy), (0, 255, 255),
                    cv2.MARKER_CROSS, 24, 3
                )

        self.front_native_size = (left.shape[1], left.shape[0])
        if self._bearing_x_lookup is None:
            self._build_bearing_lookup()

        if not self.show_side_by_side:
            right, box_w, box_h = scale_rear(
                right, left.shape[1], left.shape[0],
                self.overlay.rear_scale)
            if self.telemetry:
                right_canvas = np.zeros(
                    (max(left.shape[0], box_h), box_w, 3), dtype=np.uint8)
                right_canvas[:right.shape[0], :right.shape[1]] = right
                if not self._rear_window_ready:
                    cv2.namedWindow(self.window_right, cv2.WINDOW_NORMAL)
                    cv2.setMouseCallback(self.window_right, self._on_rear_mouse)
                    self._rear_window_ready = True
                right_canvas = display_sized_image(right_canvas, self.window_right)
                panel_y = round(box_h * right_canvas.shape[0] / max(left.shape[0], box_h))
                self._draw_dashboard(right_canvas, 0, panel_y)
                right = right_canvas

            self.front_display_size = (left.shape[1], left.shape[0])
            left = np.pad(left, ((0, max(1, round(left.shape[0] / 3))), (0, 0), (0, 0)))
            self._control_bottom = left.shape[0]
            cv2.setMouseCallback(
                self.window_left, self._on_front_mouse
            )
            self._click_poll()
            self._click_drive_buttons(left, *self.front_display_size)
            self._draw_spray(left)
            cv2.imshow(
                self.window_left,
                left
            )

            cv2.imshow(
                self.window_right,
                right
            )

            key = cv2.waitKey(
                self.waitkey_ms
            ) & 0xFF

            if key == ord('q'):
                self._manual_stop()
                rclpy.shutdown()

            return

        combo = compose(left, right, 0.5)
        combo = np.pad(combo, ((0, max(1, round(left.shape[0] / 3))), (0, 0), (0, 0)))
        # IMPORTANT:
        # Check whether the window was closed BEFORE imshow().
        # Calling imshow() first would recreate a closed window.
        if cv2.getWindowProperty(
            self.combined_window,
            cv2.WND_PROP_VISIBLE
        ) < 1:
            self._manual_stop()
            rclpy.shutdown()
            return

        # The window was created during __init__, before we knew the
        # dimensions of the incoming images. Size it exactly once when
        # the first composite image is ready.
        if not self._window_sized:

            cv2.resizeWindow(
                self.combined_window,
                combo.shape[1],
                combo.shape[0]
            )

            self._window_sized = True

        original_combo_h = combo.shape[0]

        combo = display_sized_image(combo, self.combined_window)
        if self.telemetry:
            fh, fw = left.shape[:2]
            panel_x = round(fw * combo.shape[0] / original_combo_h)
            panel_top = round(fh * 0.5 * combo.shape[0] / original_combo_h)
            self._draw_dashboard(combo, panel_x, panel_top)

        # OpenCV mouse coordinates refer to the displayed image.
        # Convert the front-camera region using the composite scale.
        native_front_w, native_front_h = self.front_native_size
        self.front_display_size = (
            round(native_front_w * combo.shape[0] / original_combo_h),
            round(native_front_h * combo.shape[0] / original_combo_h)
        )
        self._control_bottom = combo.shape[0]

        self._click_poll()
        self._click_drive_buttons(combo, *self.front_display_size)
        self._draw_spray(combo)
        cv2.imshow(
            self.combined_window,
            combo
        )

        if not self._window_maximize_attempted:
            self._window_maximize_attempted = True
            if not maximize_window(self.combined_window):
                self.get_logger().warning(
                    'Could not maximize the camera window; '
                    'the OpenCV Qt window was not found.')

        key = cv2.waitKey(
            self.waitkey_ms
        ) & 0xFF

        if key == ord('q'):
            self._manual_stop()
            rclpy.shutdown()

    def destroy_node(self):

        self._manual_stop()
        self.tracking_worker.close()
        app = QApplication.instance()
        if app is not None and self._motion_window_events is not None:
            app.removeEventFilter(self._motion_window_events)
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = DualCompressedViewer()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # ROS callbacks run in the background. OpenCV GUI operations remain
    # in the main thread, which is required for reliable GTK/OpenCV
    # window behavior.
    def spin():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    spin_thread = threading.Thread(
        target=spin,
        daemon=True
    )

    spin_thread.start()

    try:

        period = 0.0

        if node.max_display_hz > 0.0:
            period = 1.0 / node.max_display_hz

        while rclpy.ok():

            node.draw()

            if period > 0.0:
                time.sleep(period)

    except KeyboardInterrupt:
        pass

    finally:
        node._manual_stop()
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        executor.remove_node(node)
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
