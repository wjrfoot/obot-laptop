#!/usr/bin/env python3

import math
import threading
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from obot_msgs.srv import ClickDrive
from tf2_ros import Buffer, TransformException, TransformListener
from PyQt5.QtWidgets import QApplication

from .fisheye_bearing import FisheyeBearing
from .camera_dashboard import render_dashboard
from .camera_telemetry import Telemetry, wheel_speeds, yaw_from_quaternion


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
    if factor <= 1:
        return image
    size = (max(1, round(image_w * factor)),
            max(1, round(image_h * factor)))
    return cv2.resize(image, size, interpolation=cv2.INTER_LINEAR)


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
        if settings['enabled']:
            self._init_telemetry(settings)

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
        self.selected_target = None
        self.click_state = 'idle'  # idle, selected, turning, ready, driving, stopping
        self.click_distance_text = '5.0'
        self.click_distance_edit = False
        self._click_saw_turn_motion = False
        self._click_saw_drive_motion = False
        self.click_message = ''
        self.click_enable_motion = False
        self.declare_parameter('click.enable_motion', False)
        self.click_enable_motion = bool(self.get_parameter('click.enable_motion').value)
        self._click_client = self.create_client(ClickDrive, '/click_drive')
        self._click_future = None
        self._click_request_kind = None
        self._click_last_status = 0.0
        self._click_turn_started = 0.0
        self._click_button_hits = {}
        self.front_display_size = None
        self.front_native_size = None

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
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
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

    def _on_command(self, msg):
        values = (msg.twist.linear.x, msg.twist.angular.z)
        self.telemetry.put('command', values if all(map(math.isfinite, values)) else None)

    def _on_odom(self, msg):
        values = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        self.telemetry.put('actual', values if all(map(math.isfinite, values)) else None)

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

    def _click_request(self, command, angle=0.0, distance=0.0):
        """Nonblocking ROS service call; results processed in draw()."""
        if self._click_future is not None:
            return False
        if not self._click_client.service_is_ready():
            self.click_message = 'ClickDrive service unavailable'
            return False
        request = ClickDrive.Request()
        request.command = command
        request.angle_deg = float(angle)
        request.distance_m = float(distance)
        self._click_future = self._click_client.call_async(request)
        self._click_request_kind = command
        return True

    def _click_turn(self):
        if self.click_state != 'selected' or self.selected_target is None:
            return
        if not self.click_enable_motion:
            self.click_message = 'TEST MODE: TURN not sent (click.enable_motion=false)'
            return
        if self._click_request('TURN', self.selected_target[2]):
            self.click_state = 'turning'
            self._click_turn_started = time.monotonic()
            self._click_saw_turn_motion = False
            self.click_message = 'TURN requested...'

    def _click_cancel(self):
        if self.click_state == 'selected':
            self.selected_target = None
            self.click_state = 'idle'
            self.click_message = 'Target canceled'
        elif self.click_state in ('ready', 'turning', 'driving', 'stopping'):
            # Wait for any in-flight request before STOP.
            self.click_state = 'stopping'
            self.click_message = 'STOP pending...'
            if self._click_future is None:
                self._click_request('STOP')

    def _click_go(self):
        if self.click_state != 'ready' or self._click_future is not None:
            return
        try:
            distance = float(self.click_distance_text)
        except ValueError:
            self.click_message = 'Enter a valid distance (0.1 to 5.0 m)'
            return
        if not math.isfinite(distance) or not 0.1 <= distance <= 5.0:
            self.click_message = 'Distance must be 0.1 to 5.0 m'
            return
        if not self.click_enable_motion:
            self.click_message = 'TEST MODE: DRIVE not sent'
            return
        if self._click_request('DRIVE', distance=distance):
            self.click_state = 'driving'
            self._click_saw_drive_motion = False
            self.click_distance_edit = False
            self.click_message = f'DRIVE {distance:.2f} m requested...'

    def _click_distance_key(self, key):
        if self.click_state != 'ready' or not self.click_distance_edit:
            return
        if key in (10, 13):
            self.click_distance_edit = False
        elif key in (8, 127):
            self.click_distance_text = self.click_distance_text[:-1]
        elif 48 <= key <= 57 or key == 46:
            char = chr(key)
            if char != '.' or '.' not in self.click_distance_text:
                self.click_distance_text = (self.click_distance_text + char)[:8]

    def _click_poll(self):
        """Called by GUI thread; never blocks camera drawing."""
        future = self._click_future
        if future is not None and future.done():
            kind = self._click_request_kind
            self._click_future = None
            self._click_request_kind = None
            try:
                response = future.result()
                if not response.accepted:
                    self.click_message = f'{kind} rejected: {response.message}'
                    if kind == 'TURN' and self.click_state == 'turning':
                        self.click_state = 'selected'
                    elif kind == 'DRIVE' and self.click_state == 'driving':
                        self.click_state = 'ready'
                elif kind in ('TURN', 'DRIVE'):
                    self.click_message = response.message
                elif kind == 'STOP':
                    self.click_message = 'STOP accepted'
                    self.click_state = 'idle'
                    self.selected_target = None
                elif kind == 'STATUS' and self.click_state in ('turning', 'driving'):
                    # Do not mistake the brief idle interval after enqueue for
                    # completion. Require observing non-idle first.
                    if response.state != 'idle':
                        if self.click_state == 'turning':
                            self._click_saw_turn_motion = True
                        else:
                            self._click_saw_drive_motion = True
                    elif self.click_state == 'turning' and self._click_saw_turn_motion:
                        self.click_state = 'ready'
                        self.click_message = 'Turn complete; enter distance, then GO'
                    elif self.click_state == 'driving' and self._click_saw_drive_motion:
                        self.click_state = 'idle'
                        self.selected_target = None
                        self.click_message = 'Drive complete (controller idle)'
            except Exception as exc:
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
        """Draw buttons in displayed pixel coordinates; cache hitboxes."""
        self._click_button_hits = {}
        if self.selected_target is None and self.click_state == 'idle':
            return
        w = min(front_w, canvas.shape[1])
        h = min(front_h, canvas.shape[0])
        if w < 150 or h < 75:
            return
        margin = 8
        gap = 8
        button_w = max(60, (w - 2 * margin - gap) // 2)
        button_h = 34
        y0 = h - button_h - margin
        if self.click_state == 'ready':
            # Click the distance field and type a new number; Enter finishes.
            field_y0 = max(36, y0 - 40)
            field_y1 = y0 - 5
            cv2.rectangle(canvas, (margin, field_y0), (w-margin, field_y1),
                          (70, 70, 70) if self.click_distance_edit else (35, 35, 35), -1)
            cv2.rectangle(canvas, (margin, field_y0), (w-margin, field_y1),
                          (255, 255, 255), 1)
            label = f'Distance: {self.click_distance_text} m' + ('  [typing]' if self.click_distance_edit else '  [click to edit]')
            cv2.putText(canvas, label[:38], (margin+7, field_y0+22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255,255,255), 1)
            self._click_button_hits['DISTANCE'] = (margin, field_y0, w-margin, field_y1)
        for name, x0, color in (
            ('GO' if self.click_state == 'ready' else 'TURN', margin, (35, 135, 35)),
            ('CANCEL', margin + button_w + gap, (35, 35, 180)),
        ):
            x1 = min(x0 + button_w, w - margin)
            y1 = y0 + button_h
            cv2.rectangle(canvas, (x0, y0), (x1, y1), color, -1)
            cv2.rectangle(canvas, (x0, y0), (x1, y1), (220, 220, 220), 1)
            label = 'STOP' if name == 'CANCEL' and self.click_state in ('turning', 'driving', 'stopping') else name
            cv2.putText(canvas, label, (x0 + 9, y0 + 23),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2)
            self._click_button_hits[name] = (x0, y0, x1, y1)
        if self.click_message:
            cv2.putText(canvas, self.click_message[:55], (8, max(45, y0 - 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1)

    def _click_handle_button(self, x, y):
        for name, (x0, y0, x1, y1) in self._click_button_hits.items():
            if x0 <= x < x1 and y0 <= y < y1:
                if name == 'TURN':
                    self._click_turn()
                elif name == 'GO':
                    self._click_go()
                elif name == 'DISTANCE':
                    self.click_distance_edit = True
                    self.click_distance_text = ''
                    self.click_message = 'Type distance in meters; press Enter'
                else:
                    self._click_cancel()
                return True
        return False

    def _select_front_target(self, x, y):
        """Select a point in displayed front-image coordinates."""
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

        if self.click_state in ('turning', 'driving', 'stopping', 'ready'):
            return
        self.selected_target = (pixel_x, pixel_y, bearing)
        self.click_state = 'selected'
        self.click_message = ('TEST MODE: motion disabled' if not self.click_enable_motion
                              else 'Select TURN or CANCEL')

        direction = 'RIGHT' if bearing > 0 else 'LEFT'
        self.get_logger().info(
            f'TARGET pixel=({pixel_x:.1f}, {pixel_y:.1f}) '
            f'bearing={bearing:+.2f} deg ({direction})'
        )

    def _on_front_mouse(self, event, x, y, _flags, _userdata):
        """Mouse callback for the separate front-camera window."""
        if event == cv2.EVENT_LBUTTONUP:
            if not self._click_handle_button(x, y):
                self._select_front_target(x, y)

    def _on_mouse(self, event, x, y, _flags, _userdata):
        if event != cv2.EVENT_LBUTTONUP:
            return

        # Front-camera clicks select targets.
        if self.front_display_size is not None:
            fw, fh = self.front_display_size
            if 0 <= x < fw and 0 <= y < fh:
                if not self._click_handle_button(x, y):
                    self._select_front_target(x, y)
                return

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

        draw_markers(left, self.overlay)

        if self.selected_target is not None:
            tx, ty, bearing = self.selected_target
            cx, cy = round(tx), round(ty)

            cv2.drawMarker(
                left, (cx, cy), (0, 255, 255),
                cv2.MARKER_CROSS, 16, 2
            )

            direction = 'RIGHT' if bearing >= 0 else 'LEFT'
            label = f'{direction} {abs(bearing):.1f} deg'

            cv2.rectangle(
                left, (4, 4), (225, 31), (0, 0, 0), -1
            )
            cv2.putText(
                left, label, (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65, (0, 255, 255), 2
            )

        self.front_native_size = (left.shape[1], left.shape[0])

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
                    cv2.setMouseCallback(self.window_right, self._on_mouse)
                    self._rear_window_ready = True
                right_canvas = display_sized_image(right_canvas, self.window_right)
                panel_y = round(box_h * right_canvas.shape[0] / max(left.shape[0], box_h))
                self._draw_dashboard(right_canvas, 0, panel_y)
                right = right_canvas

            self.front_display_size = (
                left.shape[1], left.shape[0]
            )
            cv2.setMouseCallback(
                self.window_left, self._on_front_mouse
            )
            self._click_poll()
            self._click_drive_buttons(left, *self.front_display_size)
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

            if self.click_distance_edit:
                self._click_distance_key(key)
            elif key == ord('q'):
                rclpy.shutdown()

            return

        combo = compose(left, right, self.overlay.rear_scale)
        # IMPORTANT:
        # Check whether the window was closed BEFORE imshow().
        # Calling imshow() first would recreate a closed window.
        if cv2.getWindowProperty(
            self.combined_window,
            cv2.WND_PROP_VISIBLE
        ) < 1:
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

        original_combo_h, original_combo_w = combo.shape[:2]

        if self.telemetry:
            original_h, original_w = combo.shape[:2]
            combo = display_sized_image(combo, self.combined_window)
            fh, fw = left.shape[:2]
            panel_x = round(fw * combo.shape[1] / original_w)
            panel_top = round(fh * self.overlay.rear_scale
                              * combo.shape[0] / original_h)
            self._draw_dashboard(combo, panel_x, panel_top)

        # OpenCV mouse coordinates refer to the displayed image.
        # Convert the front-camera region using the composite scale.
        native_front_w, native_front_h = self.front_native_size
        self.front_display_size = (
            round(native_front_w * combo.shape[1] / original_combo_w),
            round(native_front_h * combo.shape[0] / original_combo_h)
        )

        self._click_poll()
        self._click_drive_buttons(combo, *self.front_display_size)
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

        if self.click_distance_edit:
            self._click_distance_key(key)
        elif key == ord('q'):
            rclpy.shutdown()

    def destroy_node(self):

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

        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        executor.remove_node(node)
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()



if __name__ == '__main__':
    main()
