#!/usr/bin/env python3
"""Standalone PyQt5 map/costmap/heatmap canvas for planning spray waypoints.

Replaces an RViz session for this specific workflow: view the static map,
global costmap and Wi-Fi RSSI heatmap; set a pose estimate; define an origin
to express coordinates relative to; add waypoints with a spray duration; and
generate a text report of the plan plus any spray events observed on the bus.
"""
import math
import os
import sys
from dataclasses import dataclass
from datetime import datetime
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32
from visualization_msgs.msg import MarkerArray

from obot_msgs.msg import SprayEvent

from PyQt5.QtCore import QPointF, QRectF, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import (
    QBrush,
    QColor,
    QFont,
    QImage,
    QPageLayout,
    QPageSize,
    QPainter,
    QPdfWriter,
    QPen,
    QPolygonF,
)
from PyQt5.QtWidgets import (
    QApplication,
    QAbstractItemView,
    QComboBox,
    QDoubleSpinBox,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)


def yaw_to_quaternion(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def to_origin_frame(x: float, y: float, origin_x: float, origin_y: float, origin_yaw: float):
    dx = x - origin_x
    dy = y - origin_y
    c = math.cos(-origin_yaw)
    s = math.sin(-origin_yaw)
    return dx * c - dy * s, dx * s + dy * c


@dataclass
class Waypoint:
    x: float
    y: float
    yaw: float
    spray_sec: float


@dataclass
class SprayLogEntry:
    stamp_str: str
    x: float
    y: float
    yaw: float
    spray_sec: float
    waypoint_index: int
    label: str


MODE_NAVIGATE = 'navigate'
MODE_POSE_ESTIMATE = 'pose_estimate'
MODE_ORIGIN = 'origin'
MODE_WAYPOINT = 'waypoint'


class MapCanvas(QWidget):
    poseEstimateSet = pyqtSignal(float, float, float)
    originSet = pyqtSignal(float, float, float)
    waypointAdded = pyqtSignal(float, float, float)
    mouseWorldMoved = pyqtSignal(float, float)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setMinimumSize(480, 480)
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.StrongFocus)

        self.scale_px_per_m = 40.0
        self.center_x = 0.0
        self.center_y = 0.0

        self.show_map = True
        self.show_costmap = True
        self.show_heatmap = True
        self.show_robot_path = True

        self.mode = MODE_NAVIGATE

        self.map_msg: Optional[OccupancyGrid] = None
        self.map_image: Optional[QImage] = None
        self.costmap_msg: Optional[OccupancyGrid] = None
        self.costmap_image: Optional[QImage] = None
        self.heatmap_markers: List = []

        self.origin = (0.0, 0.0, 0.0)
        self.pose_estimate: Optional[tuple] = None
        self.robot_pose: Optional[tuple] = None
        self.robot_path: List[tuple] = []
        self.waypoints: List[Waypoint] = []
        self.spray_events: List[SprayLogEntry] = []

        self._panning = False
        self._pan_last = QPointF()
        self._dragging_pose = False
        self._drag_start_world = (0.0, 0.0)
        self._drag_current_world = (0.0, 0.0)

    # ---- data updates -------------------------------------------------
    def set_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg
        self.map_image = self._grid_to_image(msg, costmap=False)
        self.update()

    def set_costmap(self, msg: OccupancyGrid) -> None:
        self.costmap_msg = msg
        self.costmap_image = self._grid_to_image(msg, costmap=True)
        self.update()

    def set_heatmap(self, markers: List) -> None:
        self.heatmap_markers = markers
        self.update()

    def set_robot_pose(self, x: float, y: float, yaw: float) -> None:
        self.robot_pose = (x, y, yaw)
        if not self.robot_path or math.hypot(x - self.robot_path[-1][0], y - self.robot_path[-1][1]) > 0.05:
            self.robot_path.append((x, y))
            if len(self.robot_path) > 20000:
                self.robot_path = self.robot_path[-20000:]
        self.update()

    def add_spray_event(self, entry: SprayLogEntry) -> None:
        self.spray_events.append(entry)
        if len(self.spray_events) > 2000:
            self.spray_events = self.spray_events[-2000:]
        self.update()

    @staticmethod
    def _grid_to_image(msg: OccupancyGrid, costmap: bool) -> QImage:
        width = msg.info.width
        height = msg.info.height
        if width <= 0 or height <= 0:
            return QImage()

        image = QImage(width, height, QImage.Format_ARGB32)
        data = msg.data
        for row in range(height):
            image_row = height - 1 - row
            base = row * width
            for col in range(width):
                value = data[base + col]
                if costmap:
                    if value < 0:
                        color = QColor(0, 0, 0, 0)
                    else:
                        alpha = int(min(1.0, value / 100.0) * 200)
                        green = int(255 * (1.0 - value / 100.0))
                        color = QColor(255, green, 0, alpha)
                else:
                    if value < 0:
                        color = QColor(200, 200, 200, 255)
                    else:
                        shade = 255 - int(min(100, value) * 2.55)
                        color = QColor(shade, shade, shade, 255)
                image.setPixelColor(col, image_row, color)
        return image

    # ---- coordinate transforms -----------------------------------------
    def world_to_screen(self, wx: float, wy: float) -> QPointF:
        sx = self.width() / 2.0 + (wx - self.center_x) * self.scale_px_per_m
        sy = self.height() / 2.0 - (wy - self.center_y) * self.scale_px_per_m
        return QPointF(sx, sy)

    def screen_to_world(self, sx: float, sy: float):
        wx = self.center_x + (sx - self.width() / 2.0) / self.scale_px_per_m
        wy = self.center_y - (sy - self.height() / 2.0) / self.scale_px_per_m
        return wx, wy

    # ---- mouse / interaction -------------------------------------------
    def set_mode(self, mode: str) -> None:
        self.mode = mode

    def wheelEvent(self, event) -> None:
        pos = event.pos()
        wx_before, wy_before = self.screen_to_world(pos.x(), pos.y())
        factor = 1.15 if event.angleDelta().y() > 0 else 1.0 / 1.15
        self.scale_px_per_m = max(2.0, min(400.0, self.scale_px_per_m * factor))
        wx_after, wy_after = self.screen_to_world(pos.x(), pos.y())
        self.center_x += wx_before - wx_after
        self.center_y += wy_before - wy_after
        self.update()

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.MiddleButton or (
            event.button() == Qt.LeftButton and self.mode == MODE_NAVIGATE
        ):
            self._panning = True
            self._pan_last = event.pos()
            return

        if event.button() == Qt.LeftButton and self.mode in (
            MODE_POSE_ESTIMATE,
            MODE_ORIGIN,
            MODE_WAYPOINT,
        ):
            wx, wy = self.screen_to_world(event.pos().x(), event.pos().y())
            self._dragging_pose = True
            self._drag_start_world = (wx, wy)
            self._drag_current_world = (wx, wy)
            self.update()

    def mouseMoveEvent(self, event) -> None:
        wx, wy = self.screen_to_world(event.pos().x(), event.pos().y())
        self.mouseWorldMoved.emit(wx, wy)

        if self._panning:
            delta = event.pos() - self._pan_last
            self._pan_last = event.pos()
            self.center_x -= delta.x() / self.scale_px_per_m
            self.center_y += delta.y() / self.scale_px_per_m
            self.update()
            return

        if self._dragging_pose:
            self._drag_current_world = (wx, wy)
            self.update()

    def mouseReleaseEvent(self, event) -> None:
        if event.button() == Qt.MiddleButton or (
            event.button() == Qt.LeftButton and self._panning
        ):
            self._panning = False
            return

        if event.button() == Qt.LeftButton and self._dragging_pose:
            self._dragging_pose = False
            sx, sy = self._drag_start_world
            ex, ey = self._drag_current_world
            if math.hypot(ex - sx, ey - sy) > 0.05:
                yaw = math.atan2(ey - sy, ex - sx)
            else:
                yaw = 0.0

            if self.mode == MODE_POSE_ESTIMATE:
                self.pose_estimate = (sx, sy, yaw)
                self.poseEstimateSet.emit(sx, sy, yaw)
            elif self.mode == MODE_ORIGIN:
                self.origin = (sx, sy, yaw)
                self.originSet.emit(sx, sy, yaw)
            elif self.mode == MODE_WAYPOINT:
                self.waypointAdded.emit(sx, sy, yaw)
            self.update()

    # ---- painting --------------------------------------------------------
    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor(60, 60, 60))
        painter.setRenderHint(QPainter.Antialiasing, True)

        if self.show_map and self.map_msg is not None and self.map_image is not None:
            self._draw_grid_image(painter, self.map_msg, self.map_image)
        if self.show_costmap and self.costmap_msg is not None and self.costmap_image is not None:
            self._draw_grid_image(painter, self.costmap_msg, self.costmap_image)
        if self.show_heatmap:
            self._draw_heatmap(painter)
        if self.show_robot_path:
            self._draw_robot_path(painter, self.robot_path)

        self._draw_origin(painter)
        self._draw_waypoints(painter)
        self._draw_spray_events(painter)
        if self.pose_estimate is not None:
            self._draw_arrow(painter, self.pose_estimate, QColor(0, 200, 0), 0.5)
        if self.robot_pose is not None:
            self._draw_arrow(painter, self.robot_pose, QColor(30, 120, 255), 0.4)
        if self._dragging_pose:
            sx, sy = self._drag_start_world
            ex, ey = self._drag_current_world
            yaw = math.atan2(ey - sy, ex - sx) if math.hypot(ex - sx, ey - sy) > 0.05 else 0.0
            color = {
                MODE_POSE_ESTIMATE: QColor(0, 200, 0),
                MODE_ORIGIN: QColor(230, 0, 230),
                MODE_WAYPOINT: QColor(0, 170, 230),
            }.get(self.mode, QColor(255, 255, 255))
            self._draw_arrow(painter, (sx, sy, yaw), color, 0.5)

        painter.end()

    def _draw_grid_image(
        self, painter: QPainter, msg: OccupancyGrid, image: QImage, w2s=None, scale=None
    ) -> None:
        w2s = w2s or self.world_to_screen
        scale = self.scale_px_per_m if scale is None else scale
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        width_m = msg.info.width * res
        height_m = msg.info.height * res
        top_left = w2s(ox, oy + height_m)
        w_px = width_m * scale
        h_px = height_m * scale
        painter.drawImage(QRectF(top_left.x(), top_left.y(), w_px, h_px), image)

    def _draw_heatmap(self, painter: QPainter, markers=None, w2s=None, scale=None) -> None:
        markers = self.heatmap_markers if markers is None else markers
        w2s = w2s or self.world_to_screen
        scale = self.scale_px_per_m if scale is None else scale
        for marker in markers:
            if marker.action != 0:  # ADD
                continue
            cx = marker.pose.position.x
            cy = marker.pose.position.y
            w_m = marker.scale.x
            h_m = marker.scale.y
            color = QColor(
                int(marker.color.r * 255),
                int(marker.color.g * 255),
                int(marker.color.b * 255),
                int(marker.color.a * 255),
            )
            top_left = w2s(cx - w_m / 2.0, cy + h_m / 2.0)
            w_px = w_m * scale
            h_px = h_m * scale
            painter.fillRect(QRectF(top_left.x(), top_left.y(), w_px, h_px), QBrush(color))

    def _draw_robot_path(self, painter: QPainter, path, w2s=None) -> None:
        if len(path) < 2:
            return
        w2s = w2s or self.world_to_screen
        painter.setPen(QPen(QColor(30, 120, 255), 2))
        points = [w2s(x, y) for x, y in path]
        for i in range(len(points) - 1):
            painter.drawLine(points[i], points[i + 1])

    def _draw_origin(self, painter: QPainter, origin=None, w2s=None, scale=None) -> None:
        origin = self.origin if origin is None else origin
        w2s = w2s or self.world_to_screen
        scale = self.scale_px_per_m if scale is None else scale
        ox, oy, oyaw = origin
        center = w2s(ox, oy)
        axis_len_px = 0.6 * scale
        x_end = w2s(ox + 0.6 * math.cos(oyaw), oy + 0.6 * math.sin(oyaw))
        y_end = w2s(
            ox + 0.6 * math.cos(oyaw + math.pi / 2), oy + 0.6 * math.sin(oyaw + math.pi / 2)
        )
        painter.setPen(QPen(QColor(230, 0, 0), 2))
        painter.drawLine(center, x_end)
        painter.setPen(QPen(QColor(0, 200, 0), 2))
        painter.drawLine(center, y_end)
        painter.setPen(QPen(QColor(230, 0, 230), 1))
        painter.drawEllipse(center, axis_len_px * 0.08, axis_len_px * 0.08)

    def _draw_waypoints(self, painter: QPainter, waypoints=None, w2s=None) -> None:
        waypoints = self.waypoints if waypoints is None else waypoints
        w2s = w2s or self.world_to_screen
        for idx, wp in enumerate(waypoints):
            self._draw_arrow(painter, (wp.x, wp.y, wp.yaw), QColor(0, 170, 230), 0.35, w2s=w2s)
            pt = w2s(wp.x, wp.y)
            label = f'{idx + 1}'
            if wp.spray_sec > 0.0:
                label += f' ({wp.spray_sec:.1f}s)'
            painter.setPen(QPen(QColor(255, 255, 255)))
            painter.drawText(pt + QPointF(8, -8), label)

    def _draw_spray_events(self, painter: QPainter) -> None:
        for entry in self.spray_events[-200:]:
            length_m = max(0.2, min(2.0, 0.2 + entry.spray_sec * 0.05))
            pose = (entry.x, entry.y, entry.yaw)
            self._draw_arrow(painter, pose, QColor(255, 140, 0), length_m, filled_base=True)

    def _draw_arrow(
        self,
        painter: QPainter,
        pose: tuple,
        color: QColor,
        length_m: float,
        filled_base: bool = False,
        w2s=None,
    ) -> None:
        w2s = w2s or self.world_to_screen
        x, y, yaw = pose
        tail = w2s(x, y)
        head = w2s(x + length_m * math.cos(yaw), y + length_m * math.sin(yaw))
        painter.setPen(QPen(color, 2))
        painter.drawLine(tail, head)

        angle = math.atan2(head.y() - tail.y(), head.x() - tail.x())
        wing = 8.0
        left = head + QPointF(
            -wing * math.cos(angle - math.pi / 6), -wing * math.sin(angle - math.pi / 6)
        )
        right = head + QPointF(
            -wing * math.cos(angle + math.pi / 6), -wing * math.sin(angle + math.pi / 6)
        )
        painter.setBrush(QBrush(color))
        painter.drawPolygon(QPolygonF([head, left, right]))

        if filled_base:
            painter.setBrush(QBrush(color))
            painter.drawEllipse(tail, 4, 4)

    # ---- static render for the PDF report -------------------------------
    def render_report_image(self, width_px: int, height_px: int) -> QImage:
        """Render map + heatmap + origin + robot path + waypoints to a fresh image, fit to bounds."""
        xs: List[float] = []
        ys: List[float] = []
        if self.map_msg is not None:
            res = self.map_msg.info.resolution
            ox = self.map_msg.info.origin.position.x
            oy = self.map_msg.info.origin.position.y
            xs += [ox, ox + self.map_msg.info.width * res]
            ys += [oy, oy + self.map_msg.info.height * res]
        for wp in self.waypoints:
            xs.append(wp.x)
            ys.append(wp.y)
        for px, py in self.robot_path:
            xs.append(px)
            ys.append(py)
        ox_, oy_, _ = self.origin
        xs.append(ox_)
        ys.append(oy_)
        if not xs:
            xs, ys = [-1.0, 1.0], [-1.0, 1.0]

        margin_m = 1.0
        min_x, max_x = min(xs) - margin_m, max(xs) + margin_m
        min_y, max_y = min(ys) - margin_m, max(ys) + margin_m
        world_w = max(0.5, max_x - min_x)
        world_h = max(0.5, max_y - min_y)

        margin_px = 20.0
        scale = min((width_px - 2 * margin_px) / world_w, (height_px - 2 * margin_px) / world_h)
        cx = (min_x + max_x) / 2.0
        cy = (min_y + max_y) / 2.0

        def w2s(wx: float, wy: float) -> QPointF:
            sx = width_px / 2.0 + (wx - cx) * scale
            sy = height_px / 2.0 - (wy - cy) * scale
            return QPointF(sx, sy)

        image = QImage(width_px, height_px, QImage.Format_ARGB32)
        image.fill(QColor(60, 60, 60))
        painter = QPainter(image)
        painter.setRenderHint(QPainter.Antialiasing, True)

        if self.map_msg is not None and self.map_image is not None:
            self._draw_grid_image(painter, self.map_msg, self.map_image, w2s=w2s, scale=scale)
        self._draw_heatmap(painter, w2s=w2s, scale=scale)
        self._draw_robot_path(painter, self.robot_path, w2s=w2s)
        self._draw_origin(painter, w2s=w2s, scale=scale)
        self._draw_waypoints(painter, w2s=w2s)

        painter.end()
        return image


class SprayConsoleNode(Node):
    def __init__(self) -> None:
        super().__init__('spray_mission_console')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('heatmap_topic', '/wifi/heatmap_markers')
        self.declare_parameter('initialpose_topic', '/initialpose')
        self.declare_parameter('amcl_pose_topic', '/amcl_pose')
        self.declare_parameter('spray_topic', '/spray/event')
        self.declare_parameter('valve_command_topic', '/spray/valve_command')

        self.map_frame = str(self.get_parameter('map_frame').value)
        map_topic = str(self.get_parameter('map_topic').value)
        costmap_topic = str(self.get_parameter('costmap_topic').value)
        heatmap_topic = str(self.get_parameter('heatmap_topic').value)
        initialpose_topic = str(self.get_parameter('initialpose_topic').value)
        amcl_pose_topic = str(self.get_parameter('amcl_pose_topic').value)
        self.spray_topic = str(self.get_parameter('spray_topic').value)
        valve_command_topic = str(self.get_parameter('valve_command_topic').value)

        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        self.on_map = None
        self.on_costmap = None
        self.on_heatmap = None
        self.on_amcl_pose = None
        self.on_spray_event = None

        self.create_subscription(OccupancyGrid, map_topic, self._map_cb, map_qos)
        self.create_subscription(OccupancyGrid, costmap_topic, self._costmap_cb, map_qos)
        self.create_subscription(MarkerArray, heatmap_topic, self._heatmap_cb, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, amcl_pose_topic, self._amcl_pose_cb, 10
        )
        self.create_subscription(SprayEvent, self.spray_topic, self._spray_event_cb, 20)

        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, initialpose_topic, 10
        )
        self.spray_pub = self.create_publisher(SprayEvent, self.spray_topic, 20)
        self.valve_cmd_pub = self.create_publisher(Float32, valve_command_topic, 10)

        self.get_logger().info(
            f'spray_mission_console started: map={map_topic}, costmap={costmap_topic}, '
            f'heatmap={heatmap_topic}, spray_topic={self.spray_topic}, '
            f'valve_command_topic={valve_command_topic}'
        )

    def _map_cb(self, msg: OccupancyGrid) -> None:
        if self.on_map is not None:
            self.on_map(msg)

    def _costmap_cb(self, msg: OccupancyGrid) -> None:
        if self.on_costmap is not None:
            self.on_costmap(msg)

    def _heatmap_cb(self, msg: MarkerArray) -> None:
        if self.on_heatmap is not None:
            self.on_heatmap(list(msg.markers))

    def _amcl_pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        if self.on_amcl_pose is not None:
            q = msg.pose.pose.orientation
            yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
            self.on_amcl_pose(msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def _spray_event_cb(self, msg: SprayEvent) -> None:
        if self.on_spray_event is not None:
            q = msg.pose.orientation
            yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
            self.on_spray_event(msg, yaw)

    def publish_initial_pose(self, x: float, y: float, yaw: float) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.07
        self.initialpose_pub.publish(msg)

    def publish_spray_event(
        self, x: float, y: float, yaw: float, spray_sec: float, waypoint_index: int, label: str
    ) -> None:
        msg = SprayEvent()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        msg.spray_duration_sec = float(spray_sec)
        msg.waypoint_index = int(waypoint_index)
        msg.label = label
        self.spray_pub.publish(msg)

        # Drives the CYD-attached solenoid: positive duration opens it, <=0 closes it.
        self.valve_cmd_pub.publish(Float32(data=float(spray_sec)))


class MainWindow(QMainWindow):
    def __init__(self, node: SprayConsoleNode) -> None:
        super().__init__()
        self.node = node
        self.setWindowTitle('OBot Spray Mission Console')
        self.resize(1200, 800)

        self.canvas = MapCanvas(self)
        self.canvas.poseEstimateSet.connect(self._on_pose_estimate_set)
        self.canvas.originSet.connect(self._on_origin_set)
        self.canvas.waypointAdded.connect(self._on_waypoint_added)
        self.canvas.mouseWorldMoved.connect(self._on_mouse_world_moved)

        node.on_map = self.canvas.set_map
        node.on_costmap = self.canvas.set_costmap
        node.on_heatmap = self.canvas.set_heatmap
        node.on_amcl_pose = self._on_amcl_pose
        node.on_spray_event = self._on_spray_event

        central = QWidget(self)
        layout = QHBoxLayout(central)
        layout.addWidget(self.canvas, stretch=1)
        layout.addWidget(self._build_side_panel(), stretch=0)
        self.setCentralWidget(central)
        self.statusBar().showMessage('Ready')

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._spin_once)
        self.timer.start(30)

    # ---- side panel ----------------------------------------------------
    def _build_side_panel(self) -> QWidget:
        panel = QWidget(self)
        v = QVBoxLayout(panel)
        panel.setFixedWidth(340)

        layers_box = QGroupBox('Layers')
        layers_layout = QVBoxLayout(layers_box)
        self.map_checkbox = self._checkbox('Map', True, self._on_map_toggled)
        self.costmap_checkbox = self._checkbox('Costmap', True, self._on_costmap_toggled)
        self.heatmap_checkbox = self._checkbox('Wi-Fi heatmap', True, self._on_heatmap_toggled)
        self.robot_path_checkbox = self._checkbox('Robot path', True, self._on_robot_path_toggled)
        for cb in (
            self.map_checkbox,
            self.costmap_checkbox,
            self.heatmap_checkbox,
            self.robot_path_checkbox,
        ):
            layers_layout.addWidget(cb)
        v.addWidget(layers_box)

        mode_box = QGroupBox('Interaction mode')
        mode_layout = QVBoxLayout(mode_box)
        self.mode_combo = QComboBox()
        self.mode_combo.addItem('Navigate (pan/zoom)', MODE_NAVIGATE)
        self.mode_combo.addItem('Set pose estimate', MODE_POSE_ESTIMATE)
        self.mode_combo.addItem('Set origin', MODE_ORIGIN)
        self.mode_combo.addItem('Add waypoint', MODE_WAYPOINT)
        self.mode_combo.currentIndexChanged.connect(self._on_mode_changed)
        mode_layout.addWidget(self.mode_combo)
        mode_layout.addWidget(
            QLabel('Click and drag on the map to set position + heading.')
        )
        v.addWidget(mode_box)

        origin_box = QGroupBox('Origin')
        origin_layout = QVBoxLayout(origin_box)
        self.origin_label = QLabel('x=0.00 y=0.00 yaw=0.0°')
        origin_layout.addWidget(self.origin_label)
        reset_origin_btn = QPushButton('Reset origin to map (0,0,0)')
        reset_origin_btn.clicked.connect(self._reset_origin)
        origin_layout.addWidget(reset_origin_btn)
        v.addWidget(origin_box)

        pose_box = QGroupBox('Pose estimate')
        pose_layout = QVBoxLayout(pose_box)
        self.pose_label = QLabel('(not set)')
        pose_layout.addWidget(self.pose_label)
        republish_btn = QPushButton('Re-publish pose estimate')
        republish_btn.clicked.connect(self._republish_pose_estimate)
        pose_layout.addWidget(republish_btn)
        v.addWidget(pose_box)

        wp_box = QGroupBox('Waypoints')
        wp_layout = QVBoxLayout(wp_box)
        default_spray_row = QHBoxLayout()
        default_spray_row.addWidget(QLabel('Default spray (s):'))
        self.default_spray_spin = QDoubleSpinBox()
        self.default_spray_spin.setRange(0.0, 3600.0)
        self.default_spray_spin.setValue(2.0)
        default_spray_row.addWidget(self.default_spray_spin)
        wp_layout.addLayout(default_spray_row)

        self.wp_table = QTableWidget(0, 6)
        self.wp_table.setHorizontalHeaderLabels(
            ['#', 'X (m)', 'Y (m)', 'Yaw (°)', 'Rel. X/Y', 'Spray (s)']
        )
        self.wp_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.wp_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.wp_table.itemChanged.connect(self._on_wp_item_changed)
        wp_layout.addWidget(self.wp_table)

        wp_buttons = QHBoxLayout()
        remove_btn = QPushButton('Remove selected')
        remove_btn.clicked.connect(self._remove_selected_waypoint)
        clear_btn = QPushButton('Clear all')
        clear_btn.clicked.connect(self._clear_waypoints)
        wp_buttons.addWidget(remove_btn)
        wp_buttons.addWidget(clear_btn)
        wp_layout.addLayout(wp_buttons)

        trigger_btn = QPushButton('Trigger spray @ selected waypoint')
        trigger_btn.clicked.connect(self._trigger_spray_selected)
        wp_layout.addWidget(trigger_btn)
        v.addWidget(wp_box)

        report_box = QGroupBox('Report')
        report_layout = QVBoxLayout(report_box)
        report_btn = QPushButton('Generate report')
        report_btn.clicked.connect(self._generate_report)
        report_layout.addWidget(report_btn)
        v.addWidget(report_box)

        v.addStretch(1)
        return panel

    @staticmethod
    def _checkbox(label, checked, slot):
        from PyQt5.QtWidgets import QCheckBox

        cb = QCheckBox(label)
        cb.setChecked(checked)
        cb.stateChanged.connect(slot)
        return cb

    # ---- Qt/ROS glue -----------------------------------------------------
    def _spin_once(self) -> None:
        rclpy.spin_once(self.node, timeout_sec=0.0)

    # ---- layer toggles ----------------------------------------------------
    def _on_map_toggled(self, state) -> None:
        self.canvas.show_map = state == Qt.Checked
        self.canvas.update()

    def _on_costmap_toggled(self, state) -> None:
        self.canvas.show_costmap = state == Qt.Checked
        self.canvas.update()

    def _on_heatmap_toggled(self, state) -> None:
        self.canvas.show_heatmap = state == Qt.Checked
        self.canvas.update()

    def _on_robot_path_toggled(self, state) -> None:
        self.canvas.show_robot_path = state == Qt.Checked
        self.canvas.update()

    def _on_mode_changed(self, _index) -> None:
        self.canvas.set_mode(self.mode_combo.currentData())

    # ---- origin / pose estimate ---------------------------------------
    def _on_origin_set(self, x, y, yaw) -> None:
        self.origin_label.setText(f'x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.1f}°')
        self._refresh_waypoint_table()

    def _reset_origin(self) -> None:
        self.canvas.origin = (0.0, 0.0, 0.0)
        self.origin_label.setText('x=0.00 y=0.00 yaw=0.0°')
        self.canvas.update()
        self._refresh_waypoint_table()

    def _on_pose_estimate_set(self, x, y, yaw) -> None:
        self.pose_label.setText(f'x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.1f}°')
        self.node.publish_initial_pose(x, y, yaw)
        self.statusBar().showMessage(f'Published initial pose ({x:.2f}, {y:.2f})', 3000)

    def _republish_pose_estimate(self) -> None:
        if self.canvas.pose_estimate is None:
            QMessageBox.information(self, 'No pose set', 'Set a pose estimate on the map first.')
            return
        x, y, yaw = self.canvas.pose_estimate
        self.node.publish_initial_pose(x, y, yaw)
        self.statusBar().showMessage('Re-published initial pose', 3000)

    def _on_amcl_pose(self, x, y, yaw) -> None:
        self.canvas.set_robot_pose(x, y, yaw)

    def _on_mouse_world_moved(self, x, y) -> None:
        ox, oy, oyaw = self.canvas.origin
        rx, ry = to_origin_frame(x, y, ox, oy, oyaw)
        self.statusBar().showMessage(
            f'map: ({x:.2f}, {y:.2f})   relative to origin: ({rx:.2f}, {ry:.2f})'
        )

    # ---- waypoints ---------------------------------------------------
    def _on_waypoint_added(self, x, y, yaw) -> None:
        wp = Waypoint(x=x, y=y, yaw=yaw, spray_sec=float(self.default_spray_spin.value()))
        self.canvas.waypoints.append(wp)
        self._refresh_waypoint_table()
        self.canvas.update()

    def _refresh_waypoint_table(self) -> None:
        self.wp_table.blockSignals(True)
        self.wp_table.setRowCount(len(self.canvas.waypoints))
        ox, oy, oyaw = self.canvas.origin
        for row, wp in enumerate(self.canvas.waypoints):
            rx, ry = to_origin_frame(wp.x, wp.y, ox, oy, oyaw)
            values = [
                str(row + 1),
                f'{wp.x:.2f}',
                f'{wp.y:.2f}',
                f'{math.degrees(wp.yaw):.1f}',
                f'{rx:.2f}, {ry:.2f}',
                f'{wp.spray_sec:.1f}',
            ]
            for col, text in enumerate(values):
                item = QTableWidgetItem(text)
                if col != 5:
                    item.setFlags(item.flags() & ~Qt.ItemIsEditable)
                self.wp_table.setItem(row, col, item)
        self.wp_table.blockSignals(False)

    def _on_wp_item_changed(self, item: QTableWidgetItem) -> None:
        if item.column() != 5:
            return
        row = item.row()
        if row >= len(self.canvas.waypoints):
            return
        try:
            self.canvas.waypoints[row].spray_sec = max(0.0, float(item.text()))
        except ValueError:
            self._refresh_waypoint_table()
            return
        self.canvas.update()

    def _remove_selected_waypoint(self) -> None:
        rows = sorted({idx.row() for idx in self.wp_table.selectedIndexes()}, reverse=True)
        for row in rows:
            if 0 <= row < len(self.canvas.waypoints):
                del self.canvas.waypoints[row]
        self._refresh_waypoint_table()
        self.canvas.update()

    def _clear_waypoints(self) -> None:
        self.canvas.waypoints.clear()
        self._refresh_waypoint_table()
        self.canvas.update()

    def _trigger_spray_selected(self) -> None:
        rows = sorted({idx.row() for idx in self.wp_table.selectedIndexes()})
        if not rows:
            QMessageBox.information(self, 'No selection', 'Select one or more waypoint rows first.')
            return
        for row in rows:
            if row >= len(self.canvas.waypoints):
                continue
            wp = self.canvas.waypoints[row]
            if self.canvas.robot_pose is not None:
                x, y, yaw = self.canvas.robot_pose
            else:
                x, y, yaw = wp.x, wp.y, wp.yaw
            self.node.publish_spray_event(x, y, yaw, wp.spray_sec, row, label=f'waypoint_{row + 1}')
        self.statusBar().showMessage(f'Triggered spray on {len(rows)} waypoint(s)', 3000)

    def _on_spray_event(self, msg: SprayEvent, yaw: float) -> None:
        entry = SprayLogEntry(
            stamp_str=datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            x=msg.pose.position.x,
            y=msg.pose.position.y,
            yaw=yaw,
            spray_sec=msg.spray_duration_sec,
            waypoint_index=msg.waypoint_index,
            label=msg.label,
        )
        self.canvas.add_spray_event(entry)

    # ---- report --------------------------------------------------------
    def _generate_report(self) -> None:
        reports_dir = os.path.expanduser('~/reports')
        os.makedirs(reports_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        file_path = os.path.join(reports_dir, f'spray_report-{timestamp}.txt')

        ox, oy, oyaw = self.canvas.origin
        lines = []
        lines.append(f'Spray mission report - {datetime.now().isoformat()}')
        lines.append('')
        lines.append(f'Origin (map frame): x={ox:.3f} y={oy:.3f} yaw_deg={math.degrees(oyaw):.2f}')
        if self.canvas.pose_estimate is not None:
            px, py, pyaw = self.canvas.pose_estimate
            lines.append(
                f'Last pose estimate (map frame): x={px:.3f} y={py:.3f} '
                f'yaw_deg={math.degrees(pyaw):.2f}'
            )
        else:
            lines.append('Last pose estimate: (none set)')
        lines.append('')
        lines.append(f'Waypoints ({len(self.canvas.waypoints)}):')
        for idx, wp in enumerate(self.canvas.waypoints):
            rx, ry = to_origin_frame(wp.x, wp.y, ox, oy, oyaw)
            lines.append(
                f'  #{idx + 1}: map=({wp.x:.3f}, {wp.y:.3f}) yaw_deg={math.degrees(wp.yaw):.2f} '
                f'relative_to_origin=({rx:.3f}, {ry:.3f}) spray_sec={wp.spray_sec:.1f}'
            )
        lines.append('')
        lines.append(f'Spray events observed ({len(self.canvas.spray_events)}):')
        for entry in self.canvas.spray_events:
            rx, ry = to_origin_frame(entry.x, entry.y, ox, oy, oyaw)
            lines.append(
                f'  [{entry.stamp_str}] waypoint_index={entry.waypoint_index} '
                f'label={entry.label!r} map=({entry.x:.3f}, {entry.y:.3f}) '
                f'yaw_deg={math.degrees(entry.yaw):.2f} relative_to_origin=({rx:.3f}, {ry:.3f}) '
                f'spray_sec={entry.spray_sec:.1f}'
            )

        with open(file_path, 'w') as f:
            f.write('\n'.join(lines) + '\n')

        pdf_path = os.path.join(reports_dir, f'spray_report-{timestamp}.pdf')
        self._write_pdf_report(pdf_path)

        self.statusBar().showMessage(f'Report written to {file_path} and {pdf_path}', 5000)
        QMessageBox.information(
            self, 'Report generated', f'Saved report to:\n{file_path}\n{pdf_path}'
        )

    def _write_pdf_report(self, file_path: str) -> None:
        writer = QPdfWriter(file_path)
        writer.setResolution(150)
        writer.setPageSize(QPageSize(QPageSize.A4))
        writer.setPageOrientation(QPageLayout.Landscape)

        painter = QPainter(writer)
        page_rect = writer.pageLayout().paintRectPixels(writer.resolution())

        map_image = self.canvas.render_report_image(page_rect.width(), page_rect.height())
        painter.drawImage(page_rect, map_image)

        writer.newPage()
        self._draw_waypoint_table_page(painter, writer, page_rect)

        painter.end()

    def _draw_waypoint_table_page(self, painter: QPainter, writer: QPdfWriter, page_rect) -> None:
        ox, oy, oyaw = self.canvas.origin
        col_x = [
            page_rect.left() + 10,
            page_rect.left() + 80,
            page_rect.left() + 220,
            page_rect.left() + 360,
            page_rect.left() + 560,
        ]
        headers = ['#', 'Rel. X (m)', 'Rel. Y (m)', 'Orientation (deg)', 'Spray length (s)']

        base_font = painter.font()
        title_font = QFont(base_font)
        title_font.setPointSize(base_font.pointSize() + 4)
        title_font.setBold(True)
        bold_font = QFont(base_font)
        bold_font.setBold(True)

        line_h = 28
        y = page_rect.top() + 30
        painter.setFont(title_font)
        painter.drawText(col_x[0], y, 'Waypoint list (position/orientation relative to origin)')
        y += line_h * 2

        painter.setFont(bold_font)
        for x, text in zip(col_x, headers):
            painter.drawText(x, y, text)
        painter.setFont(base_font)
        y += line_h

        for idx, wp in enumerate(self.canvas.waypoints):
            if y > page_rect.bottom() - line_h:
                writer.newPage()
                y = page_rect.top() + 30
            rx, ry = to_origin_frame(wp.x, wp.y, ox, oy, oyaw)
            row = [
                str(idx + 1),
                f'{rx:.2f}',
                f'{ry:.2f}',
                f'{math.degrees(wp.yaw):.1f}',
                f'{wp.spray_sec:.1f}',
            ]
            for x, text in zip(col_x, row):
                painter.drawText(x, y, text)
            y += line_h


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SprayConsoleNode()

    app = QApplication(sys.argv[:1] if args is None else [sys.argv[0]])
    window = MainWindow(node)
    window.show()

    try:
        app.exec_()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
