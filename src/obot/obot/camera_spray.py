"""Viewer-side client and compact rendering for the existing spray action."""
import math
import threading
import time

import cv2
import numpy as np
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class SprayControl:
    def __init__(self, node, clock=time.monotonic, action_name='/obot/spray',
                 state_topic='/obot/valve/state'):
        from obot_valve_interfaces.action import SprayValve

        self.node = node
        self.action_type = SprayValve
        self.clock = clock
        self.lock = threading.RLock()
        self.client = ActionClient(node, SprayValve, action_name)
        self.subscription = node.create_subscription(
            Bool, state_topic, self._on_state,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE))
        self.valve_open = None
        self.state_time = None
        self.active = False
        self.goal_handle = None
        self.cancel_pending = False
        self.cancel_requested = False
        self.remaining = None
        self.feedback_time = None
        self.message = ''
        self.generation = 0

    def _on_state(self, msg):
        with self.lock:
            self.valve_open = bool(msg.data)
            self.state_time = self.clock()

    def snapshot(self):
        with self.lock:
            fresh = (self.state_time is not None
                     and 0 <= self.clock() - self.state_time <= 0.6)
            state = ('OPEN' if self.valve_open else 'CLOSED') if fresh else 'UNKNOWN'
            message = self.message
            if not self.client.server_is_ready():
                message = 'SERVER UNAVAILABLE'
            feedback_fresh = (self.feedback_time is not None
                              and 0 <= self.clock() - self.feedback_time <= 0.6)
            remaining = self.remaining if feedback_fresh else None
            return state, self.active, remaining, message

    def start(self, mode):
        with self.lock:
            if self.active:
                return
            if not self.client.server_is_ready():
                self.message = 'SERVER UNAVAILABLE'
                return
            goal = self.action_type.Goal()
            goal.spray_mode = mode
            goal.duration_sec = 0.0
            self.generation += 1
            generation = self.generation
            self.active = True
            self.goal_handle = None
            self.cancel_pending = False
            self.cancel_requested = False
            self.remaining = None
            self.feedback_time = None
            self.message = 'REQUESTING'
            try:
                future = self.client.send_goal_async(
                    goal, feedback_callback=lambda msg: self._on_feedback(generation, msg))
                future.add_done_callback(lambda future: self._on_goal(generation, future))
            except Exception as exc:
                self.message = 'GOAL UNKNOWN'
                self.node.get_logger().error(f'Spray request failed: {exc}')

    def cancel(self):
        with self.lock:
            if not self.active:
                return
            if self.goal_handle is None:
                self.cancel_pending = True
                self.message = 'CANCEL PENDING'
            else:
                self._send_cancel()

    def _on_goal(self, generation, future):
        with self.lock:
            if generation != self.generation or not self.active:
                return
            try:
                handle = future.result()
            except Exception as exc:
                self.message = 'GOAL UNKNOWN'
                self.node.get_logger().error(f'Spray goal response failed: {exc}')
                return
            if not handle.accepted:
                self.active = False
                self.cancel_pending = False
                self.message = 'GOAL REJECTED'
                return
            self.goal_handle = handle
            self.message = 'ACTIVE'
            try:
                handle.get_result_async().add_done_callback(
                    lambda future: self._on_result(generation, future))
            except Exception as exc:
                self.message = 'RESULT UNKNOWN'
                self.node.get_logger().error(f'Spray result request failed: {exc}')
            if self.cancel_pending:
                self._send_cancel()

    def _send_cancel(self):
        if self.cancel_requested:
            return
        self.cancel_requested = True
        self.message = 'CANCEL REQUESTED'
        generation = self.generation
        try:
            self.goal_handle.cancel_goal_async().add_done_callback(
                lambda future: self._on_cancel(generation, future))
        except Exception as exc:
            self.cancel_requested = False
            self.message = 'CANCEL FAILED'
            self.node.get_logger().error(f'Spray cancel request failed: {exc}')

    def _on_cancel(self, generation, future):
        with self.lock:
            if generation != self.generation or not self.active:
                return
            try:
                accepted = bool(future.result().goals_canceling)
            except Exception as exc:
                self.cancel_requested = False
                self.message = 'CANCEL FAILED'
                self.node.get_logger().error(f'Spray cancel response failed: {exc}')
                return
            if not accepted and self.active:
                self.cancel_requested = False
                self.message = 'CANCEL REJECTED'

    def _on_feedback(self, generation, msg):
        remaining = float(msg.feedback.remaining_time)
        with self.lock:
            if (generation == self.generation and self.active
                    and math.isfinite(remaining) and remaining >= 0):
                self.remaining = remaining
                self.feedback_time = self.clock()

    def _on_result(self, generation, future):
        with self.lock:
            if generation != self.generation or not self.active:
                return
            try:
                result = future.result()
            except Exception as exc:
                self.message = 'RESULT UNKNOWN'
                self.node.get_logger().error(f'Spray result failed: {exc}')
                return
            self.active = False
            self.goal_handle = None
            self.cancel_pending = False
            self.cancel_requested = False
            self.remaining = None
            self.feedback_time = None
            self.message = {
                GoalStatus.STATUS_SUCCEEDED: 'COMPLETE',
                GoalStatus.STATUS_CANCELED: 'CANCELED',
            }.get(result.status, 'ACTION FAILED')
            if result.status not in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED):
                self.node.get_logger().error(
                    f'Spray action failed: {result.result.message}')


def draw_spray_panel(canvas, front_w, front_h, snapshot, available=True):
    """Draw icons and return the panel and button hitboxes."""
    w, h = min(front_w, canvas.shape[1]), min(front_h, canvas.shape[0])
    if w < 280 or h < 240:
        return None, {}
    x, y = 4, 29
    panel = (x, y, x + 136, y + 69)
    state, active, remaining, message = snapshot
    cv2.rectangle(canvas, (x, y), (panel[2], panel[3]), (30, 30, 30), -1)
    cv2.putText(canvas, 'SPRAY ' + state, (x + 4, y + 13),
                cv2.FONT_HERSHEY_SIMPLEX, 0.36, (235, 235, 235), 1, cv2.LINE_AA)
    hits = {}
    for i, name in enumerate(('square', 'circle', 'triangle', 'cancel')):
        bx, by = x + 5 + i * 31, y + 18
        hits[name] = (bx, by, bx + 26, by + 23)
        enabled = available and (active if name == 'cancel' else not active)
        color = (220, 220, 220) if enabled else (110, 110, 110)
        cv2.rectangle(canvas, (bx, by), (bx + 26, by + 23), (65, 65, 65), -1)
        cv2.rectangle(canvas, (bx, by), (bx + 26, by + 23), color, 1)
        cx, cy = bx + 13, by + 11
        if name == 'square':
            cv2.rectangle(canvas, (cx - 6, cy - 6), (cx + 6, cy + 6), color, 2)
        elif name == 'circle':
            cv2.circle(canvas, (cx, cy), 6, color, 2)
        elif name == 'triangle':
            cv2.polylines(canvas, [np.array([[cx, cy - 7], [cx - 7, cy + 6],
                                            [cx + 7, cy + 6]], dtype=np.int32)],
                          True, color, 2)
        else:
            cv2.line(canvas, (cx - 6, cy - 6), (cx + 6, cy + 6), color, 2)
            cv2.line(canvas, (cx + 6, cy - 6), (cx - 6, cy + 6), color, 2)
    detail = f'{remaining:.1f}s ' if remaining is not None else ''
    detail += message
    cv2.putText(canvas, detail[:23], (x + 4, y + 59),
                cv2.FONT_HERSHEY_SIMPLEX, 0.30, (0, 220, 255), 1, cv2.LINE_AA)
    return panel, hits
