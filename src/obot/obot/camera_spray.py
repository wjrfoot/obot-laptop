"""Viewer-side spray action client and operator control rendering."""
from dataclasses import dataclass
import math
import threading
import time

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
import cv2
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


MODE_NAMES = ('circle', 'square', 'triangle')
MODE_INDEX = {name: index for index, name in enumerate(MODE_NAMES)}
MODE_PARAMETER = {
    'circle': 'circle_duration',
    'square': 'square_duration',
    'triangle': 'triangle_duration',
}


@dataclass(frozen=True)
class SpraySnapshot:
    state: str
    active: bool
    remaining: float | None
    message: str
    durations: dict[str, float | None]


class SprayControl:
    def __init__(self, node, clock=time.monotonic, action_name='/obot/spray',
                 state_topic='/obot/valve/state', config_node_name='valve_server'):
        from obot_valve_interfaces.action import SprayValve

        self.node = node
        self.action_type = SprayValve
        self.clock = clock
        self.lock = threading.RLock()
        self.client = ActionClient(node, SprayValve, action_name)
        self.cancel_all_client = node.create_client(
            CancelGoal, action_name.rstrip('/') + '/_action/cancel_goal')
        self.config_client = node.create_client(
            GetParameters, f'/{config_node_name}/get_parameters')
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
        self.durations = {name: None for name in MODE_NAMES}
        self.config_message = 'SPRAY CONFIG UNAVAILABLE'
        self._config_request = None
        self._last_config_request = 0.0

    def _on_state(self, msg):
        with self.lock:
            self.valve_open = bool(msg.data)
            self.state_time = self.clock()

    def _schedule_config_refresh(self):
        now = self.clock()
        with self.lock:
            if self._config_request is not None:
                return
            if now - self._last_config_request < 1.0:
                return
            self._last_config_request = now
            if not self.config_client.service_is_ready():
                return
            request = GetParameters.Request()
            request.names = [MODE_PARAMETER[name] for name in MODE_NAMES]
            try:
                future = self.config_client.call_async(request)
            except RuntimeError as exc:
                self.node.get_logger().error(
                    f'Spray duration query failed: {exc}')
                return
            self._config_request = future
            future.add_done_callback(self._on_config)

    def _on_config(self, future):
        with self.lock:
            if future is not self._config_request:
                return
            self._config_request = None
        try:
            response = future.result()
            if len(response.values) != len(MODE_NAMES):
                raise ValueError('incomplete spray duration response')
            durations = {}
            for name, value in zip(MODE_NAMES, response.values):
                if value.type not in (
                        ParameterType.PARAMETER_DOUBLE,
                        ParameterType.PARAMETER_INTEGER):
                    raise ValueError(
                        f'{MODE_PARAMETER[name]} was not numeric')
                duration = float(value.double_value if value.type == ParameterType.PARAMETER_DOUBLE
                                 else value.integer_value)
                if not math.isfinite(duration) or duration < 0:
                    raise ValueError(
                        f'{MODE_PARAMETER[name]} has invalid value {duration}')
                durations[name] = duration
        except Exception as exc:
            with self.lock:
                self.config_message = 'SPRAY CONFIG ERROR'
            self.node.get_logger().error(
                f'Spray duration query failed: {exc}')
            return
        with self.lock:
            self.durations = durations
            self.config_message = ''

    def mode_enabled(self, name):
        with self.lock:
            duration = self.durations.get(name)
            return duration is not None and duration > 0.0

    def snapshot(self):
        self._schedule_config_refresh()
        with self.lock:
            fresh = (self.state_time is not None
                     and 0 <= self.clock() - self.state_time <= 0.6)
            state = ('OPEN' if self.valve_open else 'CLOSED') if fresh else 'UNKNOWN'
            message = self.message
            if self.config_message:
                message = self.config_message
            if not self.client.server_is_ready():
                message = 'SERVER UNAVAILABLE'
            feedback_fresh = (self.feedback_time is not None
                              and 0 <= self.clock() - self.feedback_time <= 0.6)
            remaining = self.remaining if feedback_fresh else None
            return SpraySnapshot(
                state=state, active=self.active, remaining=remaining,
                message=message, durations=dict(self.durations))

    def start(self, mode):
        if mode not in (0, 1, 2):
            return
        mode_name = MODE_NAMES[mode]
        with self.lock:
            if self.active:
                return
            duration = self.durations.get(mode_name)
            if duration is None:
                self.message = 'SPRAY CONFIG UNAVAILABLE'
                return
            if duration <= 0.0:
                self.message = f'{mode_name.upper()} DISABLED'
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

    def cancel_all(self):
        """Cancel all accepted spray goals, including goals from other clients."""
        self.cancel()
        if not self.cancel_all_client.service_is_ready():
            with self.lock:
                self.message = 'GLOBAL CANCEL UNAVAILABLE'
            self.node.get_logger().error('Global spray cancellation unavailable')
            return
        try:
            future = self.cancel_all_client.call_async(CancelGoal.Request())
            future.add_done_callback(self._on_cancel_all)
        except RuntimeError as exc:
            with self.lock:
                self.message = 'GLOBAL CANCEL FAILED'
            self.node.get_logger().error(f'Global spray cancellation failed: {exc}')

    def _on_cancel_all(self, future):
        try:
            response = future.result()
            if response.return_code not in (
                    CancelGoal.Response.ERROR_NONE,
                    CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID):
                raise RuntimeError(f'cancel error code {response.return_code}')
        except Exception as exc:
            with self.lock:
                self.message = 'GLOBAL CANCEL FAILED'
            self.node.get_logger().error(f'Global spray cancellation failed: {exc}')

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


def draw_spray_panel(canvas, front_w, front_h, snapshot: SpraySnapshot, available=True):
    """Draw the spray controls and return panel and button hitboxes."""
    w, h = min(front_w, canvas.shape[1]), min(front_h, canvas.shape[0])
    if w < 560 or h < 300:
        return None, {}
    x, y = 8, 12
    panel = (x, y, x + 340, y + 124)
    cv2.rectangle(canvas, (x, y), (panel[2], panel[3]), (30, 30, 30), -1)
    cv2.rectangle(canvas, (x, y), (panel[2], panel[3]), (200, 200, 200), 2)
    cv2.putText(canvas, 'SPRAY ' + snapshot.state, (x + 10, y + 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.78, (235, 235, 235), 2, cv2.LINE_AA)
    hits = {}
    for index, name in enumerate(('square', 'circle', 'triangle', 'cancel')):
        bx, by = x + 10 + index * 82, y + 40
        hits[name] = (bx, by, bx + 74, by + 48)
        if name == 'cancel':
            enabled = available and snapshot.active
            label = 'STOP'
        else:
            duration = snapshot.durations.get(name)
            enabled = available and not snapshot.active and duration is not None and duration > 0.0
            label = '--' if duration is None else f'{duration:.1f}s'
        fill = (45, 75, 200) if name == 'cancel' and enabled else (
            (40, 125, 40) if enabled else (70, 70, 70))
        edge = (210, 210, 210) if enabled else (120, 120, 120)
        cv2.rectangle(canvas, (bx, by), (bx + 74, by + 48), fill, -1)
        cv2.rectangle(canvas, (bx, by), (bx + 74, by + 48), edge, 2)
        cv2.putText(canvas, name.upper(), (bx + 8, by + 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, edge, 1, cv2.LINE_AA)
        cv2.putText(canvas, label, (bx + 8, by + 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.62, edge, 2, cv2.LINE_AA)
    detail = f'{snapshot.remaining:.1f}s ' if snapshot.remaining is not None else ''
    detail += snapshot.message
    cv2.putText(canvas, detail[:36], (x + 10, y + 112),
                cv2.FONT_HERSHEY_SIMPLEX, 0.53, (0, 220, 255), 1, cv2.LINE_AA)
    return panel, hits
