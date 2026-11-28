import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock
import uuid

from action_msgs.msg import GoalStatus
import cv2
import numpy as np

from obot.camera_spray import draw_spray_panel, SprayControl, SpraySnapshot
from obot.dual_compressed_viewer import DualCompressedViewer
from obot_valve_interfaces.action import SprayValve
import pytest
from rcl_interfaces.msg import ParameterType
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class Future:
    def __init__(self):
        self.callback = None

    def add_done_callback(self, callback):
        self.callback = callback

    def finish(self, value=None, error=None):
        self.result = Mock(side_effect=error) if error else Mock(return_value=value)
        self.callback(self)


@pytest.fixture
def spray(monkeypatch):
    node = Mock()
    clock = Mock(return_value=100.0)
    client = Mock()
    client.server_is_ready.return_value = True
    client.send_goal_async.return_value = Future()
    cancel_client = Mock()
    config_client = Mock()
    config_client.service_is_ready.return_value = False
    node.create_client.side_effect = [cancel_client, config_client]
    monkeypatch.setattr('obot.camera_spray.ActionClient', lambda *args: client)
    control = SprayControl(node, clock)
    control.durations = {'circle': 4.0, 'square': 2.0, 'triangle': 0.0}
    control.config_message = ''
    return control, client, node, clock


def test_start_uses_server_duration_goal_and_rejects_disabled_mode(spray):
    control, client, _, _ = spray
    control.start(2)
    assert control.snapshot().message == 'TRIANGLE DISABLED'
    client.send_goal_async.assert_not_called()
    control.start(1)
    goal = client.send_goal_async.call_args.args[0]
    assert (goal.spray_mode, goal.duration_sec) == (1, 0.0)


def test_snapshot_returns_structured_state_and_durations(spray):
    control, _, _, _ = spray
    snap = control.snapshot()
    assert isinstance(snap, SpraySnapshot)
    assert snap.durations == {'circle': 4.0, 'square': 2.0, 'triangle': 0.0}


def test_global_cancel_requests_action_cancel_all(spray):
    control, _, _, _ = spray
    client = control.cancel_all_client
    client.service_is_ready.return_value = True
    future = Future()
    client.call_async.return_value = future
    control.cancel_all()
    request = client.call_async.call_args.args[0]
    assert not any(request.goal_info.goal_id.uuid)
    future.finish(SimpleNamespace(return_code=0))


def _snap_tuple(control):
    snap = control.snapshot()
    return snap.state, snap.active, snap.remaining, snap.message


@pytest.mark.parametrize(('mode', 'expected'), [(0, 0), (1, 1)])
def test_spray_modes_use_server_default_duration_and_are_exclusive(
        spray, mode, expected):
    control, client, _, _ = spray
    control.start(mode)
    goal = client.send_goal_async.call_args.args[0]
    assert (goal.spray_mode, goal.duration_sec) == (expected, 0.0)
    control.start(mode)
    assert client.send_goal_async.call_count == 1
    assert control.snapshot().active


def test_cancel_before_acceptance_and_terminal_result(spray):
    control, client, _, _ = spray
    result_future, cancel_future = Future(), Future()
    handle = Mock(accepted=True)
    handle.get_result_async.return_value = result_future
    handle.cancel_goal_async.return_value = cancel_future
    control.start(1)
    control.cancel()
    assert control.snapshot().message == 'CANCEL PENDING'
    client.send_goal_async.return_value.finish(handle)
    handle.cancel_goal_async.assert_called_once()
    control.cancel()
    handle.cancel_goal_async.assert_called_once()
    cancel_future.finish(SimpleNamespace(goals_canceling=[1]))
    result_future.finish(SimpleNamespace(
        status=GoalStatus.STATUS_CANCELED,
        result=SimpleNamespace(message='canceled')))
    assert _snap_tuple(control) == ('UNKNOWN', False, None, 'CANCELED')


def test_rejection_and_server_unavailable(spray):
    control, client, _, _ = spray
    client.server_is_ready.return_value = False
    control.start(0)
    client.send_goal_async.assert_not_called()
    assert control.snapshot().message == 'SERVER UNAVAILABLE'
    client.server_is_ready.return_value = True
    control.start(0)
    client.send_goal_async.return_value.finish(SimpleNamespace(accepted=False))
    assert _snap_tuple(control) == ('UNKNOWN', False, None, 'GOAL REJECTED')


def test_send_error_keeps_unknown_goal_locked(spray):
    control, client, node, _ = spray
    client.send_goal_async.side_effect = RuntimeError('transport failure')
    control.start(0)
    assert _snap_tuple(control) == ('UNKNOWN', True, None, 'GOAL UNKNOWN')
    control.start(1)
    assert client.send_goal_async.call_count == 1
    node.get_logger().error.assert_called()


def test_missing_and_stale_state_and_feedback(spray):
    control, client, node, clock = spray
    assert control.snapshot().state == 'UNKNOWN'
    node.create_subscription.assert_called_once()
    assert node.create_subscription.call_args.args[3].durability.name == 'VOLATILE'
    control._on_state(SimpleNamespace(data=True))
    control.start(0)
    client.send_goal_async.call_args.kwargs['feedback_callback'](
        SimpleNamespace(feedback=SimpleNamespace(remaining_time=1.2)))
    assert _snap_tuple(control)[:3] == ('OPEN', True, 1.2)
    clock.return_value = 100.7
    assert _snap_tuple(control)[:3] == ('UNKNOWN', True, None)
    control._on_state(SimpleNamespace(data=False))
    assert control.snapshot().state == 'CLOSED'


def test_result_failure_does_not_claim_goal_finished(spray):
    control, client, node, _ = spray
    result_future = Future()
    handle = Mock(accepted=True)
    handle.get_result_async.return_value = result_future
    control.start(0)
    client.send_goal_async.return_value.finish(handle)
    result_future.finish(error=RuntimeError('lost'))
    assert control.snapshot().active
    assert control.snapshot().message == 'RESULT UNKNOWN'
    node.get_logger().error.assert_called()


def test_cancel_rejection_can_be_retried(spray):
    control, client, _, _ = spray
    cancel_future = Future()
    handle = Mock(accepted=True)
    handle.get_result_async.return_value = Future()
    handle.cancel_goal_async.return_value = cancel_future
    control.start(0)
    client.send_goal_async.return_value.finish(handle)
    control.cancel()
    cancel_future.finish(SimpleNamespace(goals_canceling=[]))
    assert control.snapshot().message == 'CANCEL REJECTED'
    control.cancel()
    assert handle.cancel_goal_async.call_count == 2


def test_late_old_callbacks_do_not_change_next_goal(spray):
    control, client, _, _ = spray
    result_future, cancel_future = Future(), Future()
    handle = Mock(accepted=True)
    handle.get_result_async.return_value = result_future
    handle.cancel_goal_async.return_value = cancel_future
    control.start(0)
    first_feedback = client.send_goal_async.call_args.kwargs['feedback_callback']
    client.send_goal_async.return_value.finish(handle)
    control.cancel()
    result_future.finish(SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED,
        result=SimpleNamespace(message='done')))
    client.send_goal_async.return_value = Future()
    control.start(1)
    cancel_future.finish(SimpleNamespace(goals_canceling=[]))
    first_feedback(SimpleNamespace(
        feedback=SimpleNamespace(remaining_time=99.0)))
    assert _snap_tuple(control) == ('UNKNOWN', True, None, 'REQUESTING')


def test_panel_hitboxes_and_small_images():
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    snapshot = SpraySnapshot('CLOSED', False, None, '', {
        'circle': 4.0, 'square': 2.0, 'triangle': 0.0})
    panel, hits = draw_spray_panel(image, 640, 480, snapshot)
    assert set(hits) == {'square', 'circle', 'triangle', 'cancel'}
    assert image[panel[1]:panel[3], panel[0]:panel[2]].any()
    assert draw_spray_panel(image[:200, :280], 280, 200, snapshot) == (None, {})


@pytest.mark.parametrize(('name', 'mode'), [
    ('square', 1), ('circle', 0), ('triangle', 2), ('cancel', None)])
def test_mouse_clicks_call_only_spray_controls(name, mode):
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    snapshot = SpraySnapshot('CLOSED', False, None, '', {
        'circle': 4.0, 'square': 2.0, 'triangle': 0.0})
    panel, hits = draw_spray_panel(image, 640, 480, snapshot)
    viewer = SimpleNamespace(_spray_panel=panel, _spray_hits=hits,
                             _spray_down=None, spray=Mock())
    x, y = hits[name][:2]
    assert DualCompressedViewer._handle_spray_mouse(
        viewer, cv2.EVENT_LBUTTONDOWN, x + 3, y + 3)
    assert DualCompressedViewer._handle_spray_mouse(
        viewer, cv2.EVENT_LBUTTONUP, x + 3, y + 3)
    if mode is None:
        viewer.spray.cancel.assert_called_once()
        viewer.spray.start.assert_not_called()
    else:
        viewer.spray.start.assert_called_once_with(mode)
        viewer.spray.cancel.assert_not_called()


def test_release_without_press_or_outside_button_never_sprays():
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    snapshot = SpraySnapshot('CLOSED', False, None, '', {
        'circle': 4.0, 'square': 2.0, 'triangle': 0.0})
    panel, hits = draw_spray_panel(image, 640, 480, snapshot)
    viewer = SimpleNamespace(_spray_panel=panel, _spray_hits=hits,
                             _spray_down=None, spray=Mock())
    x, y = hits['square'][:2]
    DualCompressedViewer._handle_spray_mouse(
        viewer, cv2.EVENT_LBUTTONUP, x + 3, y + 3)
    DualCompressedViewer._handle_spray_mouse(
        viewer, cv2.EVENT_LBUTTONDOWN, x + 3, y + 3)
    DualCompressedViewer._handle_spray_mouse(
        viewer, cv2.EVENT_LBUTTONUP, 500, 300)
    viewer.spray.start.assert_not_called()


def test_spray_duration_query_accepts_ros_parameter_types(spray):
    control, _, _, _ = spray
    control.config_client.call_async.return_value = Future()
    future = control.config_client.call_async.return_value
    control.config_client.service_is_ready.return_value = True
    control._schedule_config_refresh()
    values = []
    for kind, number in (
            (ParameterType.PARAMETER_DOUBLE, 4.0),
            (ParameterType.PARAMETER_DOUBLE, 2.0),
            (ParameterType.PARAMETER_DOUBLE, 0.0)):
        values.append(SimpleNamespace(type=kind, double_value=number,
                                      integer_value=0))
    future.finish(SimpleNamespace(values=values))
    assert control.durations == {'circle': 4.0, 'square': 2.0, 'triangle': 0.0}
    assert not control.config_message


def test_spray_duration_query_rejects_non_numeric_and_reports_error(spray):
    control, _, node, _ = spray
    control.config_client.call_async.return_value = Future()
    control.config_client.service_is_ready.return_value = True
    control._schedule_config_refresh()
    control._config_request.finish(SimpleNamespace(values=[
        SimpleNamespace(type=0, double_value=0.0, integer_value=0)] * 3))
    assert control.config_message == 'SPRAY CONFIG ERROR'
    node.get_logger().error.assert_called()


@pytest.mark.parametrize('handler', ['_on_mouse', '_on_front_mouse'])
def test_spray_click_does_not_select_camera_target(handler):
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    snapshot = SpraySnapshot('CLOSED', False, None, '', {
        'circle': 4.0, 'square': 2.0, 'triangle': 0.0})
    panel, hits = draw_spray_panel(image, 640, 480, snapshot)
    spray = Mock()
    spray.mode_enabled.side_effect = lambda name: name != 'triangle'
    viewer = SimpleNamespace(
        _spray_panel=panel, _spray_hits=hits, _spray_down=None, spray=spray,
        _handle_motion_mouse=Mock(return_value=False),
        _click_handle_button=Mock(), _select_front_target=Mock(),
        _on_dashboard_mouse=Mock(), front_display_size=(640, 480),
        _directional_hits={})
    viewer._handle_spray_mouse = DualCompressedViewer._handle_spray_mouse.__get__(viewer)
    x, y = hits['square'][:2]
    callback = getattr(DualCompressedViewer, handler)
    callback(viewer, cv2.EVENT_LBUTTONDOWN, x + 3, y + 3, 0, None)
    callback(viewer, cv2.EVENT_LBUTTONUP, x + 3, y + 3, 0, None)
    spray.start.assert_called_once_with(1)
    viewer._click_handle_button.assert_not_called()
    viewer._select_front_target.assert_not_called()
    viewer._on_dashboard_mouse.assert_not_called()


def test_real_action_handoff_with_isolated_mock_server_and_global_cancel():
    name = uuid.uuid4().hex
    rclpy.init()
    server_node = Node('test_spray_server_' + name)
    owner_node = Node('test_spray_owner_' + name)
    stopper_node = Node('test_spray_stopper_' + name)
    group = ReentrantCallbackGroup()
    received = []

    def execute(handle):
        received.append((handle.request.spray_mode, handle.request.duration_sec))
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline and not handle.is_cancel_requested:
            time.sleep(0.01)
        result = SprayValve.Result()
        if handle.is_cancel_requested:
            handle.canceled()
            result.canceled = True
        else:
            handle.abort()
            result.message = 'Test cancellation timed out'
        return result

    action_name = '/test_spray_' + name
    server = ActionServer(
        server_node, SprayValve, action_name, execute,
        callback_group=group,
        cancel_callback=lambda _: CancelResponse.ACCEPT)
    owner = SprayControl(owner_node, action_name=action_name,
                         state_topic='/test_valve_owner_' + name)
    stopper = SprayControl(stopper_node, action_name=action_name,
                           state_topic='/test_valve_stopper_' + name)
    owner.durations['square'] = 2.0
    owner.config_message = ''
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (server_node, owner_node, stopper_node):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin)
    thread.start()

    def wait_for(predicate):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.01)
        pytest.fail('Timed out waiting for mock spray action')

    try:
        wait_for(owner.client.server_is_ready)
        wait_for(stopper.cancel_all_client.service_is_ready)
        owner.start(1)
        wait_for(lambda: len(received) == 1)
        assert received == [(1, 0.0)]
        stopper.cancel_all()
        wait_for(lambda: owner.snapshot().message == 'CANCELED')
        assert not owner.snapshot().active
    finally:
        executor.shutdown()
        thread.join(timeout=3)
        server.destroy()
        for control in (owner, stopper):
            control.client.destroy()
            control.cancel_all_client.destroy()
            control.config_client.destroy()
            control.subscription.destroy()
        for node in (server_node, owner_node, stopper_node):
            node.destroy_node()
        rclpy.shutdown()


def test_panel_draws_duration_labels_and_disable_state():
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    snapshot = SpraySnapshot(
        state='CLOSED',
        active=False,
        remaining=None,
        message='',
        durations={'circle': 4.0, 'square': 2.0, 'triangle': 0.0},
    )
    panel, hits = draw_spray_panel(image, 640, 480, snapshot)
    assert panel is not None
    assert set(hits) == {'square', 'circle', 'triangle', 'cancel'}
    assert image[panel[1]:panel[3], panel[0]:panel[2]].any()


def test_mouse_clicks_ignore_disabled_mode_and_allow_enabled():
    panel = (8, 12, 348, 136)
    hits = {
        'square': (18, 52, 92, 100),
        'circle': (100, 52, 174, 100),
        'triangle': (182, 52, 256, 100),
        'cancel': (264, 52, 338, 100),
    }
    spray = Mock()
    spray.mode_enabled.side_effect = lambda name: name != 'triangle'
    viewer = SimpleNamespace(_spray_panel=panel, _spray_hits=hits,
                             _spray_down=None, spray=spray)
    for name in ('square', 'triangle'):
        x, y = hits[name][:2]
        DualCompressedViewer._handle_spray_mouse(
            viewer, cv2.EVENT_LBUTTONDOWN, x + 3, y + 3)
        DualCompressedViewer._handle_spray_mouse(
            viewer, cv2.EVENT_LBUTTONUP, x + 3, y + 3)
    spray.start.assert_called_once_with(1)
