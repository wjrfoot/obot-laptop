from types import SimpleNamespace
from unittest.mock import Mock
import threading
import time
import uuid

import cv2
import numpy as np
import pytest
from action_msgs.msg import GoalStatus
from obot_valve_interfaces.action import SprayValve
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool

from obot.camera_spray import SprayControl, draw_spray_panel
from obot.dual_compressed_viewer import DualCompressedViewer


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
    monkeypatch.setattr('obot.camera_spray.ActionClient', lambda *args: client)
    return SprayControl(node, clock), client, node, clock


@pytest.mark.parametrize(('mode', 'expected'), [(0, 0), (1, 1), (2, 2)])
def test_spray_modes_use_server_duration_and_are_exclusive(spray, mode, expected):
    control, client, _, _ = spray
    control.start(mode)
    goal = client.send_goal_async.call_args.args[0]
    assert (goal.spray_mode, goal.duration_sec) == (expected, 0.0)
    control.start(mode)
    assert client.send_goal_async.call_count == 1
    assert control.snapshot()[1:]
    assert control.snapshot()[1] is True


def test_cancel_before_acceptance_and_terminal_result(spray):
    control, client, _, _ = spray
    result_future, cancel_future = Future(), Future()
    handle = Mock(accepted=True)
    handle.get_result_async.return_value = result_future
    handle.cancel_goal_async.return_value = cancel_future
    control.start(1)
    control.cancel()
    assert control.snapshot()[3] == 'CANCEL PENDING'
    client.send_goal_async.return_value.finish(handle)
    handle.cancel_goal_async.assert_called_once()
    control.cancel()
    handle.cancel_goal_async.assert_called_once()
    cancel_future.finish(SimpleNamespace(goals_canceling=[1]))
    result_future.finish(SimpleNamespace(
        status=GoalStatus.STATUS_CANCELED, result=SimpleNamespace(message='canceled')))
    assert control.snapshot() == ('UNKNOWN', False, None, 'CANCELED')


def test_rejection_and_server_unavailable(spray):
    control, client, _, _ = spray
    client.server_is_ready.return_value = False
    control.start(0)
    client.send_goal_async.assert_not_called()
    assert control.snapshot()[3] == 'SERVER UNAVAILABLE'
    client.server_is_ready.return_value = True
    control.start(0)
    client.send_goal_async.return_value.finish(SimpleNamespace(accepted=False))
    assert control.snapshot() == ('UNKNOWN', False, None, 'GOAL REJECTED')


def test_send_error_keeps_unknown_goal_locked(spray):
    control, client, node, _ = spray
    client.send_goal_async.side_effect = RuntimeError('transport failure')
    control.start(0)
    assert control.snapshot() == ('UNKNOWN', True, None, 'GOAL UNKNOWN')
    control.start(1)
    assert client.send_goal_async.call_count == 1
    node.get_logger().error.assert_called()


def test_missing_and_stale_state_and_feedback(spray):
    control, client, node, clock = spray
    assert control.snapshot()[0] == 'UNKNOWN'
    node.create_subscription.assert_called_once()
    qos = node.create_subscription.call_args.args[3]
    assert qos.durability.name == 'VOLATILE'
    control._on_state(SimpleNamespace(data=True))
    control.start(0)
    client.send_goal_async.call_args.kwargs['feedback_callback'](
        SimpleNamespace(feedback=SimpleNamespace(remaining_time=1.2)))
    assert control.snapshot()[:3] == ('OPEN', True, 1.2)
    clock.return_value = 100.7
    assert control.snapshot()[:3] == ('UNKNOWN', True, None)
    control._on_state(SimpleNamespace(data=False))
    assert control.snapshot()[0] == 'CLOSED'


def test_result_failure_does_not_claim_goal_finished(spray):
    control, client, node, _ = spray
    result_future = Future()
    handle = Mock(accepted=True)
    handle.get_result_async.return_value = result_future
    control.start(0)
    client.send_goal_async.return_value.finish(handle)
    result_future.finish(error=RuntimeError('lost'))
    assert control.snapshot()[1]
    assert control.snapshot()[3] == 'RESULT UNKNOWN'
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
    assert control.snapshot()[3] == 'CANCEL REJECTED'
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
        status=GoalStatus.STATUS_SUCCEEDED, result=SimpleNamespace(message='done')))
    client.send_goal_async.return_value = Future()
    control.start(1)
    cancel_future.finish(SimpleNamespace(goals_canceling=[]))
    first_feedback(SimpleNamespace(feedback=SimpleNamespace(remaining_time=99.0)))
    assert control.snapshot() == ('UNKNOWN', True, None, 'REQUESTING')


def test_panel_icons_hitboxes_and_small_images():
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    panel, hits = draw_spray_panel(image, 640, 480, ('CLOSED', False, None, ''))
    assert set(hits) == {'square', 'circle', 'triangle', 'cancel'}
    assert panel[3] < 480 - 140
    assert image[panel[1]:panel[3], panel[0]:panel[2]].any()
    assert draw_spray_panel(image[:200, :280], 280, 200,
                            ('UNKNOWN', False, None, '')) == (None, {})
    assert panel[2] <= 140


@pytest.mark.parametrize(('name', 'mode'), [
    ('square', 1), ('circle', 0), ('triangle', 2), ('cancel', None)])
def test_mouse_clicks_call_only_spray_controls(name, mode):
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    panel, hits = draw_spray_panel(image, 640, 480, ('CLOSED', False, None, ''))
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
    panel, hits = draw_spray_panel(image, 640, 480, ('CLOSED', False, None, ''))
    v = SimpleNamespace(_spray_panel=panel, _spray_hits=hits,
                        _spray_down=None, spray=Mock())
    x, y = hits['square'][:2]
    DualCompressedViewer._handle_spray_mouse(v, cv2.EVENT_LBUTTONUP, x + 3, y + 3)
    DualCompressedViewer._handle_spray_mouse(v, cv2.EVENT_LBUTTONDOWN, x + 3, y + 3)
    DualCompressedViewer._handle_spray_mouse(v, cv2.EVENT_LBUTTONUP, 500, 300)
    v.spray.start.assert_not_called()


@pytest.mark.parametrize('handler', ['_on_mouse', '_on_front_mouse'])
def test_spray_click_does_not_select_camera_target(handler):
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    panel, hits = draw_spray_panel(image, 640, 480, ('CLOSED', False, None, ''))
    v = SimpleNamespace(_spray_panel=panel, _spray_hits=hits, _spray_down=None,
                        spray=Mock(), _handle_motion_mouse=Mock(return_value=False),
                        _click_handle_button=Mock(), _select_front_target=Mock(),
                        _on_dashboard_mouse=Mock(), front_display_size=(640, 480))
    v._handle_spray_mouse = DualCompressedViewer._handle_spray_mouse.__get__(v)
    x, y = hits['square'][:2]
    callback = getattr(DualCompressedViewer, handler)
    callback(v, cv2.EVENT_LBUTTONDOWN, x + 3, y + 3, 0, None)
    callback(v, cv2.EVENT_LBUTTONUP, x + 3, y + 3, 0, None)
    v.spray.start.assert_called_once_with(1)
    v._click_handle_button.assert_not_called()
    v._select_front_target.assert_not_called()
    v._on_dashboard_mouse.assert_not_called()


def test_real_action_handoff_with_isolated_mock_server():
    name = uuid.uuid4().hex
    rclpy.init()
    server_node = Node('test_spray_server_' + name)
    client_node = Node('test_spray_viewer_' + name)
    group = ReentrantCallbackGroup()
    received = []

    def execute(handle):
        received.append((handle.request.spray_mode, handle.request.duration_sec))
        feedback = SprayValve.Feedback()
        feedback.remaining_time = 3.0
        feedback.valve_open = True
        handle.publish_feedback(feedback)
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
    server = ActionServer(server_node, SprayValve, action_name, execute,
                          callback_group=group,
                          cancel_callback=lambda _: CancelResponse.ACCEPT)
    control = SprayControl(client_node, action_name=action_name,
                           state_topic='/test_valve_' + name)
    state_pub = server_node.create_publisher(Bool, '/test_valve_' + name, 10)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(server_node)
    executor.add_node(client_node)
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
        wait_for(control.client.server_is_ready)
        wait_for(lambda: state_pub.get_subscription_count() > 0)
        state_pub.publish(Bool(data=True))
        wait_for(lambda: control.snapshot()[0] == 'OPEN')
        control.start(1)
        wait_for(lambda: control.snapshot()[2] is not None)
        assert received == [(1, 0.0)]
        control.cancel()
        wait_for(lambda: control.snapshot()[3] == 'CANCELED')
        assert not control.snapshot()[1]
    finally:
        executor.shutdown()
        thread.join(timeout=3)
        server.destroy()
        control.client.destroy()
        server_node.destroy_node()
        client_node.destroy_node()
        rclpy.shutdown()
