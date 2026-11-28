import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock

import cv2
import numpy as np
from obot.camera_spray import SpraySnapshot
from obot.dual_compressed_viewer import (
    DIRECTIONAL_DEFAULTS, directional_speeds, DualCompressedViewer,
    MotionWindowEvents, MovementRequest, TargetTrackerState, TargetTrackingWorker,
    validate_directional_settings,
)
from PyQt5.QtCore import QEvent, Qt
import pytest


def settings():
    return {name.removeprefix('directional.'): value
            for name, value in DIRECTIONAL_DEFAULTS.items()}


def viewer():
    v = SimpleNamespace(
        directional_settings=settings(), directional_speed=0.2,
        speed_presets={'SLOW': 0.2, 'MEDIUM': 0.5, 'FAST': 1.0},
        selected_speed_preset='SLOW', operator_units='m',
        _manual_button=None, _manual_owned=False, _manual_future=None,
        _manual_kind=None, _slider_dragging=False, _directional_hits={},
        _click_button_hits={}, _fixed_go_hits={}, _pressed_click=None, _pending_go=None,
        selected_target=TargetTrackerState(), click_state='idle',
        click_message='', target_standoff_m=1.0,
        click_enable_motion=True, directional_enable_motion=True,
        _click_future=None, _click_request_kind=None,
        _manual_client=Mock(), _manual_last_send=0.0, _manual_future_started=0.0,
        combined_window='combined', window_left='front', show_side_by_side=True,
        _viewer_id='viewer', _viewer_sequence=0, _motion_session_ready=True,
        _motion_result_id='viewer:1', _active_go=None, lidar=None,
        _go_requested_distance_m=None, _go_start_pose=None,
        _go_distance_from_lidar=Mock(return_value=4.0),
        _go_button_text=Mock(return_value='GO 4.0 m'),
        _request_drive=Mock(),
        tracking_worker=Mock(),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=100_000_000_000)))
    for name in ('_viewer_request', '_manual_send', '_manual_stop', '_manual_press',
                 '_manual_tick', '_handle_motion_mouse', '_fresh_center_distance',
                 '_go_distance_from_lidar', '_drive_progress_m', '_go_remaining_m',
                 '_clear_selected_target', '_track_click_press'):
        setattr(v, name, getattr(DualCompressedViewer, name).__get__(v))
    return v


def test_three_panels_are_below_camera_and_register_direction_buttons():
    v = viewer()
    image = np.zeros((720, 640, 3), dtype=np.uint8)
    v.selected_target = TargetTrackerState(status='selected', bearing_deg=-29.2)
    v.click_state = 'selected'
    DualCompressedViewer._click_drive_buttons(v, image, 640, 480)
    assert set(v._click_button_hits) == {'TURN', 'STOP', 'GO'}
    assert set(v._fixed_go_hits) == {'2', '4'}
    assert set(v._directional_hits) == {
        'FORWARD', 'REVERSE', 'LEFT', 'RIGHT', 'SLOW', 'MEDIUM', 'FAST'}
    assert all(rect[2] <= 640 // 3 for rect in v._click_button_hits.values())
    assert all(640 // 3 <= rect[0] < rect[2] <= 2 * 640 // 3
               for rect in v._directional_hits.values())
    assert not image[:480].any()
    assert all(rect[3] <= image.shape[0] for rect in (
        *v._click_button_hits.values(), *v._fixed_go_hits.values(),
        *v._directional_hits.values()))
    for name in ('TURN', 'STOP'):
        x0, y0, _, _ = v._click_button_hits[name]
        assert tuple(image[y0 + 2, x0 + 2]) != (0, 0, 0)


@pytest.mark.parametrize(('width', 'camera_height', 'strip_height'), [
    (640, 480, 160), (960, 720, 240), (320, 240, 80)])
def test_control_panel_geometry_and_hitboxes(width, camera_height, strip_height):
    v = viewer()
    v.front_display_size = (width, camera_height)
    v.spray = Mock()
    v.spray.snapshot.return_value = SpraySnapshot(
        'CLOSED', False, None, '', {'square': 2.0, 'circle': 4.0, 'triangle': 0.0})
    image = np.zeros((camera_height + strip_height, width, 3), dtype=np.uint8)
    DualCompressedViewer._click_drive_buttons(v, image, width, camera_height)
    DualCompressedViewer._draw_spray(v, image)
    first, second = width // 3, 2 * width // 3
    assert v._spray_panel == (second, camera_height, width, image.shape[0])
    assert all(camera_height <= box[1] < box[3] <= image.shape[0]
               for box in (*v._click_button_hits.values(), *v._fixed_go_hits.values(),
                           *v._directional_hits.values(), *v._spray_hits.values()))
    assert all(second <= box[0] < box[2] <= width for box in v._spray_hits.values())
    turn, stop, go = (v._click_button_hits[name] for name in ('TURN', 'STOP', 'GO'))
    fixed = [v._fixed_go_hits[str(amount)] for amount in (2, 4)]
    assert turn[1] == stop[1] and turn[2] <= stop[0]
    assert go[2] == stop[2]
    assert turn[3] < go[1] and go[1] == fixed[0][1]
    assert all(box[1:4:2] == fixed[0][1:4:2] for box in fixed)
    widths = [box[2] - box[0] for box in (*fixed, go)]
    assert widths[0] == widths[1] and widths[2] > widths[0]
    heights = {box[3] - box[1] for box in (
        *v._click_button_hits.values(), *v._fixed_go_hits.values(),
        *v._directional_hits.values(), *v._spray_hits.values())}
    assert len(heights) == 1
    assert heights.pop() <= 30
    arrows = v._directional_hits
    assert arrows['FORWARD'][3] <= arrows['LEFT'][1]
    assert arrows['LEFT'][3] <= arrows['REVERSE'][1]
    assert 0 <= arrows['SLOW'][1] - arrows['REVERSE'][3] <= 4
    assert strip_height / (camera_height + strip_height) == pytest.approx(0.25)
    assert abs(first - (second - first)) <= 1
    assert abs(first - (width - second)) <= 1


def test_stop_and_fixed_go_hitboxes_route_to_existing_handlers():
    v = viewer()
    v._click_turn = Mock()
    v._click_go = Mock()
    v._click_cancel = Mock()
    v._click_go_fixed = Mock()
    image = np.zeros((720, 640, 3), dtype=np.uint8)
    DualCompressedViewer._click_drive_buttons(v, image, 640, 480)
    for name in ('STOP', '2', '4'):
        box = (v._click_button_hits if name == 'STOP' else v._fixed_go_hits)[name]
        assert DualCompressedViewer._click_handle_button(
            v, (box[0] + box[2]) // 2, (box[1] + box[3]) // 2)
    v._click_cancel.assert_called_once()
    assert [call.args[0] for call in v._click_go_fixed.call_args_list] == [2.0, 4.0]


def test_compact_direction_and_speed_hitboxes_route_to_expected_controls():
    v = viewer()
    v._manual_press = Mock()
    DualCompressedViewer._click_drive_buttons(v, np.zeros((640, 640, 3), dtype=np.uint8),
                                              640, 480)
    for name in ('FORWARD', 'LEFT', 'RIGHT', 'REVERSE'):
        x0, y0, x1, y1 = v._directional_hits[name]
        assert v._handle_motion_mouse(
            cv2.EVENT_LBUTTONDOWN, (x0 + x1) // 2, (y0 + y1) // 2, 0)
        assert v._manual_press.call_args.args == (name,)
    x0, y0, x1, y1 = v._directional_hits['FAST']
    assert v._handle_motion_mouse(
        cv2.EVENT_LBUTTONDOWN, (x0 + x1) // 2, (y0 + y1) // 2, 0)
    assert v.selected_speed_preset == 'FAST'


def test_compact_variable_go_label_remains_readable(monkeypatch):
    v = viewer()
    v._go_button_text.return_value = 'GO 4.2 m'
    scales = {}
    original = cv2.putText

    def record(image, text, position, font, scale, *args):
        scales[text] = scale
        return original(image, text, position, font, scale, *args)

    monkeypatch.setattr(cv2, 'putText', record)
    DualCompressedViewer._click_drive_buttons(v, np.zeros((640, 640, 3), dtype=np.uint8),
                                              640, 480)
    assert scales['GO 4.2 m'] >= 0.32


def test_front_camera_lower_area_remains_selectable():
    v = viewer()
    v.front_display_size = (640, 480)
    v._control_bottom = 720
    v._handle_motion_mouse = Mock(return_value=False)
    v._handle_spray_mouse = Mock(return_value=False)
    v._click_handle_button = Mock(return_value=False)
    v._select_front_target = Mock()
    v._on_dashboard_mouse = Mock()
    DualCompressedViewer._on_mouse(v, cv2.EVENT_LBUTTONUP, 100, 450, 0, None)
    v._select_front_target.assert_called_once_with(100, 450)
    DualCompressedViewer._on_mouse(v, cv2.EVENT_LBUTTONUP, 100, 500, 0, None)
    v._select_front_target.assert_called_once()


@pytest.mark.parametrize('enabled', [False, True])
def test_turn_button_respects_click_motion_setting(enabled):
    v = viewer()
    v.click_enable_motion = enabled
    v.selected_target = TargetTrackerState(status='selected', bearing_deg=-29.2)
    v.click_state = 'selected'
    v._click_request = Mock(return_value=True)
    v._click_turn = DualCompressedViewer._click_turn.__get__(v)
    v._click_handle_button = DualCompressedViewer._click_handle_button.__get__(v)
    image = np.zeros((720, 640, 3), dtype=np.uint8)
    DualCompressedViewer._click_drive_buttons(v, image, 640, 480)
    x0, y0, _, _ = v._click_button_hits['TURN']
    assert v._click_handle_button(x0 + 2, y0 + 2)
    if enabled:
        v._click_request.assert_called_once_with('TURN', -29.2)
        assert v.click_state == 'turning'
    else:
        v._click_request.assert_not_called()
        assert v.click_state == 'selected'
        assert v.click_message == 'CLICK MOTION DISABLED'


def test_speed_conversion_and_caps():
    cfg = settings()
    assert directional_speeds(2.5, cfg) == (0.2, 0.35)
    cfg['linear_limit_mps'] = 1.0
    cfg['angular_limit_rps'] = 5.0
    assert directional_speeds(0.5, cfg) == pytest.approx((0.5, 1 / 0.486))


@pytest.mark.parametrize(('name', 'value'), [
    ('min_speed_mps', 2.5), ('max_speed_mps', 0), ('wheel_separation_m', 0),
    ('angular_limit_rps', float('nan')), ('linear_limit_mps', -1),
])
def test_invalid_settings_are_rejected(name, value):
    cfg = settings()
    cfg[name] = value
    with pytest.raises(ValueError):
        validate_directional_settings(cfg)


def test_shared_speed_is_copied_into_click_and_manual_requests():
    v = viewer()
    v.directional_settings['linear_limit_mps'] = 1.0
    v.directional_settings['angular_limit_rps'] = 5.0
    v.directional_speed = 0.5
    click = v._viewer_request('TURN')
    manual = v._viewer_request('MANUAL')
    assert click.linear_speed_mps == manual.linear_speed_mps == 0.5
    assert click.angular_speed_rps == pytest.approx(1.0 / 0.486)
    assert click.angular_speed_rps == manual.angular_speed_rps
    v.directional_speed = 0.8
    assert click.linear_speed_mps == 0.5


def test_speed_buttons_select_and_do_not_initiate_motion():
    v = viewer()
    v._directional_hits = {'MEDIUM': (10, 0, 110, 20)}
    v._handle_motion_mouse(cv2.EVENT_LBUTTONDOWN, 20, 10, 0)
    assert v.directional_speed == 0.5
    assert v.selected_speed_preset == 'MEDIUM'
    v._manual_client.call_async.assert_not_called()


@pytest.mark.parametrize(('button', 'expected'), [
    ('FORWARD', (1, 0)), ('REVERSE', (-1, 0)), ('LEFT', (0, 1)), ('RIGHT', (0, -1)),
])
def test_press_waits_for_ack_and_release_requests_stop(monkeypatch, button, expected):
    v = viewer()
    v._directional_hits = {button: (10, 10, 50, 30)}
    future = Mock()
    future.done.return_value = True
    future.result.return_value = SimpleNamespace(accepted=True, message='ok')
    v._manual_client.call_async.return_value = future
    monkeypatch.setattr('obot.dual_compressed_viewer.window_has_focus', lambda _: True)
    v._handle_motion_mouse(cv2.EVENT_LBUTTONDOWN, 20, 20, cv2.EVENT_FLAG_LBUTTON)
    assert [c.args[0].command for c in v._manual_client.call_async.call_args_list] == [
        'PREPARE']
    future.result.return_value.permit_id = 'permit'
    v._manual_tick()
    assert v._manual_client.call_async.call_args.args[0].command == 'MANUAL_BEGIN'
    v._manual_tick()
    v._manual_tick()
    request = v._manual_client.call_async.call_args.args[0]
    assert request.command == 'MANUAL'
    assert (request.linear_direction, request.angular_direction) == expected
    v._handle_motion_mouse(cv2.EVENT_LBUTTONUP, 20, 20, 0)
    assert v._manual_client.call_async.call_args.args[0].command == 'MANUAL_END'
    assert v._manual_button is None


def test_mouse_leaving_button_and_focus_loss_stop(monkeypatch):
    v = viewer()
    v._directional_hits = {'FORWARD': (10, 10, 50, 30)}
    v._manual_button = 'FORWARD'
    v._manual_owned = True
    v._handle_motion_mouse(cv2.EVENT_MOUSEMOVE, 51, 20, cv2.EVENT_FLAG_LBUTTON)
    assert v._manual_client.call_async.call_args.args[0].command == 'MANUAL_END'
    v._manual_button = 'FORWARD'
    v._manual_owned = True
    monkeypatch.setattr('obot.dual_compressed_viewer.window_has_focus', lambda _: False)
    v._manual_tick()
    assert v._manual_button is None
    assert v._manual_client.call_async.call_args.args[0].command == 'MANUAL_END'


def test_manual_press_clears_pending_click_target_and_future():
    v = viewer()
    v.selected_target = TargetTrackerState(status='selected', bearing_deg=3.0)
    v.click_state = 'driving'
    v._click_future = Mock()
    v._manual_press('LEFT')
    assert v.selected_target.status == 'idle' and v.click_state == 'idle'
    assert v._click_future is None
    assert v._manual_client.call_async.call_args.args[0].command == 'PREPARE'


def test_service_timeout_stops_without_motion(monkeypatch):
    v = viewer()
    v._manual_button = 'FORWARD'
    v._manual_future = Mock()
    v._manual_future.done.return_value = False
    v._manual_future_started = time.monotonic() - 0.3
    monkeypatch.setattr('obot.dual_compressed_viewer.window_has_focus', lambda _: True)
    v._manual_tick()
    assert v._manual_button is None
    assert v._manual_client.call_async.call_args.args[0].command == 'MANUAL_END'


def test_unacknowledged_takeover_never_commands_motion(monkeypatch):
    v = viewer()
    future = Mock()
    future.done.return_value = False
    v._manual_client.call_async.return_value = future
    monkeypatch.setattr('obot.dual_compressed_viewer.window_has_focus', lambda _: True)
    v._manual_press('FORWARD')
    v._manual_tick()
    assert [call.args[0].command for call in v._manual_client.call_async.call_args_list] == [
        'PREPARE']
    v._manual_stop()
    assert v._manual_client.call_async.call_args.args[0].command == 'MANUAL_END'


def test_shutdown_stop_sends_end_for_active_motion():
    v = viewer()
    v._manual_button = 'LEFT'
    v._manual_owned = True
    v._manual_stop()
    assert v._manual_client.call_async.call_args.args[0].command == 'MANUAL_END'
    assert not v._manual_owned and v._manual_button is None


def test_new_viewer_waits_for_stop_and_permit_before_turn():
    v = viewer()
    v._motion_session_ready = False
    ready = Mock()
    ready.done.return_value = True
    ready.result.return_value = SimpleNamespace(accepted=True, permit_id='permit')
    v._manual_client.call_async.return_value = ready
    operation = MovementRequest(v, v._manual_client, v._viewer_request('TURN'))
    assert v._manual_client.call_async.call_args.args[0].command == 'STOP'
    assert not operation.done()
    assert v._manual_client.call_async.call_args.args[0].command == 'PREPARE'
    assert not operation.done()
    sent = v._manual_client.call_async.call_args.args[0]
    assert sent.command == 'TURN' and sent.permit_id == 'permit'
    assert operation.done()


def test_older_controller_cannot_authorize_automatic_turn_continuation():
    v = viewer()
    ready = Mock()
    ready.done.return_value = True
    ready.result.return_value = SimpleNamespace(
        accepted=True, permit_id='permit', message='Movement permit issued')
    v._manual_client.call_async.return_value = ready
    request = v._viewer_request('DRIVE')
    request.permit_command = 'viewer:1'
    operation = MovementRequest(v, v._manual_client, request)
    prepared = v._manual_client.call_async.call_args.args[0]
    assert prepared.command == 'PREPARE' and prepared.permit_id == 'viewer:1'
    assert operation.done()
    assert not operation.result().accepted
    assert 'cannot confirm' in operation.result().message
    assert v._manual_client.call_async.call_count == 1


def test_late_permit_response_is_not_sent_after_manual_release():
    v = viewer()
    pending = Mock()
    pending.done.return_value = False
    v._manual_client.call_async.return_value = pending
    v._manual_press('FORWARD')
    abandoned = v._manual_future
    v._manual_stop()
    pending.done.return_value = True
    pending.result.return_value = SimpleNamespace(accepted=True, permit_id='late')
    assert v._manual_future is None
    assert abandoned.stage == 'prepare'
    assert [call.args[0].command for call in v._manual_client.call_async.call_args_list] == [
        'PREPARE', 'MANUAL_END']


def test_cancel_does_not_wait_for_pending_click_request():
    v = viewer()
    v.click_state = 'turning'
    v._click_future = Mock()
    v._click_request = Mock(return_value=True)
    DualCompressedViewer._click_cancel(v)
    assert v._click_future is None
    v._click_request.assert_called_once_with('STOP')


def test_click_stop_interrupts_manual_and_sends_stop_when_idle():
    v = viewer()
    v._manual_button = 'FORWARD'
    v._manual_owned = True
    v._click_request = Mock(return_value=True)
    DualCompressedViewer._click_cancel(v)
    assert v._manual_button is None and not v._manual_owned
    assert v._manual_client.call_async.call_args.args[0].command == 'MANUAL_END'
    v._click_request.assert_called_once_with('STOP')
    assert v.click_state == 'stopping'


def test_rear_window_click_does_not_trigger_front_controls():
    v = viewer()
    v._on_dashboard_mouse = Mock()
    DualCompressedViewer._on_rear_mouse(v, cv2.EVENT_LBUTTONUP, 30, 20, 0, None)
    v._on_dashboard_mouse.assert_called_once_with(30, 20)
    v._manual_client.call_async.assert_not_called()


@pytest.mark.parametrize('kind', [QEvent.Leave, QEvent.WindowDeactivate, QEvent.Close])
def test_window_events_request_stop(monkeypatch, kind):
    class Widget:
        def window(self):
            return self

        def windowTitle(self):
            return 'combined'

    monkeypatch.setattr('obot.dual_compressed_viewer.QWidget', Widget)
    v = viewer()
    v._manual_button = 'LEFT'
    v._manual_owned = True
    event_filter = MotionWindowEvents(v)
    assert not event_filter.eventFilter(Widget(), QEvent(kind))
    assert v._manual_client.call_async.call_args.args[0].command == 'MANUAL_END'


def test_space_stops_motion_and_spray_and_discards_pending_request():
    v = viewer()
    v._manual_button = 'LEFT'
    v._manual_owned = True
    v.click_state = 'driving'
    v.selected_target = TargetTrackerState(status='selected', bearing_deg=3.0)
    v._click_future = Mock()
    v._click_request = Mock(return_value=True)
    v.spray = Mock()
    DualCompressedViewer._universal_stop(v)
    assert v._manual_client.call_async.call_args.args[0].command == 'MANUAL_END'
    v._click_request.assert_called_once_with('STOP')
    v.spray.cancel_all.assert_called_once()
    assert v._click_future is None and v.selected_target.status == 'idle'
    assert v.click_state == 'stopping'


def test_space_is_handled_once_for_key_autorepeat(monkeypatch):
    class Widget:
        def window(self):
            return self

        def windowTitle(self):
            return 'combined'

    def key_event(repeat):
        return SimpleNamespace(
            type=lambda: QEvent.KeyPress,
            key=lambda: Qt.Key_Space,
            isAutoRepeat=lambda: repeat)

    monkeypatch.setattr('obot.dual_compressed_viewer.QWidget', Widget)
    v = viewer()
    v.window_right = 'rear'
    v._universal_stop = Mock()
    event_filter = MotionWindowEvents(v)
    assert event_filter.eventFilter(Widget(), key_event(False))
    assert event_filter.eventFilter(Widget(), key_event(True))
    v._universal_stop.assert_called_once()


def test_lidar_go_distance_uses_standoff_and_rejects_invalid_samples():
    v = viewer()
    reading = SimpleNamespace(front=SimpleNamespace(distance_m=5.0, held=False))
    v.lidar = SimpleNamespace(snapshot=lambda: SimpleNamespace(status='ok', reading=reading))
    assert DualCompressedViewer._go_distance_from_lidar(v) == pytest.approx(4.0)
    reading.front.held = True
    assert DualCompressedViewer._go_distance_from_lidar(v) is None
    reading.front.held = False
    reading.front.distance_m = 0.8
    assert DualCompressedViewer._go_distance_from_lidar(v) is None


def test_fixed_go_buttons_convert_selected_units():
    v = viewer()
    v.operator_units = 'ft'
    v._begin_go = Mock()
    v._click_go_fixed = DualCompressedViewer._click_go_fixed.__get__(v)
    v._click_go_fixed(6.0)
    v._begin_go.assert_called_once_with(pytest.approx(1.8288))


def test_remaining_distance_uses_odometry_progress():
    v = viewer()
    v.click_state = 'driving'
    v._go_requested_distance_m = 4.0
    v._go_start_pose = (1.0, 2.0, 0.0)
    v._latest_odom_pose = (2.5, 2.0, 0.0)
    v._latest_odom_time = time.monotonic()
    assert DualCompressedViewer._go_remaining_m(v) == pytest.approx(2.5)


def operation_viewer():
    v = viewer()
    v._latest_odom_pose = (0.0, 0.0, 0.0)
    v._latest_odom_time = time.monotonic()
    v._click_last_status = time.monotonic()
    v._click_request = Mock(return_value=True)
    for name in ('_request_drive', '_start_go_drive', '_begin_go', '_click_go',
                 '_click_go_fixed', '_click_turn', '_click_poll', '_click_cancel'):
        setattr(v, name, getattr(DualCompressedViewer, name).__get__(v))
    return v


def status(v, state, message='result=success, request=viewer:1, active=None, pending=0'):
    future = Mock()
    future.done.return_value = True
    future.result.return_value = SimpleNamespace(accepted=True, state=state, message=message)
    v._click_future = future
    v._click_request_kind = 'STATUS'
    v._click_poll()


@pytest.mark.parametrize('distance', [2.0, 4.0, 9.0])
def test_go_without_target_drives_current_heading(distance):
    v = operation_viewer()
    v._begin_go(distance)
    v._click_request.assert_called_once_with('DRIVE', distance=distance, continuation_of='')
    assert v.click_state == 'driving'
    assert v._go_requested_distance_m == distance


def test_combined_lidar_go_refreshes_after_turn_and_freezes_distance():
    v = operation_viewer()
    v.selected_target = TargetTrackerState(status='tracking', bearing_deg=20.0)
    v.click_state = 'selected'
    v._go_distance_from_lidar = Mock(side_effect=[8.0, 3.0])
    v._click_go()
    v._click_request.assert_called_once_with('TURN', 20.0)
    assert v._go_distance_from_lidar.call_count == 0
    v._click_saw_turn_motion = True
    status(v, 'idle')
    assert v._click_request.call_args.args == ('DRIVE',)
    assert v._click_request.call_args.kwargs == {
        'distance': 8.0, 'continuation_of': 'viewer:1'}
    assert v._go_requested_distance_m == 8.0 and v.click_state == 'driving'
    assert v._go_button_text() == 'GO 4.0 m'  # fixture mock
    assert v._go_requested_distance_m == 8.0


@pytest.mark.parametrize('outcome', ['failed', 'missing', 'invalid'])
def test_combined_go_never_drives_after_failed_or_invalid_turn(outcome):
    v = operation_viewer()
    v.selected_target = TargetTrackerState(status='tracking', bearing_deg=20.0)
    v.click_state = 'selected'
    v._click_go()
    v._click_saw_turn_motion = True
    if outcome == 'invalid':
        v._go_distance_from_lidar = Mock(return_value=None)
    response = ('result=failed, reason=interrupted' if outcome == 'failed' else
                'active=None' if outcome == 'missing'
                else 'result=success, request=viewer:1, active=None')
    status(v, 'idle', response)
    assert [call.args[0] for call in v._click_request.call_args_list] == ['TURN']
    assert v.click_state == 'ready'


def test_standalone_turn_finishes_ready_and_allows_new_target():
    v = operation_viewer()
    v.selected_target = TargetTrackerState(status='tracking', bearing_deg=10.0)
    v.click_state = 'selected'
    v._click_turn()
    v._click_saw_turn_motion = True
    status(v, 'idle')
    assert v.click_state == 'ready'
    assert v.selected_target.status == 'idle'
    assert v.click_message == 'Turn complete'
    v._click_go_fixed(2)
    assert v._click_request.call_args.kwargs == {'distance': 2.0, 'continuation_of': ''}


def test_unrelated_success_is_not_used_for_combined_go():
    v = operation_viewer()
    v.selected_target = TargetTrackerState(status='tracking', bearing_deg=10.0)
    v.click_state = 'selected'
    v._click_go_fixed(2)
    v._click_saw_turn_motion = True
    status(v, 'idle', 'result=success, request=other:1, active=None')
    assert [call.args[0] for call in v._click_request.call_args_list] == ['TURN']
    assert 'unconfirmed' in v.click_message


def test_go_label_keeps_updating_during_turn_and_drive():
    v = operation_viewer()
    v._go_distance_from_lidar = Mock(side_effect=[4.0, 3.0])
    v.click_state = 'turning'
    assert DualCompressedViewer._go_button_text(v) == 'GO 4.0 m'
    v.click_state = 'driving'
    assert DualCompressedViewer._go_button_text(v) == 'GO 3.0 m'


@pytest.mark.parametrize(('active_go', 'expected_label'), [('GO', 'GO 4.0 m'), ('2', 'GO-2')])
def test_only_requested_go_button_is_active(monkeypatch, active_go, expected_label):
    import obot.dual_compressed_viewer as module

    buttons = Mock()
    monkeypatch.setattr(module, '_push_button', buttons)
    v = viewer()
    v.click_state = 'driving'
    v._active_go = active_go
    image = np.zeros((720, 640, 3), dtype=np.uint8)
    DualCompressedViewer._click_drive_buttons(v, image, 640, 480)
    active_labels = [call.args[2] for call in buttons.call_args_list
                     if call.kwargs.get('active') and call.args[1][0] < 640 // 3]
    assert active_labels == [expected_label]


@pytest.mark.parametrize('phase', ['turning', 'driving'])
def test_stop_cancels_pending_combined_operation(phase):
    v = operation_viewer()
    v.click_state = phase
    v._pending_go = ('fixed', 4.0)
    v._click_future = Mock()
    v._click_cancel()
    assert v._pending_go is None and v.click_state == 'stopping'
    assert v._click_request.call_args.args == ('STOP',)


def test_tracker_worker_coalesces_frames_and_reports_tracking_loss():
    entered_update = threading.Event()
    release_update = threading.Event()
    updates = []

    class Tracker:
        def init(self, _frame, _box):
            return None

        def update(self, frame):
            value = int(frame[0, 0, 0])
            updates.append(value)
            if value == 1:
                entered_update.set()
                assert release_update.wait(2)
            if value == 8:
                return False, None
            return True, (10, 10, 20, 20)

    worker = TargetTrackingWorker(
        Tracker, lambda x, _y: float(x), stale_timeout_s=1.0)
    try:
        worker.submit_frame(np.zeros((64, 64, 3), dtype=np.uint8))
        worker.select(32, 32, 0.0)
        deadline = time.monotonic() + 1
        while worker.snapshot().status != 'tracking' and time.monotonic() < deadline:
            time.sleep(0.005)
        worker.submit_frame(np.ones((64, 64, 3), dtype=np.uint8))
        assert entered_update.wait(1)
        for value in range(2, 9):
            worker.submit_frame(np.full((64, 64, 3), value, dtype=np.uint8))
        release_update.set()
        deadline = time.monotonic() + 1
        while 8 not in updates and time.monotonic() < deadline:
            time.sleep(0.005)
        assert updates == [1, 8]
        assert worker.snapshot().status == 'lost'
    finally:
        release_update.set()
        worker.close()


def test_tracker_replaces_selected_feature_and_updates_display_position():
    class Tracker:
        def init(self, _frame, box):
            self.box = box
            return None

        def update(self, frame):
            x, y, width, height = self.box
            return True, (x + int(frame[0, 0, 0]), y, width, height)

    worker = TargetTrackingWorker(
        Tracker, lambda x, y: float(x + y / 100), stale_timeout_s=1.0)
    try:
        worker.submit_frame(np.zeros((64, 64, 3), dtype=np.uint8))
        worker.select(40, 40, 0.0)
        deadline = time.monotonic() + 1
        while worker.snapshot().status != 'tracking' and time.monotonic() < deadline:
            time.sleep(0.005)
        worker.submit_frame(np.full((64, 64, 3), 5, dtype=np.uint8))
        deadline = time.monotonic() + 1
        while worker.snapshot().pixel_x != 45 and time.monotonic() < deadline:
            time.sleep(0.005)
        state = worker.snapshot()
        assert state.status == 'tracking'
        assert (state.pixel_x, state.pixel_y) == pytest.approx((45.0, 40.0))
        assert state.bearing_deg == pytest.approx(45.4)

        worker.select(20, 20, -1.0)
        deadline = time.monotonic() + 1
        while (worker.snapshot().status != 'tracking'
               and time.monotonic() < deadline):
            time.sleep(0.005)
        worker.submit_frame(np.full((64, 64, 3), 16, dtype=np.uint8))
        deadline = time.monotonic() + 1
        while worker.snapshot().pixel_x != 36 and time.monotonic() < deadline:
            time.sleep(0.005)
        state = worker.snapshot()
        assert state.status == 'tracking'
        assert (state.pixel_x, state.pixel_y) == pytest.approx((36.0, 20.0))
        assert state.bearing_deg == pytest.approx(36.2)
    finally:
        worker.close()


def test_universal_stop_is_responsive_while_tracker_update_is_blocked():
    entered_update = threading.Event()
    release_update = threading.Event()

    class SlowTracker:
        def init(self, _frame, _box):
            return None

        def update(self, _frame):
            entered_update.set()
            release_update.wait(2)
            return True, (10, 10, 20, 20)

    worker = TargetTrackingWorker(SlowTracker, lambda _x, _y: 0.0)
    try:
        worker.submit_frame(np.zeros((64, 64, 3), dtype=np.uint8))
        worker.select(32, 32, 0.0)
        deadline = time.monotonic() + 1
        while worker.snapshot().status != 'tracking' and time.monotonic() < deadline:
            time.sleep(0.005)
        worker.submit_frame(np.ones((64, 64, 3), dtype=np.uint8))
        assert entered_update.wait(1)
        v = SimpleNamespace(
            _manual_stop=Mock(), _clear_selected_target=Mock(),
            _click_future=None, _click_request_kind=None,
            click_state='driving', _go_requested_distance_m=4.0,
            _go_start_pose=(0.0, 0.0, 0.0), click_message='',
            _click_request=Mock(), spray=Mock())
        started = time.monotonic()
        DualCompressedViewer._universal_stop(v)
        assert time.monotonic() - started < 0.1
        v._manual_stop.assert_called_once()
        v._clear_selected_target.assert_called_once()
        v._click_request.assert_called_once_with('STOP')
        v.spray.cancel_all.assert_called_once()
    finally:
        release_update.set()
        worker.close()
