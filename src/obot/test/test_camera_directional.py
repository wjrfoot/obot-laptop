import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock

import cv2
import numpy as np
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
        _click_button_hits={}, _fixed_go_hits={},
        selected_target=TargetTrackerState(), click_state='idle',
        click_message='', target_standoff_m=1.0,
        click_enable_motion=True, directional_enable_motion=True,
        _click_future=None, _click_request_kind=None,
        _manual_client=Mock(), _manual_last_send=0.0, _manual_future_started=0.0,
        combined_window='combined', window_left='front', show_side_by_side=True,
        _viewer_id='viewer', _viewer_sequence=0, _motion_session_ready=True,
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
                 '_clear_selected_target'):
        setattr(v, name, getattr(DualCompressedViewer, name).__get__(v))
    return v


def test_panel_is_in_two_halves_and_registers_direction_buttons():
    v = viewer()
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    v.selected_target = TargetTrackerState(status='selected', bearing_deg=-29.2)
    v.click_state = 'selected'
    DualCompressedViewer._click_drive_buttons(v, image, 640, 480)
    assert set(v._click_button_hits) == {'TURN', 'CANCEL', 'GO'}
    assert set(v._fixed_go_hits) == {'2', '4', '6'}
    assert set(v._directional_hits) == {
        'FORWARD', 'REVERSE', 'LEFT', 'RIGHT', 'SLOW', 'MEDIUM', 'FAST'}
    assert all(rect[2] <= 320 for rect in v._click_button_hits.values())
    assert all(rect[0] >= 320 for rect in v._directional_hits.values())
    assert not image[:230].any()
    assert all(rect[3] <= image.shape[0] for rect in (
        *v._click_button_hits.values(), *v._fixed_go_hits.values(),
        *v._directional_hits.values()))
    for name in ('TURN', 'CANCEL'):
        x0, y0, _, _ = v._click_button_hits[name]
        assert tuple(image[y0 + 2, x0 + 2]) != (0, 0, 0)


@pytest.mark.parametrize('enabled', [False, True])
def test_turn_button_respects_click_motion_setting(enabled):
    v = viewer()
    v.click_enable_motion = enabled
    v.selected_target = TargetTrackerState(status='selected', bearing_deg=-29.2)
    v.click_state = 'selected'
    v._click_request = Mock(return_value=True)
    v._click_turn = DualCompressedViewer._click_turn.__get__(v)
    v._click_handle_button = DualCompressedViewer._click_handle_button.__get__(v)
    image = np.zeros((480, 640, 3), dtype=np.uint8)
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
    v._click_go_fixed = DualCompressedViewer._click_go_fixed.__get__(v)
    v._click_go_fixed(6.0)
    v._request_drive.assert_called_once_with(pytest.approx(1.8288))


def test_remaining_distance_uses_odometry_progress():
    v = viewer()
    v.click_state = 'driving'
    v._go_requested_distance_m = 4.0
    v._go_start_pose = (1.0, 2.0, 0.0)
    v._latest_odom_pose = (2.5, 2.0, 0.0)
    v._latest_odom_time = time.monotonic()
    assert DualCompressedViewer._go_remaining_m(v) == pytest.approx(2.5)


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
