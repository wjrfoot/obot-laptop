import time
from types import SimpleNamespace
from unittest.mock import Mock

import cv2
import numpy as np
import pytest
from PyQt5.QtCore import QEvent

from obot.dual_compressed_viewer import (
    DIRECTIONAL_DEFAULTS, DualCompressedViewer, MotionWindowEvents, directional_speeds,
    validate_directional_settings,
)


def settings():
    return {name.removeprefix('directional.'): value
            for name, value in DIRECTIONAL_DEFAULTS.items()}


def viewer():
    v = SimpleNamespace(
        directional_settings=settings(), directional_speed=0.2,
        _manual_button=None, _manual_owned=False, _manual_future=None,
        _manual_kind=None, _slider_dragging=False, _directional_hits={},
        _click_button_hits={}, selected_target=None, click_state='idle',
        click_distance_text='5.0', click_distance_edit=False, click_message='',
        click_enable_motion=True, directional_enable_motion=True,
        _click_future=None, _click_request_kind=None,
        _manual_client=Mock(), _manual_last_send=0.0, _manual_future_started=0.0,
        combined_window='combined', window_left='front', show_side_by_side=True,
        _viewer_id='viewer', _viewer_sequence=0,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=100_000_000_000)))
    for name in ('_viewer_request', '_manual_send', '_manual_stop', '_manual_press',
                 '_manual_tick', '_set_slider', '_handle_motion_mouse'):
        setattr(v, name, getattr(DualCompressedViewer, name).__get__(v))
    return v


def test_panel_is_in_two_halves_and_registers_direction_buttons():
    v = viewer()
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    v.selected_target = (100, 100, -29.2)
    v.click_state = 'selected'
    DualCompressedViewer._click_drive_buttons(v, image, 640, 480)
    assert set(v._click_button_hits) == {'TURN', 'CANCEL'}
    assert set(v._directional_hits) == {'FORWARD', 'REVERSE', 'LEFT', 'RIGHT', 'SLIDER'}
    assert all(rect[2] <= 320 for rect in v._click_button_hits.values())
    assert all(rect[0] >= 320 for rect in v._directional_hits.values())
    assert not image[:340].any()
    for name, color in [('TURN', (35, 135, 35)), ('CANCEL', (35, 35, 180))]:
        x0, y0, _, _ = v._click_button_hits[name]
        assert tuple(image[y0 + 2, x0 + 2]) == color


@pytest.mark.parametrize('enabled', [False, True])
def test_turn_button_respects_click_motion_setting(enabled):
    v = viewer()
    v.click_enable_motion = enabled
    v.selected_target = (100, 100, -29.2)
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


def test_slider_clamps_and_does_not_initiate_motion():
    v = viewer()
    v._directional_hits = {'SLIDER': (10, 0, 110, 20)}
    v._handle_motion_mouse(cv2.EVENT_LBUTTONDOWN, 10, 10, 0)
    assert v.directional_speed == 0.2
    v._handle_motion_mouse(cv2.EVENT_MOUSEMOVE, 200, 10, cv2.EVENT_FLAG_LBUTTON)
    assert v.directional_speed == 2.5
    v._handle_motion_mouse(cv2.EVENT_LBUTTONUP, 200, 10, 0)
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
    assert [c.args[0].command for c in v._manual_client.call_async.call_args_list] == ['MANUAL_BEGIN']
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
    v.selected_target = (1, 2, 3)
    v.click_state = 'driving'
    v._click_future = Mock()
    v._manual_press('LEFT')
    assert v.selected_target is None and v.click_state == 'idle'
    assert v._click_future is None
    assert v._manual_client.call_async.call_args.args[0].command == 'MANUAL_BEGIN'


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
        'MANUAL_BEGIN']
    v._manual_stop()
    assert v._manual_client.call_async.call_args.args[0].command == 'MANUAL_END'


def test_shutdown_stop_sends_end_for_active_motion():
    v = viewer()
    v._manual_button = 'LEFT'
    v._manual_owned = True
    v._manual_stop()
    assert v._manual_client.call_async.call_args.args[0].command == 'MANUAL_END'
    assert not v._manual_owned and v._manual_button is None


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
