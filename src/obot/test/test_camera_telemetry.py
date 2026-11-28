import math
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import cv2
from geometry_msgs.msg import TransformStamped
import numpy as np
from obot import dual_compressed_viewer
from obot.camera_dashboard import render_dashboard
from obot.camera_telemetry import Telemetry, wheel_speeds, yaw_from_quaternion
from obot.dual_compressed_viewer import DualCompressedViewer
import pytest
import rclpy
from sensor_msgs.msg import CompressedImage


def test_wheels_use_names_and_reverse_only_left_sign():
    msg = SimpleNamespace(name=['right', 'other', 'left'],
                          velocity=[-math.pi, 0, 2 * math.pi])
    assert wheel_speeds(msg, 'left', 'right') == pytest.approx(
        {'left': -60, 'right': -30})
    msg.velocity = [math.pi]
    partial = wheel_speeds(msg, 'left', 'right')
    assert partial['left'] is None
    assert partial['right'] == pytest.approx(30.0)
    msg.name = ['other']
    assert wheel_speeds(msg, 'left', 'right') == {
        'left': None, 'right': None}


@pytest.mark.parametrize(
    ('left', 'right', 'expected_left', 'expected_right'),
    [(-math.pi, math.pi, 30, 30), (math.pi, -math.pi, -30, -30),
     (0.0, 0.0, 0.0, 0.0)])
def test_wheel_display_forward_and_reverse(
        left, right, expected_left, expected_right):
    msg = SimpleNamespace(name=['left', 'right'], velocity=[left, right])
    assert wheel_speeds(msg, 'left', 'right') == pytest.approx(
        {'left': expected_left, 'right': expected_right})
    assert msg.velocity == [left, right]


def test_quaternion_yaw_and_invalid_input():
    q = SimpleNamespace(x=0.0, y=0.0, z=math.sin(math.pi / 4),
                        w=math.cos(math.pi / 4))
    assert yaw_from_quaternion(q) == pytest.approx(math.pi / 2)
    with pytest.raises(ValueError):
        yaw_from_quaternion(SimpleNamespace(x=0, y=0, z=0, w=0))


def test_reset_rotated_origin_and_independent_frames():
    clock = [10.0]
    state = Telemetry(2.0, clock=lambda: clock[0])
    state.put('pose_odom', (2.0, 3.0, math.pi / 2))
    state.put('pose_map', (100.0, -20.0, 0.0))
    assert state.reset()
    assert state.snapshot()['position'] == pytest.approx((0, 0, 0))
    state.put('pose_odom', (1.0, 5.0, math.pi))
    assert state.snapshot()['position'] == pytest.approx((2, 1, math.pi / 2))
    state.select('map')
    assert state.snapshot()['position'] == (100, -20, 0)
    assert state.reset()
    state.put('pose_map', (101, -22, -math.pi / 2))
    assert state.snapshot()['position'] == pytest.approx((1, -2, -math.pi / 2))
    state.select('odom')
    assert state.snapshot()['position'] == pytest.approx((2, 1, math.pi / 2))


def test_stale_data_and_failed_reset_preserve_origin():
    clock = [1.0]
    state = Telemetry(2, clock=lambda: clock[0])
    state.put('pose_odom', (1, 2, 0))
    state.put('command', (-0.5, 0.2))
    state.put('actual', (0.25, -0.3))
    state.put('voltage_left', 18.6)
    state.put('health', 'NORMAL')
    assert state.reset()
    clock[0] = 4
    data = state.snapshot()
    assert all(data[key] is None for key in
               ('command', 'actual', 'voltage_left', 'health', 'position'))
    assert not state.reset()
    state.put('pose_odom', (2, 2, 0))
    assert state.snapshot()['position'] == pytest.approx((1, 0, 0))
    state.select('map')
    state.put('pose_map', None)
    assert state.snapshot()['position'] is None
    assert not state.reset()


def test_ros_callbacks_extract_motion_and_voltage():
    state = Telemetry(2)
    viewer = SimpleNamespace(telemetry=state)
    msg = SimpleNamespace(twist=SimpleNamespace(
        linear=SimpleNamespace(x=-0.4), angular=SimpleNamespace(z=0.2)))
    DualCompressedViewer._on_command(viewer, msg)
    odom = SimpleNamespace(twist=SimpleNamespace(twist=SimpleNamespace(
        linear=SimpleNamespace(x=0.3), angular=SimpleNamespace(z=-0.7))),
        pose=SimpleNamespace(pose=SimpleNamespace(
            position=SimpleNamespace(x=1.5, y=-2.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))))
    DualCompressedViewer._on_odom(viewer, odom)
    DualCompressedViewer._on_voltage(viewer, 'left', SimpleNamespace(data=18.6))
    DualCompressedViewer._on_voltage(viewer, 'right', SimpleNamespace(data=float('nan')))
    DualCompressedViewer._on_health(viewer, SimpleNamespace(data='RECOVERING'))
    data = state.snapshot()
    assert data['command'] == (-0.4, 0.2)
    assert data['actual'] == (0.3, -0.7)
    assert viewer._latest_odom_pose == pytest.approx((1.5, -2.0, 0.0))
    assert data['voltage_left'] == 18.6
    assert data['voltage_right'] is None
    assert data['health'] == 'RECOVERING'


@pytest.mark.parametrize('status', ['NORMAL', 'RECOVERING', 'FAULT', None])
def test_dashboard_renders_status_and_controls(status):
    image, hits = render_dashboard(320, 240, {
        'source': 'map', 'health': status, 'source_menu': True})
    assert image.shape == (240, 320, 3)
    assert np.any(image)
    assert set(hits) == {'source', 'reset', 'clear', 'odom', 'map'}


@pytest.mark.parametrize(('width', 'height'), [
    (160, 120), (240, 180), (320, 240), (480, 360), (800, 600)])
def test_reset_button_does_not_cover_wheel_speed_row(width, height, monkeypatch):
    original = cv2.putText
    drawn = {}

    def record(image, text, position, font, scale, color, thickness, line_type):
        size, baseline = cv2.getTextSize(text, font, scale, thickness)
        drawn[text] = (position, size, baseline)
        return original(image, text, position, font, scale, color, thickness, line_type)

    monkeypatch.setattr(cv2, 'putText', record)
    _, hits = render_dashboard(width, height, {
        'wheels': {'left': 23.4, 'right': 22.7}})
    (x, y), (w, h), baseline = drawn['WHEEL RPM  L +23.4  R +22.7']
    assert 0 <= x and x + w < width
    assert 0 <= y - h and y + baseline < hits['reset'][1]


@pytest.mark.parametrize(('width', 'height'), [
    (160, 120), (240, 180), (320, 240), (480, 360), (800, 600)])
@pytest.mark.parametrize('source', ['odom', 'map'])
def test_reset_button_is_bottom_aligned_and_separate_from_selector(
        width, height, source, monkeypatch):
    original = cv2.putText
    drawn = {}

    def record(image, text, position, font, scale, color, thickness, line_type):
        if text in ('[ODOM v]', '[MAP v]', 'RESET ORIGIN'):
            size, _ = cv2.getTextSize(text, font, scale, thickness)
            drawn[text] = (position, size)
        return original(image, text, position, font, scale, color, thickness, line_type)

    monkeypatch.setattr(cv2, 'putText', record)
    _, hits = render_dashboard(width, height, {'source': source})
    (source_x, source_y), (source_w, source_h) = drawn[f'[{source.upper()} v]']
    (button_x, button_y), (button_w, button_h) = drawn['RESET ORIGIN']
    assert source_x + source_w <= hits['source'][2]
    assert hits['reset'][0] <= button_x
    assert button_x + button_w < hits['reset'][2]
    assert hits['reset'][1] <= button_y - button_h
    assert button_y < hits['reset'][3]
    assert hits['reset'][1] > hits['source'][3]


@pytest.mark.parametrize(('width', 'height'), [
    (160, 120), (240, 180), (320, 240), (480, 360), (800, 600)])
def test_clear_button_fits_and_does_not_cover_other_rows(width, height, monkeypatch):
    original = cv2.putText
    drawn = {}

    def record(image, text, position, font, scale, color, thickness, line_type):
        size, _ = cv2.getTextSize(text, font, scale, thickness)
        drawn[text] = (position, size)
        return original(image, text, position, font, scale, color, thickness, line_type)

    monkeypatch.setattr(cv2, 'putText', record)
    _, hits = render_dashboard(width, height, {
        'health': 'NORMAL', 'notice': 'Path cleared'})
    reset, clear = hits['reset'], hits['clear']
    assert 0 <= reset[0] < reset[2] < clear[0] < clear[2] <= width
    assert reset[1] == clear[1] and reset[3] == clear[3]
    (x, y), (w, h) = drawn['CLEAR PATH']
    assert clear[0] <= x and x + w < clear[2]
    assert clear[1] <= y - h and y < clear[3]
    for text in ('WATCHDOG  Motors OK', 'Path cleared'):
        if text not in drawn:
            continue
        (_, baseline), (_, text_height) = drawn[text]
        assert baseline - text_height < clear[1]
    assert 'Path cleared' in drawn
    if 'WATCHDOG  Motors OK' in drawn:
        watch_y = drawn['WATCHDOG  Motors OK'][0][1]
        assert drawn['Path cleared'][0][1] - drawn['Path cleared'][1][1] > watch_y


@pytest.mark.parametrize(('width', 'height'), [
    (160, 120), (240, 180), (320, 240), (480, 360), (800, 600)])
@pytest.mark.parametrize('units', ['m', 'ft'])
@pytest.mark.parametrize('source_menu', [False, True])
def test_all_telemetry_lines_fit_without_overlap(
        width, height, units, source_menu, monkeypatch):
    original = cv2.putText
    drawn = []

    def record(image, text, position, font, scale, color, thickness, line_type):
        size, baseline = cv2.getTextSize(text, font, scale, thickness)
        if text:
            drawn.append((text, position, size, baseline))
        return original(image, text, position, font, scale, color, thickness, line_type)

    monkeypatch.setattr(cv2, 'putText', record)
    _, hits = render_dashboard(width, height, {
        'command': (1.0, -0.5), 'actual': (0.9, -0.4),
        'wheels': {'left': 123.4, 'right': -456.7},
        'source': 'odom', 'position': (1234.5, -9876.5, 1.2),
        'voltage_left': 24.5, 'voltage_right': 24.6,
        'health': 'NORMAL', 'notice': 'Path cleared', 'units': units,
        'source_menu': source_menu})
    telemetry = [item for item in drawn
                 if item[0] not in ('RESET ORIGIN', 'CLEAR PATH', 'Odom', 'Map')]
    assert len(telemetry) == 11
    assert any(text.startswith('WATCHDOG') for text, *_ in telemetry)
    assert any(text.startswith('WHEEL RPM') for text, *_ in telemetry)
    assert any(text.startswith('X ') for text, *_ in telemetry)
    for _, (x, y), (w, h), baseline in telemetry:
        assert 0 <= x and x + w < width
        assert 0 <= y - h and y + baseline < hits['reset'][1]
    for before, after in zip(telemetry, telemetry[1:]):
        assert before[1][1] + before[3] < after[1][1] - after[2][1]
    if source_menu:
        for text, (x, y), (w, h), baseline in drawn:
            if text in ('Odom', 'Map'):
                box = hits[text.lower()]
                assert box[0] <= x and x + w < box[2]
                assert box[1] <= y - h and y + baseline < box[3]
        assert hits['source'][3] <= hits['odom'][1]
        assert hits['odom'][3] <= telemetry[5][1][1] - telemetry[5][2][1]
    assert hits['reset'][1] == hits['clear'][1]
    assert hits['reset'][2] <= hits['clear'][0]


def test_maximize_window_selects_only_named_window(monkeypatch):
    target = Mock()
    target.windowTitle.return_value = 'OBot Cameras'
    other = Mock()
    other.windowTitle.return_value = 'Other'
    app = Mock()
    app.topLevelWidgets.return_value = [other, target]
    monkeypatch.setattr(dual_compressed_viewer.QApplication, 'instance',
                        lambda: app)
    assert dual_compressed_viewer.maximize_window('OBot Cameras')
    target.showMaximized.assert_called_once()
    other.showMaximized.assert_not_called()
    assert not dual_compressed_viewer.maximize_window('Missing')


def test_viewer_composites_camera_and_dashboard_and_checks_tf(monkeypatch):
    images = []
    maximize = Mock(return_value=True)
    monkeypatch.setattr(dual_compressed_viewer, 'maximize_window', maximize)
    monkeypatch.setattr(cv2, 'namedWindow', lambda *args: None)
    monkeypatch.setattr(cv2, 'setMouseCallback', lambda *args: None)
    monkeypatch.setattr(cv2, 'getWindowProperty', lambda *args: 1)
    monkeypatch.setattr(cv2, 'resizeWindow', lambda *args: None)
    monkeypatch.setattr(cv2, 'getWindowImageRect',
                        lambda name: (0, 0, 0, 0))
    monkeypatch.setattr(cv2, 'imshow', lambda name, image: images.append(image.copy()))
    monkeypatch.setattr(cv2, 'waitKey', lambda *args: -1)
    monkeypatch.setattr(cv2, 'destroyAllWindows', lambda: None)
    rclpy.init()
    node = DualCompressedViewer()
    try:
        image = np.full((480, 640, 3), 100, dtype=np.uint8)
        ok, encoded = cv2.imencode('.jpg', image)
        assert ok
        msg = CompressedImage()
        msg.format = 'jpeg'
        msg.data = encoded.tobytes()
        node.on_left(msg)
        node.on_right(msg)
        node.draw()
        maximize.assert_called_once_with(node.combined_window)
        assert images[-1].shape == (640, 960, 3)
        assert np.any(images[-1][240:, 640:] != 0)
        assert node.left_msg_count == node.right_msg_count == 1

        source_box = node._panel_hits['source']
        node._on_mouse(cv2.EVENT_LBUTTONUP, 640 + source_box[0] + 1,
                       240 + source_box[1] + 1, 0, None)
        node.last_draw_time = 0
        node.draw()
        maximize.assert_called_once()
        map_box = node._panel_hits['map']
        node._on_mouse(cv2.EVENT_LBUTTONUP, 640 + map_box[0] + 1,
                       240 + map_box[1] + 1, 0, None)
        assert node.telemetry.snapshot()['source'] == 'map'
        assert node.telemetry.snapshot()['position'] is None
        node.telemetry.select('odom')

        transform = TransformStamped()
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.transform.translation.x = 2.0
        transform.transform.rotation.w = 1.0
        node.tf_buffer = SimpleNamespace(lookup_transform=lambda *args: transform)
        node._refresh_poses()
        assert node.telemetry.snapshot()['position'] == (2, 0, 0)
        transform.header.stamp.sec -= 10
        node._refresh_poses()
        assert node.telemetry.snapshot()['position'] is None
        assert not node.telemetry.reset()
        node.show_side_by_side = False
        node.last_draw_time = 0
        images.clear()
        node.draw()
        assert [img.shape for img in images] == [
            (640, 640, 3), (480, 320, 3)]
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_upscaled_dashboard_and_mouse_hitboxes(monkeypatch):
    images = []
    monkeypatch.setattr(dual_compressed_viewer, 'maximize_window',
                        lambda name: True)
    monkeypatch.setattr(cv2, 'namedWindow', lambda *args: None)
    monkeypatch.setattr(cv2, 'setMouseCallback', lambda *args: None)
    monkeypatch.setattr(cv2, 'getWindowProperty', lambda *args: 1)
    monkeypatch.setattr(cv2, 'getWindowImageRect',
                        lambda name: (0, 0, 1440, 720))
    monkeypatch.setattr(cv2, 'resizeWindow', lambda *args: None)
    monkeypatch.setattr(cv2, 'imshow', lambda name, image: images.append(image.copy()))
    monkeypatch.setattr(cv2, 'waitKey', lambda *args: -1)
    monkeypatch.setattr(cv2, 'destroyAllWindows', lambda: None)
    rclpy.init()
    node = DualCompressedViewer()
    try:
        image = np.full((240, 320, 3), 100, dtype=np.uint8)
        node.last_left = node.last_right = image
        node.telemetry.put('pose_odom', (1.0, 2.0, 0.5))
        node.draw()
        assert images[-1].shape == (720, 1080, 3)
        assert node.front_display_size == (720, 540)
        assert node._panel.shape == (450, 360, 3)
        x0, y0, x1, y1 = node._panel_hits['reset']
        assert np.any(node._panel[y0:y1, x0:x1] != (35, 35, 35))
        px, py = node._panel_origin
        node._on_mouse(cv2.EVENT_LBUTTONUP, px + (x0 + x1) // 2,
                       py + (y0 + y1) // 2, 0, None)
        assert node.telemetry.snapshot()['position'] == pytest.approx((0, 0, 0))
        monkeypatch.setattr(cv2, 'getWindowImageRect',
                            lambda name: (0, 0, 720, 360))
        node.last_draw_time = 0
        node.draw()
        assert images[-1].shape == (360, 540, 3)
        assert node.front_display_size == (360, 270)
        assert node._panel.shape == (225, 180, 3)
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize(('window', 'expected_shape', 'front_size', 'panel_shape'), [
    ((1854, 1011), (1011, 1516), (1011, 758), (632, 505)),
    ((900, 1200), (600, 900), (600, 450), (375, 300)),
    ((480, 320), (320, 480), (320, 240), (200, 160)),
])
def test_composite_fits_window_with_fixed_camera_relative_geometry(
        monkeypatch, window, expected_shape, front_size, panel_shape):
    images = []
    monkeypatch.setattr(dual_compressed_viewer, 'maximize_window', lambda name: True)
    monkeypatch.setattr(cv2, 'namedWindow', lambda *args: None)
    monkeypatch.setattr(cv2, 'setMouseCallback', lambda *args: None)
    monkeypatch.setattr(cv2, 'getWindowProperty', lambda *args: 1)
    monkeypatch.setattr(cv2, 'getWindowImageRect', lambda name: (0, 0, *window))
    monkeypatch.setattr(cv2, 'resizeWindow', lambda *args: None)
    monkeypatch.setattr(cv2, 'imshow', lambda name, image: images.append(image.copy()))
    monkeypatch.setattr(cv2, 'waitKey', lambda *args: -1)
    monkeypatch.setattr(cv2, 'destroyAllWindows', lambda: None)
    rclpy.init()
    node = DualCompressedViewer()
    try:
        node.last_left = np.full((480, 640, 3), 100, dtype=np.uint8)
        node.last_right = np.full((480, 640, 3), 200, dtype=np.uint8)
        node.draw()
        assert images[-1].shape[:2] == expected_shape
        assert node.front_display_size == front_size
        assert node._panel.shape[:2] == panel_shape
        assert node._panel_origin == (front_size[0], expected_shape[0] - panel_shape[0])
        assert abs(node._panel_origin[1] - front_size[1] / 2) <= 1
        assert abs(panel_shape[1] - front_size[0] / 2) <= 1
        assert abs((expected_shape[0] - front_size[1]) / expected_shape[0] - 0.25) < 0.002
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_clear_path_service_unavailable_and_independent():
    state = Telemetry(2)
    state.put('pose_odom', (1.0, 2.0, 0.5))
    assert state.reset()
    viewer = SimpleNamespace(
        telemetry=state, _clear_lock=threading.Lock(),
        _clear_future=None, _clear_client=SimpleNamespace(
            service_is_ready=lambda: False))
    DualCompressedViewer._clear_path(viewer)
    assert state.snapshot()['notice'] == 'Path unavailable'
    assert state.snapshot()['position'] == pytest.approx((0, 0, 0))


def test_clear_path_service_async_success():
    state = Telemetry(2)
    future = Mock()
    future.result.return_value = SimpleNamespace(success=True)
    client = Mock()
    client.service_is_ready.return_value = True
    client.call_async.return_value = future
    viewer = SimpleNamespace(
        telemetry=state, _clear_lock=threading.Lock(),
        _clear_future=None, _clear_client=client,
        _clear_done=Mock())
    DualCompressedViewer._clear_path(viewer)
    assert viewer._clear_future is future
    future.add_done_callback.assert_called_once_with(viewer._clear_done)
    DualCompressedViewer._clear_done(viewer, future)
    assert state.snapshot()['notice'] == 'Path cleared'


def test_clear_path_failure_and_timeout(monkeypatch):
    state = Telemetry(2)
    future = Mock()
    viewer = SimpleNamespace(
        telemetry=state, _clear_lock=threading.Lock(),
        _clear_future=future, _clear_started=1.0, _clear_client=Mock(),
        get_logger=Mock(return_value=Mock()))
    future.result.return_value = SimpleNamespace(success=False)
    DualCompressedViewer._clear_done(viewer, future)
    assert state.snapshot()['notice'] == 'Clear failed'
    viewer._clear_future = future
    future.result.side_effect = RuntimeError('service failure')
    DualCompressedViewer._clear_done(viewer, future)
    assert state.snapshot()['notice'] == 'Clear failed'
    viewer.get_logger().warning.assert_called_once()
    viewer._clear_future = future
    monkeypatch.setattr(dual_compressed_viewer.time, 'monotonic', lambda: 7.0)
    DualCompressedViewer._check_clear_timeout(viewer)
    viewer._clear_client.remove_pending_request.assert_called_once_with(future)
    assert viewer._clear_future is None
    assert state.snapshot()['notice'] == 'Clear timeout'


def test_clear_button_click_does_not_reset_origin():
    state = Telemetry(2)
    state.put('pose_odom', (1.0, 2.0, 0.5))
    viewer = SimpleNamespace(
        telemetry=state, _panel_origin=(10, 20),
        _panel_hits={'clear': (100, 50, 150, 70)},
        _clear_path=Mock(), _panel_time=1.0,
        front_display_size=None, _handle_motion_mouse=Mock(return_value=False),
        _track_click_press=Mock(),
        _handle_spray_mouse=Mock(return_value=False))
    viewer._on_dashboard_mouse = lambda x, y: DualCompressedViewer._on_dashboard_mouse(
        viewer, x, y)
    DualCompressedViewer._on_mouse(
        viewer, cv2.EVENT_LBUTTONUP, 120, 80, 0, None)
    viewer._clear_path.assert_called_once()
    assert state.snapshot()['position'] == pytest.approx((1.0, 2.0, 0.5))
