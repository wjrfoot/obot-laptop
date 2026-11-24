import math
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from rclpy.time import Time
from tf2_ros import TransformException

from obot.camera_lidar import (
    Hazard, LidarConfig, LidarReading, LidarState,
    center_distance, corridor_hazards, process_scan, scan_points,
)
from obot.dual_compressed_viewer import (
    DualCompressedViewer, LIDAR_DEFAULTS, OVERLAY_DEFAULTS, OverlayConfig,
    draw_lidar, draw_turn_indicator, lidar_labels,
)


def scan(ranges, angle_min=0.0, angle_increment=math.radians(1)):
    return SimpleNamespace(
        ranges=ranges, angle_min=angle_min, angle_increment=angle_increment,
        range_min=0.1, range_max=12.0)


def transform(x=0.0, y=0.0, yaw=0.0):
    return SimpleNamespace(
        translation=SimpleNamespace(x=x, y=y, z=0.0),
        rotation=SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2),
                                 w=math.cos(yaw / 2)))


def displayed(reading, config=None):
    state = LidarState(config or LidarConfig())
    state.put(reading, state.clock())
    return state.snapshot()


def test_centerline_and_bumper_reference():
    cfg = LidarConfig(front_bumper_x_m=0.4)
    reading = process_scan(scan([2.0]), transform(x=0.26), cfg)
    assert reading.center_m == pytest.approx(1.86)
    assert reading.center_nearest_m == pytest.approx(1.86)


def test_default_bumper_distance_from_lidar_center():
    cfg = LidarConfig()
    assert cfg.front_bumper_x_m == pytest.approx(0.30572)
    assert LIDAR_DEFAULTS['lidar.front_bumper_x_m'] == cfg.front_bumper_x_m
    reading = process_scan(scan([1.0]), transform(x=0.260), cfg)
    assert reading.center_m == pytest.approx(1.0 - 0.04572)
    assert reading.center_nearest_m == pytest.approx(1.0 - 0.04572)


def test_centerline_median_rejects_noise_and_outside_window():
    cfg = LidarConfig(front_bumper_x_m=0.0)
    points = [(3.0, 0), (3.1, 0.02), (9.0, 0), (0.5, 0.2)]
    assert center_distance(points, cfg) == pytest.approx(3.1)


@pytest.mark.parametrize(('y', 'region'), [
    (0.3429, 'LEFT'), (-0.3429, 'RIGHT'),
])
def test_corridor_boundaries(y, region):
    cfg = LidarConfig(front_bumper_x_m=0.0)
    hazards = corridor_hazards([(1.0, y)], cfg)
    assert [(h.region, h.severity) for h in hazards] == [(region, 'yellow')]
    assert hazards[0].distance_m == pytest.approx(1.0)


def test_outside_corridor_and_beyond_warning_distance():
    cfg = LidarConfig(front_bumper_x_m=0.0)
    assert corridor_hazards([(1.0, 0.343), (1.525, 0.0)], cfg) == ()


def test_narrow_obstacle_detected_without_centerline_measurement():
    cfg = LidarConfig(front_bumper_x_m=0.0)
    points = [(0.8, 0.25)]
    assert center_distance(points, cfg) is None
    assert corridor_hazards(points, cfg) == (Hazard('LEFT', 0.8, 'yellow'),)


def test_left_right_are_robot_coordinates_after_positive_90_degree_transform():
    ranges = [float('inf')] * 91
    ranges[0] = 2.0
    ranges[80] = 1.0
    msg = scan(ranges, angle_min=-math.pi / 2 - math.radians(8),
               angle_increment=math.radians(0.2))
    result = process_scan(msg, transform(x=0.260, yaw=math.pi / 2), LidarConfig())
    assert result.right_m == pytest.approx(2 * math.cos(math.radians(8)) - 0.04572)
    assert result.left_m == pytest.approx(math.cos(math.radians(8)) - 0.04572)
    assert result.left_m < result.right_m


def test_nearest_per_region_and_threshold_boundaries():
    cfg = LidarConfig(front_bumper_x_m=0.0)
    hazards = corridor_hazards([
        (1.524, 0.25), (0.6096, 0.0), (0.60959, 0.0),
        (0.8, -0.25), (0.7, -0.25),
    ], cfg)
    assert hazards == (
        Hazard('LEFT', 1.524, 'yellow'),
        Hazard('CENTER', 0.60959, 'red'),
        Hazard('RIGHT', 0.7, 'yellow'),
    )
    assert corridor_hazards([(0.6096, 0)], cfg)[0].severity == 'yellow'


@pytest.mark.parametrize('ranges', [[], [float('nan'), float('inf'), -1, 0.05, 13]])
def test_missing_invalid_ranges_are_not_a_clear_measurement(ranges):
    result = process_scan(scan(ranges), transform(), LidarConfig())
    assert result.center_m is None
    assert result.center_nearest_m is None
    assert result.coverage_uncertain
    labels = [text for text, _ in lidar_labels(displayed(result))]
    assert labels == ['FRONT: --', 'LEFT: --', 'RIGHT: --', 'LIDAR COVERAGE UNKNOWN']


def test_zero_range_is_invalid_even_when_declared_minimum_is_zero():
    msg = scan([0.0])
    msg.range_min = 0.0
    result = process_scan(msg, transform(), LidarConfig())
    assert result.center_m is None and result.coverage_uncertain


def test_positive_90_degree_mounting_uses_tf_not_raw_zero():
    points, coverage_uncertain = scan_points(
        scan([2.0], angle_min=-math.pi / 2), transform(x=0.26, yaw=math.pi / 2))
    assert coverage_uncertain
    assert points[0] == pytest.approx((2.26, 0.0))
    assert center_distance(points, LidarConfig(front_bumper_x_m=0.26)) == pytest.approx(2)


def full_scan(yaw, blocked):
    increment = math.radians(0.2)
    values = []
    for i in range(1800):
        raw_angle = -math.pi + i * increment
        bearing = math.atan2(math.sin(raw_angle + yaw), math.cos(raw_angle + yaw))
        values.append(blocked(i, bearing))
    return scan(values, angle_min=-math.pi, angle_increment=increment)


@pytest.mark.parametrize('yaw', [0.0, math.pi / 2, -math.pi / 2])
def test_normal_scan_tolerates_scattered_invalid_returns(yaw):
    msg = full_scan(yaw, lambda i, _: (float('inf') if i % 7 == 0 else
                                      float('nan') if i % 11 == 0 else 3.0))
    reading = process_scan(msg, transform(yaw=yaw), LidarConfig())
    assert not reading.coverage_uncertain
    assert reading.center_m is not None
    assert reading.center_nearest_m > 1.524


@pytest.mark.parametrize('yaw', [0.0, math.pi / 2, -math.pi / 2])
def test_forward_sector_without_valid_returns_is_uncertain(yaw):
    msg = full_scan(yaw, lambda _, bearing:
                    float('inf') if math.radians(20) <= bearing < math.radians(40)
                    else 3.0)
    reading = process_scan(msg, transform(yaw=yaw), LidarConfig())
    assert reading.coverage_uncertain
    assert lidar_labels(displayed(reading))[-1][0] == 'LIDAR COVERAGE UNKNOWN'


def test_rear_tank_occlusion_does_not_make_forward_coverage_uncertain():
    yaw = math.pi / 2
    msg = full_scan(yaw, lambda _, bearing:
                    float('nan') if abs(bearing) > math.pi / 2 else 3.0)
    reading = process_scan(msg, transform(yaw=yaw), LidarConfig())
    assert not reading.coverage_uncertain
    assert reading.center_m is not None


def test_truncated_scan_reports_coverage_unknown_even_when_all_returns_valid():
    msg = scan([3.0] * 100, angle_min=0.0, angle_increment=math.radians(0.2))
    reading = process_scan(msg, transform(), LidarConfig())
    assert reading.coverage_uncertain


def test_unknown_coverage_does_not_hide_detected_hazard():
    msg = scan([0.5], angle_min=0.0)
    reading = process_scan(msg, transform(x=0.260), LidarConfig())
    assert reading.coverage_uncertain
    labels = lidar_labels(displayed(reading))
    assert labels[0] == ('FRONT: 1.5 ft', (0, 0, 255))
    labels = [text for text, _ in labels]
    assert labels[-1] == 'LIDAR COVERAGE UNKNOWN'


def test_returns_behind_bumper_are_excluded_but_close_ahead_is_kept():
    cfg = LidarConfig(front_bumper_x_m=0.4)
    assert center_distance([(0.3, 0.0)], cfg) is None
    assert corridor_hazards([(0.3, 0.0)], cfg) == ()
    assert corridor_hazards([(0.401, 0.0)], cfg)[0].distance_m == pytest.approx(0.001)
    assert corridor_hazards([(0.401, 0.0)], cfg)[0].severity == 'red'
    assert corridor_hazards([(0.4, 0.0)], cfg)[0].distance_m == 0


def test_stale_and_unavailable_states_use_monotonic_time():
    now = [10.0]
    state = LidarState(LidarConfig(), clock=lambda: now[0])
    assert state.snapshot().status == 'stale'
    reading = LidarReading(2, None, None, 2, False)
    state.put(reading, 10.0)
    assert state.snapshot().reading.front.distance_m == 2
    now[0] = 10.5
    assert state.snapshot().reading.front.distance_m is None
    now[0] = 10.501
    assert state.snapshot().status == 'stale'
    state.unavailable('TF unavailable')
    assert lidar_labels(state.snapshot())[0][0] == 'LIDAR TF UNAVAILABLE'


def test_distance_smoothing_approach_is_immediate_and_receding_is_gradual():
    now = [10.0]
    state = LidarState(LidarConfig(), clock=lambda: now[0])
    state.put(LidarReading(2, None, None, None, False), now[0])
    now[0] += 0.1
    state.put(LidarReading(1, None, None, None, False), now[0])
    assert state.snapshot().reading.front.distance_m == 1
    now[0] += 0.1
    state.put(LidarReading(2, None, None, None, False), now[0])
    assert state.snapshot().reading.front.distance_m == pytest.approx(1.25)


def test_warning_hysteresis_promotes_immediately_and_relaxes_only_above_margin():
    now = [10.0]
    cfg = LidarConfig(warning_distance_m=1.5, danger_distance_m=0.6)
    state = LidarState(cfg, clock=lambda: now[0])
    def update(value):
        state.put(LidarReading(value, None, None, None, False), now[0])
        return state.snapshot().reading.front
    assert update(1.6).severity == 'green'
    now[0] += 0.1
    assert update(1.49).severity == 'yellow'
    now[0] += 0.1
    assert update(0.59).severity == 'red'
    now[0] += 0.1
    assert update(0.61).severity == 'red'
    for _ in range(20):
        now[0] += 0.1
        last = update(1.55)
    assert last.severity == 'yellow'
    for _ in range(20):
        now[0] += 0.1
        last = update(1.8)
    assert last.severity == 'green'


def test_missing_return_hold_is_marked_and_expires_before_scan_staleness():
    now = [10.0]
    state = LidarState(LidarConfig(), clock=lambda: now[0])
    state.put(LidarReading(1, None, None, 1, False), now[0])
    now[0] = 10.1
    state.put(LidarReading(None, None, None, None, False), now[0])
    assert lidar_labels(state.snapshot())[0][0] == 'FRONT: ~3.3 ft'
    now[0] = 10.251
    assert lidar_labels(state.snapshot())[0][0] == 'FRONT: --'
    assert state.snapshot().status == 'ok'
    now[0] = 10.601
    assert lidar_labels(state.snapshot())[0][0] == 'LIDAR STALE'


def test_stale_gap_and_out_of_order_updates_do_not_reuse_old_values():
    now = [10.0]
    state = LidarState(LidarConfig(), clock=lambda: now[0])
    state.put(LidarReading(0.2, None, None, 0.2, False), now[0])
    state.put(LidarReading(5, None, None, 5, False), 9.9)
    assert state.snapshot().reading.front.distance_m == 0.2
    now[0] = 10.6
    state.put(LidarReading(3, None, None, 3, False), now[0])
    assert state.snapshot().reading.front.distance_m == 3
    assert state.snapshot().reading.front.severity == 'green'


@pytest.mark.parametrize(('bearing', 'expected'), [(-29.2, 'TURN LEFT 29'), (29.2, 'TURN RIGHT 29')])
def test_turn_indicator_is_compact_and_retains_direction(monkeypatch, bearing, expected):
    import cv2
    image = np.full((200, 400, 3), 255, dtype=np.uint8)
    put_text = Mock()
    monkeypatch.setattr(cv2, 'putText', put_text)
    draw_turn_indicator(image, bearing)
    assert put_text.call_args.args[1] == expected
    assert np.all(image[35:, :, :] == 255)
    assert np.all(image[:, 150:, :] == 255)


def test_center_obstacle_closer_than_median_remains_visible():
    reading = LidarReading(1.0, 0.5, None, 0.2, False)
    labels = lidar_labels(displayed(reading))
    assert labels == [('FRONT: 3.3 ft', (0, 255, 255)),
                      ('LEFT: 1.6 ft', (0, 0, 255)),
                      ('RIGHT: --', (180, 180, 180)),
                      ('NEAR: 0.7 ft', (0, 0, 255))]


def test_draw_labels_preserves_middle_and_adds_no_hitboxes():
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    reading = LidarReading(1.0, 0.5, None, 1.0, False)
    draw_lidar(image, displayed(reading))
    assert image[:100].any()
    assert not image[150:].any()


def test_scan_callback_looks_up_transform_at_scan_timestamp():
    msg = scan([2.0], angle_min=-math.pi / 2)
    msg.header = SimpleNamespace(frame_id='laser', stamp=Time(seconds=100).to_msg())
    buffer = Mock()
    buffer.lookup_transform.return_value.transform = transform(x=0.26, yaw=math.pi / 2)
    state = LidarState(LidarConfig(front_bumper_x_m=0.26))
    node = SimpleNamespace(
        lidar_config=LidarConfig(front_bumper_x_m=0.26), lidar=state,
        tf_buffer=buffer, _lidar_last_error=None,
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=100.1)),
        get_logger=Mock())
    DualCompressedViewer._on_scan(node, msg)
    args = buffer.lookup_transform.call_args.args
    assert args[:2] == ('base_link', 'laser')
    assert args[2].nanoseconds == Time(seconds=100).nanoseconds
    assert state.snapshot().reading.front.distance_m == pytest.approx(2.0)


def test_delayed_scan_does_not_refresh_stale_reading():
    msg = scan([2.0])
    msg.header = SimpleNamespace(frame_id='laser', stamp=Time(seconds=100).to_msg())
    state = LidarState(LidarConfig())
    node = SimpleNamespace(
        lidar_config=LidarConfig(), lidar=state, tf_buffer=Mock(),
        _lidar_last_error=None,
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=101)),
        get_logger=Mock())
    DualCompressedViewer._on_scan(node, msg)
    assert state.snapshot().status == 'stale'
    node.tf_buffer.lookup_transform.assert_not_called()
    node.get_logger().warning.assert_called_once()


def test_missing_transform_invalidates_previous_reading():
    msg = scan([2.0])
    msg.header = SimpleNamespace(frame_id='laser', stamp=Time(seconds=100).to_msg())
    state = LidarState(LidarConfig())
    state.put(LidarReading(2, None, None, 2, False), state.clock())
    buffer = Mock()
    buffer.lookup_transform.side_effect = TransformException('no TF')
    node = SimpleNamespace(
        lidar_config=LidarConfig(), lidar=state, tf_buffer=buffer,
        _lidar_last_error=None,
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=100.1)),
        get_logger=Mock())
    DualCompressedViewer._on_scan(node, msg)
    assert state.snapshot().status == 'TF unavailable'
    assert state.snapshot().reading is None
    node.get_logger().warning.assert_called_once()


@pytest.mark.parametrize('combined', [False, True])
def test_draw_in_both_window_modes_preserves_markers_and_mouse_setup(monkeypatch, combined):
    import cv2

    shown = {}
    monkeypatch.setattr(cv2, 'imshow', lambda name, image: shown.setdefault(name, image.copy()))
    monkeypatch.setattr(cv2, 'setMouseCallback', Mock())
    monkeypatch.setattr(cv2, 'getWindowProperty', lambda *args: 1)
    monkeypatch.setattr(cv2, 'waitKey', lambda *args: 255)
    state = LidarState(LidarConfig())
    state.put(LidarReading(1.0, 0.5, None, 1.0, False),
              state.clock())
    viewer = SimpleNamespace(
        max_display_hz=0.0, last_draw_time=0.0, _image_lock=threading.Lock(),
        last_left=np.zeros((480, 640, 3), dtype=np.uint8),
        last_right=np.zeros((480, 640, 3), dtype=np.uint8),
        overlay=OverlayConfig(OVERLAY_DEFAULTS), lidar=state,
        selected_target=None, show_side_by_side=combined, telemetry=None,
        window_left='front', window_right='rear', combined_window='combined',
        _on_front_mouse=Mock(), _window_sized=True, _window_maximize_attempted=True,
        _click_poll=Mock(), _click_drive_buttons=Mock(),
        spray=Mock(), _spray_panel=None, _spray_hits={},
        _manual_tick=Mock(), waitkey_ms=1,
        click_distance_edit=False)
    viewer.spray.snapshot.return_value = ('CLOSED', False, None, '')
    viewer._draw_spray = DualCompressedViewer._draw_spray.__get__(viewer)
    DualCompressedViewer.draw(viewer)
    image = shown['combined' if combined else 'front']
    assert image[:100, :640].any()
    assert tuple(image[240, 320]) == (0, 255, 0)
    assert viewer.front_display_size == (640, 480)
    viewer._click_drive_buttons.assert_called_once()
    assert set(viewer._spray_hits) == {'square', 'circle', 'triangle', 'cancel'}


def test_invalid_config_and_transform_raise_explicit_errors():
    with pytest.raises(ValueError):
        LidarConfig(warning_distance_m=0.1)
    bad = transform()
    bad.rotation.w = 0.0
    with pytest.raises(ValueError):
        scan_points(scan([1.0]), bad)
