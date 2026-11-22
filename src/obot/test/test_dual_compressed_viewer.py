import numpy as np
import pytest

from obot.dual_compressed_viewer import (
    compose,
    draw_markers,
    OVERLAY_DEFAULTS,
    OverlayConfig,
    scale_rear,
)

GREEN = (0, 255, 0)
RED_BGR = (255, 0, 0)  # RGB [0, 0, 255] stored as BGR


def config(**overrides):
    params = dict(OVERLAY_DEFAULTS)
    params.update(overrides)
    warnings = []
    return OverlayConfig(params, warn=warnings.append), warnings


def marker_columns(img, color):
    mask = np.all(img == color, axis=2)
    return sorted(set(np.nonzero(mask)[1]))


def blank(w=400, h=200):
    return np.zeros((h, w, 3), dtype=np.uint8)


def test_defaults_draw_three_markers_at_expected_columns():
    img = blank()
    draw_markers(img, config()[0])
    assert marker_columns(img, GREEN) == [199, 200]
    assert marker_columns(img, RED_BGR) == [99, 100, 299, 300]


def test_markers_are_short_and_centered_vertically():
    img = blank()
    draw_markers(img, config(center_thickness=1)[0])
    rows = sorted(set(np.nonzero(np.all(img == GREEN, axis=2))[0]))
    assert rows[0] == 100 - 20 and rows[-1] == 100 + 20


def test_center_offset_moves_all_three_markers_together():
    base, shifted = blank(), blank()
    draw_markers(base, config(center_thickness=1,
                              clearance_thickness=1)[0])
    draw_markers(shifted, config(center_offset_fraction=0.1,
                                 center_thickness=1,
                                 clearance_thickness=1)[0])
    assert marker_columns(base, GREEN) == [200]
    assert marker_columns(shifted, GREEN) == [240]
    assert marker_columns(base, RED_BGR) == [100, 300]
    assert marker_columns(shifted, RED_BGR) == [140, 340]


def test_clearance_offset_moves_outside_markers_symmetrically():
    img = blank()
    draw_markers(img, config(clearance_offset_fraction=0.1,
                             clearance_thickness=1)[0])
    assert marker_columns(img, RED_BGR) == [160, 240]


def test_zero_clearance_offset_hides_both_but_keeps_center():
    img = blank()
    draw_markers(img, config(clearance_offset_fraction=0.0,
                             clearance_enabled=True)[0])
    assert marker_columns(img, RED_BGR) == []
    assert marker_columns(img, GREEN)


def test_disabled_markers_are_not_drawn():
    img = blank()
    draw_markers(img, config(center_marker_enabled=False,
                             clearance_enabled=False)[0])
    assert not img.any()


def test_color_thickness_and_height_are_configurable():
    img = blank()
    draw_markers(img, config(
        center_color=[255, 0, 0], center_thickness=5,
        center_height_fraction=0.5, clearance_enabled=False)[0])
    mask = np.all(img == (0, 0, 255), axis=2)
    cols = np.nonzero(mask.any(axis=0))[0]
    rows = np.nonzero(mask.any(axis=1))[0]
    assert len(cols) == 5
    assert rows[-1] - rows[0] + 1 == 101


def test_markers_outside_image_are_skipped_without_error():
    img = blank()
    draw_markers(img, config(center_offset_fraction=0.4,
                             clearance_offset_fraction=0.5)[0])
    assert marker_columns(img, GREEN)
    assert marker_columns(img, RED_BGR) == [159, 160]


def test_marker_entirely_outside_is_skipped():
    img = blank()
    draw_markers(img, config(center_offset_fraction=0.55,
                             clearance_offset_fraction=0.1)[0])
    assert marker_columns(img, GREEN) == []
    assert marker_columns(img, RED_BGR) == [379, 380]


def test_height_larger_than_image_is_clipped():
    img = blank()
    draw_markers(img, config(center_height_fraction=1.0)[0])
    rows = np.nonzero(np.all(img == GREEN, axis=2).any(axis=1))[0]
    assert rows[0] == 0 and rows[-1] == 199


def test_positions_scale_with_image_size():
    small, large = blank(400, 200), blank(800, 400)
    cfg = config(center_offset_fraction=0.1, center_thickness=1)[0]
    draw_markers(small, cfg)
    draw_markers(large, cfg)
    assert marker_columns(large, GREEN)[0] == 2 * marker_columns(
        small, GREEN)[0]


@pytest.mark.parametrize('name, value', [
    ('rear_scale', 0.0),
    ('rear_scale', 2.0),
    ('center_offset_fraction', float('nan')),
    ('clearance_offset_fraction', -0.1),
    ('center_height_fraction', 1.5),
    ('center_thickness', 0),
    ('clearance_color', [0, 256, 0]),
    ('center_color', [0, 255]),
])
def test_invalid_values_fall_back_to_defaults_with_warning(name, value):
    cfg, warnings = config(**{name: value})
    assert warnings
    assert name in warnings[0]


def test_invalid_value_uses_default_value():
    cfg, _ = config(clearance_offset_fraction=-1.0, rear_scale=5.0)
    assert cfg.clearance_offset == 0.25
    assert cfg.rear_scale == 0.5


def test_rear_is_half_front_size_with_same_aspect():
    rear = np.zeros((480, 640, 3), dtype=np.uint8)
    scaled, box_w, box_h = scale_rear(rear, 800, 600, 0.5)
    assert (box_w, box_h) == (400, 300)
    assert scaled.shape[:2] == (300, 400)


def test_rear_with_different_aspect_is_not_distorted():
    rear = np.zeros((300, 600, 3), dtype=np.uint8)  # 2:1
    scaled, _, _ = scale_rear(rear, 640, 480, 0.5)
    assert scaled.shape[1] / scaled.shape[0] == pytest.approx(2.0)
    assert scaled.shape[1] <= 320 and scaled.shape[0] <= 240


def test_compose_keeps_front_full_size_and_places_rear_beside_it():
    front = np.full((480, 640, 3), 50, dtype=np.uint8)
    rear = np.full((480, 640, 3), 200, dtype=np.uint8)
    out = compose(front, rear, 0.5)
    assert out.shape == (480, 640 + 320, 3)
    assert (out[:, :640] == 50).all()
    assert (out[:240, 640:] == 200).all()
    assert (out[240:, 640:] == 0).all()
