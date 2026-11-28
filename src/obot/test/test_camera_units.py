"""Tests for operator-facing linear units."""

from obot.camera_dashboard import render_dashboard
from obot.camera_units import display_linear, meters

import pytest


@pytest.mark.parametrize(('units', 'value'), [('m', 1.0), ('ft', 1 / 0.3048)])
def test_linear_units_round_trip(units, value):
    """Convert metric and imperial values in both directions."""
    assert display_linear(1.0, units) == pytest.approx(value)
    assert meters(value, units) == pytest.approx(1.0)
    assert display_linear(None, units) is None


def test_reject_unknown_units():
    """Reject unsupported operator units."""
    with pytest.raises(ValueError):
        display_linear(1.0, 'yards')
    with pytest.raises(ValueError):
        meters(1.0, 'yards')


@pytest.mark.parametrize(('units', 'speed', 'position'), [
    ('m', 'm/s   +1.00  +0.50', 'X +1.00 m  Y +2.00 m'),
    ('ft', 'ft/s   +3.28  +1.64', 'X +3.28 ft  Y +6.56 ft'),
])
def test_dashboard_converts_linear_values(monkeypatch, units, speed, position):
    """Render speed and odometry position in the selected units."""
    import cv2

    original = cv2.putText
    drawn = []

    def capture(image, text, *args):
        drawn.append(text)
        return original(image, text, *args)

    monkeypatch.setattr(cv2, 'putText', capture)
    render_dashboard(600, 400, {
        'units': units, 'command': (1.0, 0.1), 'actual': (0.5, 0.1),
        'position': (1.0, 2.0, 0.0),
    })
    assert speed in drawn
    assert position in drawn
