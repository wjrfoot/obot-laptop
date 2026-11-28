"""OpenCV dashboard drawn only in the space beneath the rear camera."""

import math

import cv2
import numpy as np

from .camera_units import display_linear


def _number(value, digits=2):
    return '--' if value is None else f'{value:+.{digits}f}'


def render_dashboard(width, height, data):
    """Return a panel and its image-coordinate hitboxes."""
    panel = np.full((height, width, 3), (35, 35, 35), dtype=np.uint8)
    if width < 200 or height < 160:
        return panel, {}
    scale = min(1.5, max(0.55, 0.55 * min(width / 220, height / 180)))
    row = max(18, min(62, height // 9))
    margin = max(3, width // 50)
    thickness = max(1, round(scale * 1.5))
    color = (225, 225, 225)
    muted = (160, 160, 160)
    lower_offset = 0

    def text_at(text, y, ink=color):
        cv2.putText(panel, text, (margin, min(height - 2, y)),
                    cv2.FONT_HERSHEY_SIMPLEX, scale, ink, thickness, cv2.LINE_AA)

    def line(index, text, ink=color):
        y = (index + 1) * row + (lower_offset if index >= 5 else 0)
        text_at(text, y, ink)

    cmd = data.get('command')
    actual = data.get('actual')
    units = data.get('units', 'm')
    line(0, 'MOTION     CMD   ACTUAL')
    line(1, f'{units}/s  {_number(display_linear(cmd[0] if cmd else None, units)):>6}'
            f' {_number(display_linear(actual[0] if actual else None, units)):>6}')
    line(2, f'rad/s {_number(cmd[1] if cmd else None):>6}'
            f' {_number(actual[1] if actual else None):>6}')
    wheels = data.get('wheels') or {}
    line(3, f"WHEEL RPM  L {_number(wheels.get('left'), 1)}"
            f"  R {_number(wheels.get('right'), 1)}")
    source = data.get('source', 'odom')
    selector_text = f'[{source.upper()} v]'
    selector_width = max(
        cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)[0][0]
        for text in ('[ODOM v]', '[MAP v]'))
    selector_right = margin + selector_width
    line(4, selector_text)
    button_x = selector_right + 10
    padding = max(2, thickness + 1)
    button_gap = max(4, thickness * 2)
    button_right = button_x + (width - button_x - button_gap) // 2
    clear_x = button_right + button_gap
    button_text = 'RESET ORIGIN'
    button_scale = scale
    while True:
        text_size, baseline = cv2.getTextSize(
            button_text, cv2.FONT_HERSHEY_SIMPLEX, button_scale, thickness)
        if text_size[0] <= button_right - button_x - 2 * padding:
            break
        button_scale *= 0.9
    button_height = text_size[1] + baseline + 2 * padding
    button_bottom = height - max(10, thickness + 3)
    button_y = button_bottom - button_height
    text_y = button_bottom - baseline - padding
    cv2.rectangle(panel, (button_x, button_y),
                  (button_right - 1, button_bottom), (75, 75, 75), -1)
    cv2.rectangle(panel, (button_x, button_y),
                  (button_right - 1, button_bottom), (190, 190, 190),
                  max(1, thickness))
    text_x = button_x + max(1, (button_right - button_x - text_size[0]) // 2)
    cv2.putText(panel, button_text, (text_x, text_y),
                cv2.FONT_HERSHEY_SIMPLEX, button_scale, color, thickness,
                cv2.LINE_AA)
    lower_offset = 0
    pose = data.get('position')
    if pose is None:
        line(5, f'{source.upper()} UNAVAILABLE', muted)
        line(6, 'Heading --', muted)
    else:
        line(5, f'X {_number(display_linear(pose[0], units))} {units}'
                f'  Y {_number(display_linear(pose[1], units))} {units}')
        line(6, f'Heading {_number(math.degrees(pose[2]), 1)} deg')
    line(7, 'DC BUS VOLTAGE')
    line(8, f'L {_number(data.get("voltage_left"), 1)} V'
            f'  R {_number(data.get("voltage_right"), 1)} V')
    status = data.get('health')
    label, ink = {
        'NORMAL': ('Motors OK', (70, 220, 70)),
        'RECOVERING': ('Recovering', (0, 210, 255)),
        'FAULT': ('Motor Fault', (30, 30, 255)),
    }.get(status, ('No Data', muted))
    notice = data.get('notice')
    notice_height = cv2.getTextSize(
        notice or '', cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)[0][1]
    watch_y = min(button_y - 8, 8 * row + lower_offset)
    notice_y = min(button_y - 2, 9 * row + lower_offset)
    if notice and notice_y - notice_height <= watch_y:
        text_at(notice, notice_y, muted)
    else:
        text_at('WATCHDOG  ' + label, watch_y, ink)
        if notice:
            text_at(notice, notice_y, muted)
    clear_text = 'CLEAR PATH'
    clear_scale = button_scale
    clear_size, _ = cv2.getTextSize(
        clear_text, cv2.FONT_HERSHEY_SIMPLEX, clear_scale, thickness)
    cv2.rectangle(panel, (clear_x, button_y), (width - 1, button_bottom),
                  (75, 75, 75), -1)
    cv2.rectangle(panel, (clear_x, button_y), (width - 1, button_bottom),
                  (190, 190, 190), thickness)
    clear_text_x = clear_x + max(1, (width - clear_x - clear_size[0]) // 2)
    clear_text_y = text_y
    cv2.putText(panel, clear_text, (clear_text_x, clear_text_y),
                cv2.FONT_HERSHEY_SIMPLEX, clear_scale, color,
                thickness, cv2.LINE_AA)
    hits = {
        'source': (0, 4 * row, selector_right, 5 * row + 1),
        'reset': (button_x, button_y, button_right, button_bottom + 1),
        'clear': (clear_x, button_y, width, button_bottom + 1),
    }
    if data.get('source_menu'):
        x1 = selector_right
        cv2.rectangle(panel, (0, 5 * row), (x1, 7 * row),
                      (65, 65, 65), -1)
        cv2.putText(panel, 'Odom', (margin, 6 * row - 2),
                    cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)
        cv2.putText(panel, 'Map', (margin, 7 * row - 2),
                    cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)
        hits['odom'] = (0, 5 * row, x1, 6 * row)
        hits['map'] = (0, 6 * row, x1, 7 * row)
    return panel, hits
