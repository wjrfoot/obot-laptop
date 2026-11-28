"""OpenCV dashboard drawn only in the space beneath the rear camera."""

import math

import cv2
import numpy as np

from .camera_controls import push_button
from .camera_units import display_linear


def _number(value, digits=2):
    return '--' if value is None else f'{value:+.{digits}f}'


def render_dashboard(width, height, data):
    """Return a panel and its image-coordinate hitboxes."""
    panel = np.full((height, width, 3), (35, 35, 35), dtype=np.uint8)
    if width < 80 or height < 90:
        return panel, {}
    margin = max(3, width // 80)
    gap = max(3, margin)
    button_height = max(18, min(42, height // 8))
    button_top = height - margin - button_height
    available_height = button_top - margin - gap
    row = min(24, available_height / (12 if data.get('source_menu') else 11))
    cmd = data.get('command')
    actual = data.get('actual')
    units = data.get('units', 'm')
    wheels = data.get('wheels') or {}
    source = data.get('source', 'odom')
    pose = data.get('position')
    status = data.get('health')
    muted = (160, 160, 160)
    label, ink = {
        'NORMAL': ('Motors OK', (70, 220, 70)),
        'RECOVERING': ('Recovering', (0, 210, 255)),
        'FAULT': ('Motor Fault', (30, 30, 255)),
    }.get(status, ('No Data', muted))
    lines = [
        ('MOTION     CMD   ACTUAL', None),
        (f'{units}/s  {_number(display_linear(cmd[0] if cmd else None, units)):>6}'
         f' {_number(display_linear(actual[0] if actual else None, units)):>6}', None),
        (f'rad/s {_number(cmd[1] if cmd else None):>6}'
         f' {_number(actual[1] if actual else None):>6}', None),
        (f"WHEEL RPM  L {_number(wheels.get('left'), 1)}"
         f"  R {_number(wheels.get('right'), 1)}", None),
        (f'[{source.upper()} v]', None),
        ((f'X {_number(display_linear(pose[0], units))} {units}'
          f'  Y {_number(display_linear(pose[1], units))} {units}')
         if pose is not None else f'{source.upper()} UNAVAILABLE', None),
        (f'Heading {_number(math.degrees(pose[2]), 1)} deg'
         if pose is not None else 'Heading --', None),
        ('DC BUS VOLTAGE', None),
        (f'L {_number(data.get("voltage_left"), 1)} V'
         f'  R {_number(data.get("voltage_right"), 1)} V', None),
        ('WATCHDOG  ' + label, ink),
        (data.get('notice') or '', muted),
    ]
    if data.get('source_menu'):
        lines.insert(5, ('', None))
    scale = min(0.55, row / 25.0)
    thickness = 1
    while scale > 0.05:
        sizes = [cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)
                 for text, _ in lines]
        if all(size[0] <= width - 2 * margin and size[1] + baseline <= row - 2
               for size, baseline in sizes):
            break
        scale *= 0.9
    for index, ((text, color), (size, baseline)) in enumerate(zip(lines, sizes)):
        y = round(margin + index * row + (row + size[1] - baseline) / 2)
        cv2.putText(panel, text, (margin, y), cv2.FONT_HERSHEY_SIMPLEX,
                    scale, color or (225, 225, 225), thickness, cv2.LINE_AA)

    middle = width // 2
    reset = (margin, button_top, middle - gap // 2, height - margin)
    clear = (middle + gap // 2, button_top, width - margin, height - margin)
    push_button(panel, reset, 'RESET ORIGIN')
    push_button(panel, clear, 'CLEAR PATH')
    selector_width = max(
        cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)[0][0]
        for text in ('[ODOM v]', '[MAP v]'))
    selector_right = min(width, margin * 2 + selector_width)
    hits = {
        'source': (0, round(margin + 4 * row), selector_right,
                   round(margin + 5 * row)),
        'reset': reset,
        'clear': clear,
    }
    if data.get('source_menu'):
        menu_top = round(margin + 5 * row)
        menu_bottom = round(margin + 6 * row)
        for index, (name, label) in enumerate((('odom', 'Odom'), ('map', 'Map'))):
            x0 = margin + index * (width - 2 * margin) // 2
            x1 = margin + (index + 1) * (width - 2 * margin) // 2 - gap
            cv2.rectangle(panel, (x0, menu_top), (x1 - 1, menu_bottom - 1),
                          (65, 65, 65), -1)
            size, baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)
            y = round(menu_top + (menu_bottom - menu_top + size[1] - baseline) / 2)
            cv2.putText(panel, label, (x0 + margin, y), cv2.FONT_HERSHEY_SIMPLEX,
                        scale, (225, 225, 225), thickness, cv2.LINE_AA)
            hits[name] = (x0, menu_top, x1, menu_bottom)
    return panel, hits
