"""Shared presentation helpers for the viewer's bottom controls."""

import cv2


def fit_text(canvas, text, box, color=(235, 235, 235), scale=0.65, thickness=1):
    """Draw a single line centered inside a bounding box."""
    x0, y0, x1, y1 = box
    available_w = max(1, x1 - x0 - 8)
    available_h = max(1, y1 - y0 - 4)
    while scale > 0.1:
        (width, height), baseline = cv2.getTextSize(
            text, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)
        if width <= available_w and height + baseline <= available_h:
            break
        scale *= 0.9
    x = x0 + max(0, (x1 - x0 - width) // 2)
    y = y0 + max(height, (y1 - y0 + height - baseline) // 2)
    cv2.putText(canvas, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                scale, color, thickness, cv2.LINE_AA)


def push_button(canvas, box, label, *, enabled=True, primary=False, active=False, pressed=False):
    """Draw a consistently styled button with a label that fits its bounds."""
    x0, y0, x1, y1 = box
    if not enabled:
        fill, edge, ink = (80, 80, 80), (120, 120, 120), (150, 150, 150)
    elif pressed:
        fill, edge, ink = (95, 95, 130), (255, 255, 255), (255, 255, 255)
    elif active:
        fill, edge, ink = (30, 150, 30), (230, 230, 230), (255, 255, 255)
    elif primary:
        fill, edge, ink = (35, 140, 35), (235, 235, 235), (255, 255, 255)
    else:
        fill, edge, ink = (60, 60, 60), (225, 225, 225), (240, 240, 240)
    cv2.rectangle(canvas, (x0, y0), (x1 - 1, y1 - 1), fill, -1)
    cv2.rectangle(canvas, (x0, y0), (x1 - 1, y1 - 1), edge, 2)
    fit_text(canvas, label, box, ink, scale=0.65, thickness=2)
