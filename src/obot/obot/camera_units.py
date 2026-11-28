"""Convert operator-facing linear measurements at the display boundary."""


def display_linear(value, units):
    """Convert meters or meters per second to the selected display units."""
    if units not in ('m', 'ft'):
        raise ValueError('operator.units must be m or ft')
    if value is None:
        return None
    return value if units == 'm' else value / 0.3048


def meters(value, units):
    """Convert an operator distance to meters."""
    if units not in ('m', 'ft'):
        raise ValueError('operator.units must be m or ft')
    return value if units == 'm' else value * 0.3048
