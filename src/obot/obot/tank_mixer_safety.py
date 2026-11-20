"""ROS-independent safety and mixing logic for the tank teleop controller."""

import math
from dataclasses import dataclass
from enum import Enum, auto


class DeadmanState(Enum):
    """Current interlock state of the joystick deadman."""

    DISARMED = auto()
    WAIT_FOR_NEUTRAL = auto()
    ARMED = auto()


class OutputKind(Enum):
    """Action the ROS wrapper should take for the current joystick sample."""

    SILENT = auto()
    STOP = auto()
    DRIVE = auto()


@dataclass(frozen=True)
class MixerOutput:
    """Decision and velocity values returned by the tank controller."""

    kind: OutputKind
    linear_x: float = 0.0
    angular_z: float = 0.0
    warning: str = ''


class _MalformedJoy(ValueError):
    pass


def _clamp(value, lower, upper):
    """Bound a stick sample to its valid range."""
    return max(lower, min(upper, value))


def _apply_deadband(value, threshold):
    """Apply a deadband while preserving proportional response outside it."""
    magnitude = abs(value)
    if magnitude <= threshold or threshold == 1.0:
        return 0.0
    scaled_magnitude = (magnitude - threshold) / (1.0 - threshold)
    return math.copysign(scaled_magnitude, value)


class TankMixerController:
    """Deadman interlock and proportional tank mixer."""

    def __init__(
        self,
        *,
        left_axis=1,
        right_axis=4,
        left_turn_axis=-1,
        right_turn_axis=-1,
        invert_left=False,
        invert_right=False,
        deadband=0.50,
        track_width=0.40,
        max_linear=1.0,
        enable_button=-1,
        enable_axis=-1,
        enable_axis_threshold=0.5,
        require_enable=True,
        fast_mode_button=5,
        low_speed_scale=0.35,
        single_stick_low_speed_scale=0.50,
    ):
        """Create a controller with the supplied joystick mapping."""
        self.left_axis = int(left_axis)
        self.right_axis = int(right_axis)
        self.left_turn_axis = int(left_turn_axis)
        self.right_turn_axis = int(right_turn_axis)
        self.invert_left = bool(invert_left)
        self.invert_right = bool(invert_right)
        self.deadband = float(deadband)
        self.track_width = float(track_width)
        self.max_linear = float(max_linear)
        self.enable_button = int(enable_button)
        self.enable_axis = int(enable_axis)
        self.enable_axis_threshold = float(enable_axis_threshold)
        self.require_enable = bool(require_enable)
        self.fast_mode_button = int(fast_mode_button)
        self.low_speed_scale = float(low_speed_scale)
        self.single_stick_low_speed_scale = float(
            single_stick_low_speed_scale)
        self.state = DeadmanState.DISARMED
        self._owns_control = False
        self._require_release = False
        self.configuration_error = self._validate_configuration()

    def _validate_configuration(self):
        if self.left_axis < 0 or self.right_axis < 0:
            return 'Left and right tank axes must be non-negative'
        if self.left_axis == self.right_axis:
            return 'Left and right tank axes must be different'
        if self.left_turn_axis < -1 or self.right_turn_axis < -1:
            return 'Turn axes must be -1 or non-negative'
        if (self.left_turn_axis >= 0) != (self.right_turn_axis >= 0):
            return 'Configure both turn axes or neither'
        if self.left_turn_axis >= 0 and len({
                self.left_axis, self.right_axis,
                self.left_turn_axis, self.right_turn_axis}) != 4:
            return 'Tank and turn axes must all be different'
        if not math.isfinite(self.deadband) or not 0.0 <= self.deadband <= 1.0:
            return 'Deadband must be finite and between 0 and 1'
        if self.enable_button < -1 or self.enable_axis < -1:
            return 'Enable button and axis must be -1 or non-negative'
        if self.enable_button >= 0 and self.enable_axis >= 0:
            return 'Configure only one enable button or enable axis'
        if (self.require_enable and self.enable_button < 0
                and self.enable_axis < 0):
            return 'An enable button or axis is required'
        if self.fast_mode_button < -1:
            return 'Fast-mode button must be -1 or non-negative'
        if (not math.isfinite(self.enable_axis_threshold)
                or not -1.0 <= self.enable_axis_threshold <= 1.0):
            return 'Enable-axis threshold must be between -1 and 1'
        for value, name in (
                (self.track_width, 'Track width'),
                (self.max_linear, 'Maximum linear speed'),
                (self.low_speed_scale, 'Low-speed scale'),
                (self.single_stick_low_speed_scale,
                 'Single-stick scale')):
            if not math.isfinite(value):
                return f'{name} must be finite'
        return ''

    @staticmethod
    def _read_axis(axes, index, label):
        try:
            value = float(axes[index])
        except (IndexError, TypeError, ValueError, OverflowError) as exc:
            raise _MalformedJoy(
                f'Missing or invalid {label} axis {index}') from exc
        if not math.isfinite(value):
            raise _MalformedJoy(f'Non-finite {label} axis {index}')
        return value

    @staticmethod
    def _read_button(buttons, index, label):
        try:
            value = buttons[index]
        except (IndexError, TypeError) as exc:
            raise _MalformedJoy(
                f'Missing {label} button {index}') from exc
        if value not in (0, 1, False, True):
            raise _MalformedJoy(f'Invalid {label} button {index}')
        return bool(value)

    def _disarm(self, require_release):
        publish_stop = self._owns_control
        self.state = DeadmanState.DISARMED
        self._owns_control = False
        self._require_release = require_release
        return MixerOutput(
            OutputKind.STOP if publish_stop else OutputKind.SILENT)

    def invalidate(self, warning):
        """Disarm after invalid input and require a fresh enable release."""
        output = self._disarm(require_release=True)
        return MixerOutput(output.kind, warning=warning)

    def timeout(self):
        """Disarm after joystick timeout and require a fresh enable cycle."""
        return self._disarm(require_release=True)

    def process(self, axes, buttons):
        """Process one joystick sample into a stop, silence, or drive output."""
        if self.configuration_error:
            return self.invalidate(self.configuration_error)

        try:
            left = self._read_axis(axes, self.left_axis, 'left')
            right = self._read_axis(axes, self.right_axis, 'right')
            left_turn = right_turn = 0.0
            if self.left_turn_axis >= 0:
                left_turn = self._read_axis(
                    axes, self.left_turn_axis, 'left turn')
                right_turn = self._read_axis(
                    axes, self.right_turn_axis, 'right turn')
            enable_axis_value = None
            if self.enable_axis >= 0:
                enable_axis_value = self._read_axis(
                    axes, self.enable_axis, 'enable')

            enable_button_pressed = None
            if self.enable_button >= 0:
                enable_button_pressed = self._read_button(
                    buttons, self.enable_button, 'enable')

            fast_pressed = False
            if self.fast_mode_button >= 0:
                fast_pressed = self._read_button(
                    buttons, self.fast_mode_button, 'fast-mode')
        except _MalformedJoy as exc:
            return self.invalidate(str(exc))

        if self.invert_left:
            left *= -1.0
        if self.invert_right:
            right *= -1.0

        left_command = _apply_deadband(
            _clamp(left, -1.0, 1.0), self.deadband)
        right_command = _apply_deadband(
            _clamp(right, -1.0, 1.0), self.deadband)
        # ROS joystick convention: stick left is positive.
        spin_command = _apply_deadband(
            _clamp(left_turn, -1.0, 1.0), self.deadband)
        right_turn_command = _apply_deadband(
            _clamp(right_turn, -1.0, 1.0), self.deadband)
        neutral = (
            left_command == 0.0 and right_command == 0.0
            and spin_command == 0.0 and right_turn_command == 0.0)

        if not self.require_enable:
            enabled = True
        elif enable_button_pressed is not None:
            enabled = enable_button_pressed
        elif enable_axis_value is not None:
            enabled = enable_axis_value > self.enable_axis_threshold
        else:
            enabled = False

        if not enabled:
            self._require_release = False
            return self._disarm(require_release=False)
        if self._require_release:
            return MixerOutput(OutputKind.SILENT)

        if self.state is DeadmanState.DISARMED:
            self.state = DeadmanState.WAIT_FOR_NEUTRAL
            self._owns_control = True

        if self.state is DeadmanState.WAIT_FOR_NEUTRAL:
            if neutral:
                self.state = DeadmanState.ARMED
            return MixerOutput(OutputKind.STOP)

        scale_at_low_speed = self.max_linear * self.low_speed_scale
        track_width = (
            self.track_width if abs(self.track_width) > 1e-6 else 1e-6)
        if (spin_command != 0.0 and left_command == 0.0
                and right_command == 0.0 and right_turn_command == 0.0):
            # Left stick sideways alone: spin in place at the one-stick rate.
            spin_scale = (
                scale_at_low_speed * self.single_stick_low_speed_scale)
            return MixerOutput(
                OutputKind.DRIVE, 0.0, spin_command / track_width * spin_scale)
        single_stick_active = (left_command == 0.0) != (
            right_command == 0.0)
        scale = (self.max_linear if fast_pressed else scale_at_low_speed)
        if single_stick_active:
            scale = (
                scale_at_low_speed * self.single_stick_low_speed_scale)

        linear_x = (left_command + right_command) * 0.5 * scale
        turn = right_command - left_command
        if left_command * right_command < 0.0:
            # Opposite sticks would double the turn rate; cap it to what one
            # fully deflected stick produces.
            turn = _clamp(turn, -1.0, 1.0)
            scale = (
                scale_at_low_speed * self.single_stick_low_speed_scale)
        angular_z = turn / track_width * scale
        return MixerOutput(OutputKind.DRIVE, linear_x, angular_z)
