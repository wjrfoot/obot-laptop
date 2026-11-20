import pytest

from obot.tank_mixer_safety import (
    DeadmanState,
    OutputKind,
    TankMixerController,
)


def new_controller(**overrides):
    settings = {
        'left_axis': 1,
        'right_axis': 4,
        'enable_button': 4,
        'fast_mode_button': 5,
        'deadband': 0.5,
        'track_width': 0.486,
        'max_linear': 1.0,
        'low_speed_scale': 0.5,
        'single_stick_low_speed_scale': 0.5,
        'require_enable': True,
    }
    settings.update(overrides)
    return TankMixerController(**settings)


def joy(left=0.0, right=0.0, enabled=True, fast=False):
    axes = [0.0] * 6
    axes[1] = left
    axes[4] = right
    buttons = [0] * 11
    buttons[4] = int(enabled)
    buttons[5] = int(fast)
    return axes, buttons


def send(controller, left=0.0, right=0.0, enabled=True, fast=False):
    return controller.process(*joy(left, right, enabled, fast))


def arm(controller):
    output = send(controller)
    assert output.kind is OutputKind.STOP
    assert controller.state is DeadmanState.ARMED


def test_initial_state_is_disarmed():
    assert new_controller().state is DeadmanState.DISARMED


def test_released_f1_does_not_permit_motion():
    controller = new_controller()
    output = send(controller, left=1.0, enabled=False)
    assert output.kind is OutputKind.SILENT
    assert controller.state is DeadmanState.DISARMED


def test_neutral_f1_press_arms_without_motion_on_transition():
    controller = new_controller()
    output = send(controller)
    assert output.kind is OutputKind.STOP
    assert controller.state is DeadmanState.ARMED
    assert output.linear_x == output.angular_z == 0.0


@pytest.mark.parametrize(('left', 'right'), [(1.0, 0.0), (0.0, -1.0)])
def test_displaced_stick_waits_for_neutral_without_motion(left, right):
    controller = new_controller()
    output = send(controller, left=left, right=right)
    assert output.kind is OutputKind.STOP
    assert output.linear_x == output.angular_z == 0.0
    assert controller.state is DeadmanState.WAIT_FOR_NEUTRAL


def test_returning_both_sticks_to_neutral_arms_on_a_stop():
    controller = new_controller()
    send(controller, left=1.0)
    output = send(controller)
    assert output.kind is OutputKind.STOP
    assert output.linear_x == output.angular_z == 0.0
    assert controller.state is DeadmanState.ARMED


@pytest.mark.parametrize(
    ('left', 'right', 'expected_angular'),
    [(1.0, 0.0, -1.0 / 0.486 * 0.25),
     (0.0, 1.0, 1.0 / 0.486 * 0.25)],
)
def test_each_stick_produces_expected_tank_command(
        left, right, expected_angular):
    controller = new_controller()
    arm(controller)
    output = send(controller, left=left, right=right)
    assert output.kind is OutputKind.DRIVE
    assert output.linear_x == pytest.approx(0.125)
    assert output.angular_z == pytest.approx(expected_angular)


def test_releasing_f1_while_moving_stops_and_disarms():
    controller = new_controller()
    arm(controller)
    assert send(controller, left=1.0).kind is OutputKind.DRIVE
    output = send(controller, left=1.0, enabled=False)
    assert output.kind is OutputKind.STOP
    assert controller.state is DeadmanState.DISARMED


def test_repressing_f1_with_a_displaced_stick_waits_for_neutral():
    controller = new_controller()
    arm(controller)
    send(controller, left=1.0)
    send(controller, left=1.0, enabled=False)
    output = send(controller, left=1.0)
    assert output.kind is OutputKind.STOP
    assert controller.state is DeadmanState.WAIT_FOR_NEUTRAL
    assert send(controller, left=1.0).kind is OutputKind.STOP


def test_release_center_and_repress_reenables_normal_operation():
    controller = new_controller()
    arm(controller)
    send(controller, left=1.0)
    send(controller, left=1.0, enabled=False)
    assert send(controller, enabled=False).kind is OutputKind.SILENT
    output = send(controller)
    assert output.kind is OutputKind.STOP
    assert controller.state is DeadmanState.ARMED
    assert send(controller, right=1.0).kind is OutputKind.DRIVE


def test_timeout_disarms_and_publishes_stop_when_teleop_owned_control():
    controller = new_controller()
    arm(controller)
    send(controller, left=1.0)
    output = controller.timeout()
    assert output.kind is OutputKind.STOP
    assert controller.state is DeadmanState.DISARMED


def test_timeout_recovery_requires_release_and_neutral_enable_sequence():
    controller = new_controller()
    arm(controller)
    send(controller, left=1.0)
    controller.timeout()

    output = send(controller, left=0.0)
    assert output.kind is OutputKind.SILENT
    assert controller.state is DeadmanState.DISARMED
    assert send(controller, enabled=False).kind is OutputKind.SILENT
    assert send(controller).kind is OutputKind.STOP
    assert controller.state is DeadmanState.ARMED
    assert send(controller, right=1.0).kind is OutputKind.DRIVE


def test_fast_mode_preserves_full_speed():
    controller = new_controller()
    arm(controller)
    output = send(controller, left=1.0, right=1.0, fast=True)
    assert output.kind is OutputKind.DRIVE
    assert output.linear_x == pytest.approx(1.0)
    assert output.angular_z == pytest.approx(0.0)


def test_single_stick_speed_reduction_is_preserved():
    controller = new_controller()
    arm(controller)
    output = send(controller, left=1.0, fast=True)
    assert output.kind is OutputKind.DRIVE
    assert output.linear_x == pytest.approx(0.125)


def test_axis_inversion_is_applied_before_tank_mixing():
    controller = new_controller(invert_left=True)
    arm(controller)
    output = send(controller, left=1.0)
    assert output.kind is OutputKind.DRIVE
    assert output.linear_x == pytest.approx(-0.125)
    assert output.angular_z == pytest.approx(1.0 / 0.486 * 0.25)


@pytest.mark.parametrize(
    ('axes', 'buttons'),
    [
        (None, joy()[1]),
        (joy()[0], None),
        (joy()[0], [0, 0, 0, 0]),
    ],
)
def test_malformed_joy_cannot_enable_motion(axes, buttons):
    controller = new_controller()
    output = controller.process(axes, buttons)
    assert output.kind is OutputKind.SILENT
    assert output.warning
    assert controller.state is DeadmanState.DISARMED


def test_malformed_joy_while_active_publishes_stop():
    controller = new_controller()
    arm(controller)
    send(controller, left=1.0)
    output = controller.process(None, joy()[1])
    assert output.kind is OutputKind.STOP
    assert output.warning
    assert controller.state is DeadmanState.DISARMED


@pytest.mark.parametrize(
    'overrides',
    [
        {'enable_button': -1},
        {'enable_button': 4, 'enable_axis': 2},
        {'left_axis': -1},
    ],
)
def test_invalid_configuration_fails_closed(overrides):
    controller = new_controller(**overrides)
    output = send(controller)
    assert output.kind is OutputKind.SILENT
    assert output.warning
    assert controller.state is DeadmanState.DISARMED


def test_enable_axis_remains_supported():
    controller = new_controller(enable_button=-1, enable_axis=0)
    axes, buttons = joy()
    axes[0] = 1.0
    output = controller.process(axes, buttons)
    assert output.kind is OutputKind.STOP
    assert controller.state is DeadmanState.ARMED


def test_require_enable_false_disables_deadman_gating():
    controller = new_controller(
        enable_button=-1, fast_mode_button=-1, require_enable=False)
    output = send(controller, left=1.0)
    assert output.kind is OutputKind.STOP
    assert controller.state is DeadmanState.WAIT_FOR_NEUTRAL


def test_f1_release_stops_once_then_silences_for_nav2_handoff():
    controller = new_controller()
    arm(controller)
    send(controller, right=1.0)
    assert send(controller, enabled=False).kind is OutputKind.STOP
    assert send(controller, enabled=False).kind is OutputKind.SILENT


SINGLE_STICK_ANGULAR = 1.0 / 0.486 * 0.25


@pytest.mark.parametrize(('left', 'right'), [(1.0, -1.0), (-1.0, 1.0)])
@pytest.mark.parametrize('fast', [False, True])
def test_opposite_sticks_rotate_at_single_stick_angular_speed(
        left, right, fast):
    controller = new_controller()
    arm(controller)
    output = send(controller, left=left, right=right, fast=fast)
    assert output.kind is OutputKind.DRIVE
    assert output.linear_x == pytest.approx(0.0)
    assert output.angular_z == pytest.approx(
        (right - left) / 2.0 * SINGLE_STICK_ANGULAR)
    assert abs(output.angular_z) == pytest.approx(SINGLE_STICK_ANGULAR)


def test_single_stick_angular_matches_opposite_stick_rotation():
    controller = new_controller()
    arm(controller)
    single = send(controller, right=1.0)
    opposite = send(controller, left=-1.0, right=1.0)
    assert opposite.angular_z == pytest.approx(single.angular_z)


def test_both_sticks_forward_preserves_straight_line_speed():
    controller = new_controller()
    arm(controller)
    output = send(controller, left=1.0, right=1.0)
    assert output.linear_x == pytest.approx(0.5)
    assert output.angular_z == pytest.approx(0.0)


@pytest.mark.parametrize(
    ('left', 'right'), [(0.3, -0.4), (-0.5, 0.5), (0.2, 0.0), (0.0, 0.0)])
def test_inputs_inside_deadband_produce_no_motion(left, right):
    controller = new_controller()
    arm(controller)
    output = send(controller, left=left, right=right)
    assert output.linear_x == output.angular_z == 0.0


def test_deflected_stick_with_other_inside_deadband_is_single_stick():
    controller = new_controller()
    arm(controller)
    output = send(controller, left=0.2, right=0.9)
    assert output.linear_x == pytest.approx(0.1)
    assert output.angular_z == pytest.approx(0.8 * SINGLE_STICK_ANGULAR)


def test_intermediate_stick_positions_remain_proportional():
    controller = new_controller()
    arm(controller)

    single_stick = send(controller, right=0.75)
    assert single_stick.linear_x == pytest.approx(0.0625)
    assert single_stick.angular_z == pytest.approx(
        0.5 * SINGLE_STICK_ANGULAR)

    both_forward = send(controller, left=0.75, right=1.0)
    assert both_forward.linear_x == pytest.approx(0.375)
    assert both_forward.angular_z == pytest.approx(0.5 / 0.486 * 0.5)


def test_intermediate_opposite_sticks_rotate_proportionally():
    controller = new_controller()
    arm(controller)
    output = send(controller, left=0.75, right=-0.6)
    assert output.linear_x == pytest.approx(0.075)
    assert output.angular_z == pytest.approx(-0.7 / 0.486 * 0.25)


def spin_controller(**overrides):
    return new_controller(left_turn_axis=0, right_turn_axis=3, **overrides)


def send_spin(controller, left_x=0.0, left_y=0.0, right_x=0.0, right_y=0.0):
    axes, buttons = joy(left_y, right_y)
    axes[0] = left_x
    axes[3] = right_x
    return controller.process(axes, buttons)


def test_left_stick_right_spins_right_and_left_spins_left():
    controller = spin_controller()
    send_spin(controller)
    right = send_spin(controller, left_x=-1.0)
    left = send_spin(controller, left_x=1.0)
    assert right.kind is left.kind is OutputKind.DRIVE
    assert right.linear_x == left.linear_x == 0.0
    assert right.angular_z == pytest.approx(-SINGLE_STICK_ANGULAR)
    assert left.angular_z == pytest.approx(SINGLE_STICK_ANGULAR)


def test_spin_is_proportional_and_deadbanded():
    controller = spin_controller()
    send_spin(controller)
    assert send_spin(controller, left_x=0.4).angular_z == 0.0
    out = send_spin(controller, left_x=0.75)
    assert out.angular_z == pytest.approx(0.5 * SINGLE_STICK_ANGULAR)


@pytest.mark.parametrize('kwargs', [
    {'left_y': 0.9}, {'right_y': 0.9}, {'right_x': 0.9}])
def test_spin_ignored_unless_other_inputs_in_deadzone(kwargs):
    controller = spin_controller()
    send_spin(controller)
    out = send_spin(controller, left_x=1.0, **kwargs)
    pure_spin = (out.linear_x == 0.0
                 and out.angular_z == pytest.approx(SINGLE_STICK_ANGULAR))
    assert not pure_spin


def test_sideways_stick_must_be_neutral_before_arming():
    controller = spin_controller()
    out = send_spin(controller, left_x=1.0)
    assert out.kind is OutputKind.STOP
    assert controller.state is DeadmanState.WAIT_FOR_NEUTRAL
    assert send_spin(controller).kind is OutputKind.STOP
    assert controller.state is DeadmanState.ARMED


def test_spin_disabled_by_default_in_controller():
    controller = new_controller()
    arm(controller)
    axes, buttons = joy()
    axes[0] = 1.0
    out = controller.process(axes, buttons)
    assert out.linear_x == out.angular_z == 0.0


@pytest.mark.parametrize('overrides', [
    {'left_turn_axis': 0, 'right_turn_axis': -1},
    {'left_turn_axis': 1, 'right_turn_axis': 3},
])
def test_invalid_turn_axis_configuration_fails_closed(overrides):
    controller = new_controller(**overrides)
    assert send(controller).kind is OutputKind.SILENT
    assert controller.configuration_error
