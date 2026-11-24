import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.utilities.evaluate_parameters import evaluate_parameters


LAUNCH_FILE = (
    Path(__file__).parents[1] / 'launch' / 'obot_teleop.launch.py')
SPEC = importlib.util.spec_from_file_location('obot_teleop_launch', LAUNCH_FILE)
LAUNCH_MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(LAUNCH_MODULE)


def node_metadata(node):
    return (
        node._Node__package,
        node._Node__node_executable,
        node._Node__node_name,
    )


def node_parameters(node):
    return {
        ''.join(part.text for part in key if hasattr(part, 'text')): value
        for key, value in node._Node__parameters[0].items()
    }


def test_teleop_launch_starts_one_instance_of_each_required_node():
    description = LAUNCH_MODULE.generate_launch_description()
    nodes = [
        action for action in description.entities
        if isinstance(action, Node)
    ]
    expected = {
        ('obot', 'tank_mixer_node', 'tank_mixer_node'),
        ('obot_valve_controller', 'ps4_valve_client', 'ps4_valve_client'),
    }
    actual = [
        node_metadata(node) for node in nodes
    ]
    assert len(actual) == len(expected)
    assert set(actual) == expected


def test_spray_client_uses_configured_topics_and_durations():
    description = LAUNCH_MODULE.generate_launch_description()
    clients = [
        action for action in description.entities
        if isinstance(action, Node)
        and action._Node__node_name == 'ps4_valve_client'
    ]
    assert len(clients) == 1
    params = node_parameters(clients[0])
    context = LaunchContext()
    context.launch_configurations.update({
        'joy_topic': '/custom/joy',
        'spray_valve_state_topic': '/custom/valve/state',
        'joy_feedback_topic': '/custom/feedback',
        'circle_spray_sec': '1.25',
        'square_spray_sec': '2.5',
        'triangle_spray_sec': '3.75',
    })
    evaluated = evaluate_parameters(context, clients[0]._Node__parameters)[0]
    assert evaluated['joy_topic'] == '/custom/joy'
    assert evaluated['action_name'] == '/obot/spray'
    assert evaluated['valve_state_topic'] == '/custom/valve/state'
    assert evaluated['feedback_topic'] == '/custom/feedback'
    assert evaluated['triangle_button'] == 2
    for name, value in (
        ('circle', 1.25), ('square', 2.5), ('triangle', 3.75)
    ):
        duration = params[f'{name}_duration_sec']
        assert isinstance(duration, ParameterValue)
        assert duration.value_type is float
        assert evaluated[f'{name}_duration_sec'] == value


def test_spray_defaults_preserve_zero_disabled_and_correct_topics():
    description = LAUNCH_MODULE.generate_launch_description()
    arguments = {
        action.name: action.default_value[0].text
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
    }
    assert arguments['joy_topic'] == '/joy'
    assert arguments['spray_valve_state_topic'] == '/obot/valve/state'
    assert arguments['joy_feedback_topic'] == '/joy/set_feedback'
    for name in ('circle', 'square', 'triangle'):
        assert arguments[f'{name}_spray_sec'] == '0.0'


def test_obot_launch_requires_f1_deadman():
    description = LAUNCH_MODULE.generate_launch_description()
    launch_arguments = {
        action.name: action.default_value
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
    }
    assert 'enable_button' not in launch_arguments
    assert 'require_enable' not in launch_arguments
    mixer = next(
        action for action in description.entities
        if isinstance(action, Node)
        and action._Node__node_name == 'tank_mixer_node')
    params = node_parameters(mixer)
    assert params['enable_button'] == 4
    assert params['require_enable'] is True
