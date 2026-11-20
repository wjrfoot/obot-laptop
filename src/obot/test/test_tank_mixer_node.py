import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from sensor_msgs.msg import Joy

import pytest

from obot.tank_mixer_node import TankMixerNode
from obot.tank_mixer_safety import DeadmanState


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    if not rclpy.ok():
        rclpy.init()
        initialized_here = True
    else:
        initialized_here = False
    yield
    if initialized_here and rclpy.ok():
        rclpy.shutdown()


class Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        assert isinstance(message, TwistStamped)
        self.messages.append(message)


@pytest.fixture
def node():
    n = TankMixerNode(parameter_overrides=[
        Parameter('enable_button', value=4),
    ])
    n.publisher = Recorder()
    yield n
    n.destroy_node()


def joy(left=0.0, right=0.0, enabled=True, fast=False):
    message = Joy()
    message.axes = [0.0] * 6
    message.axes[1] = left
    message.axes[4] = right
    message.buttons = [0] * 11
    message.buttons[4] = int(enabled)
    message.buttons[5] = int(fast)
    return message


def test_ros_node_stops_on_f1_release_then_hands_off_silently(node):
    node.joy_callback(joy())
    node.joy_callback(joy(left=1.0))
    assert node.publisher.messages[-1].twist.linear.x != 0.0
    before_release = len(node.publisher.messages)

    node.joy_callback(joy(left=1.0, enabled=False))
    assert len(node.publisher.messages) == before_release + 1
    stop = node.publisher.messages[-1]
    assert stop.twist.linear.x == stop.twist.angular.z == 0.0
    assert node.controller.state is DeadmanState.DISARMED

    node.joy_callback(joy(enabled=False))
    assert len(node.publisher.messages) == before_release + 1


def test_ros_timeout_publishes_stop_and_requires_fresh_enable(node):
    node.joy_callback(joy())
    node.joy_callback(joy(left=1.0))
    assert node.publisher.messages[-1].twist.linear.x != 0.0

    node.last_joy_time = (
        node.get_clock().now() - Duration(seconds=1.0))
    node.watchdog()
    stop = node.publisher.messages[-1]
    assert stop.twist.linear.x == stop.twist.angular.z == 0.0
    assert node.controller.state is DeadmanState.DISARMED

    previous_count = len(node.publisher.messages)
    node.joy_callback(joy())
    assert len(node.publisher.messages) == previous_count
    node.joy_callback(joy(enabled=False))
    node.joy_callback(joy())
    assert len(node.publisher.messages) == previous_count + 1
    assert node.publisher.messages[-1].twist.linear.x == 0.0
    node.joy_callback(joy(right=1.0))
    assert node.publisher.messages[-1].twist.linear.x != 0.0
