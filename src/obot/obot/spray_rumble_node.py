"""
Rumble the joystick at maximum intensity while the spray valve is open.

The node follows the actual valve state and stops rumbling if valve-state
messages become stale, so a lost connection cannot leave feedback active.
"""
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JoyFeedback
from std_msgs.msg import Bool


class SprayRumbleNode(Node):
    def __init__(self):
        super().__init__('spray_rumble_node')
        self.declare_parameter('valve_state_topic', '/spray/valve_state')
        self.declare_parameter('feedback_topic', '/joy/set_feedback')
        self.declare_parameter('rumble_refresh_sec', 0.5)
        self.declare_parameter('state_timeout_sec', 0.5)

        self.rumble_refresh_sec = float(
            self.get_parameter('rumble_refresh_sec').value)
        self.state_timeout_sec = float(
            self.get_parameter('state_timeout_sec').value)
        if self.rumble_refresh_sec <= 0.0:
            raise ValueError('rumble_refresh_sec must be greater than zero')
        if self.state_timeout_sec <= 0.0:
            raise ValueError('state_timeout_sec must be greater than zero')

        self.valve_open = False
        self.last_state_time = None
        self.last_rumble_time = None
        feedback_topic = str(self.get_parameter('feedback_topic').value)
        valve_state_topic = str(self.get_parameter('valve_state_topic').value)
        self.feedback_pub = self.create_publisher(JoyFeedback, feedback_topic, 10)
        self.state_sub = self.create_subscription(
            Bool, valve_state_topic, self.on_valve_state, 10)
        self.timer = self.create_timer(0.1, self.on_timer)
        self.get_logger().info(
            f'Rumbling at maximum intensity while {valve_state_topic} is true; '
            f'joystick feedback topic: {feedback_topic}')

    def publish_rumble(self, intensity):
        feedback = JoyFeedback()
        feedback.type = JoyFeedback.TYPE_RUMBLE
        feedback.id = 0
        feedback.intensity = intensity
        self.feedback_pub.publish(feedback)

    def on_valve_state(self, msg):
        now = time.monotonic()
        self.last_state_time = now
        is_open = bool(msg.data)
        if is_open and not self.valve_open:
            self.valve_open = True
            self.publish_rumble(1.0)
            self.last_rumble_time = now
        elif not is_open:
            if self.valve_open:
                self.publish_rumble(0.0)
            self.valve_open = False
            self.last_rumble_time = None

    def on_timer(self):
        if not self.valve_open:
            return

        now = time.monotonic()
        if (self.last_state_time is None or
                now - self.last_state_time > self.state_timeout_sec):
            self.valve_open = False
            self.last_rumble_time = None
            self.publish_rumble(0.0)
            self.get_logger().warn('Valve-state messages timed out; rumble stopped')
            return

        if (self.last_rumble_time is None or
                now - self.last_rumble_time >= self.rumble_refresh_sec):
            self.publish_rumble(1.0)
            self.last_rumble_time = now

    def destroy_node(self):
        if self.context.ok():
            self.publish_rumble(0.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SprayRumbleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
