"""Map PS4 face buttons to timed spray valve commands.

Each face button has a duration parameter (seconds). A duration <= 0 (the
default) disables that button. A press publishes the duration on
/spray/valve_command; the ESP32 closes the valve when the time expires.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

# Button indices for the PS4 pad under joy_node on Linux (hid-sony driver).
BUTTONS = {
    'cross': 0,
    'circle': 1,
    'triangle': 2,
    'square': 3,
}


class JoySprayNode(Node):
    def __init__(self):
        super().__init__('joy_spray_node')
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('valve_command_topic', '/spray/valve_command')
        for name, index in BUTTONS.items():
            self.declare_parameter(f'{name}_duration_sec', 0.0)
            self.declare_parameter(f'{name}_button', index)

        self.durations = {}
        self.indices = {}
        for name in BUTTONS:
            duration = float(self.get_parameter(f'{name}_duration_sec').value)
            index = int(self.get_parameter(f'{name}_button').value)
            if duration > 0.0:
                self.durations[name] = duration
                self.indices[name] = index

        self.previous = {}
        self.pub = self.create_publisher(
            Float32, str(self.get_parameter('valve_command_topic').value), 10)
        self.create_subscription(
            Joy, str(self.get_parameter('joy_topic').value), self.on_joy, 10)
        self.get_logger().info(
            f'Enabled spray buttons: {self.durations or "none"}')

    def on_joy(self, msg):
        for name, duration in self.durations.items():
            index = self.indices[name]
            pressed = index < len(msg.buttons) and msg.buttons[index] == 1
            if pressed and not self.previous.get(name, False):
                self.pub.publish(Float32(data=duration))
                self.get_logger().info(f'{name}: spray {duration:.2f} s')
            self.previous[name] = pressed


def main(args=None):
    rclpy.init(args=args)
    node = JoySprayNode()
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
