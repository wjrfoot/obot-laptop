"""Record odom-relative pose and spray length to a text file per session.

L2 opens a new file named odom<timestamp>.txt in the output directory and R2
closes it. While a file is open, each press of a spray button with a duration
> 0 appends one line: x y yaw spray_sec. The pose is expressed relative to the
odom pose at the moment the file was opened (x forward, y left, yaw in radians).
"""
import math
import os
from datetime import datetime

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Joy

SPRAY_BUTTONS = {
    'cross': 0,
    'circle': 1,
    'triangle': 2,
    'square': 3,
}


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class OdomSprayRecord(Node):
    def __init__(self):
        super().__init__('odomsprayrecord')
        self.declare_parameter('output_dir', os.path.expanduser('~/odom_records'))
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('odom_topic', '/diff_drive_controller/odom')
        self.declare_parameter('start_button', 6)  # L2
        self.declare_parameter('stop_button', 7)   # R2
        for name, index in SPRAY_BUTTONS.items():
            self.declare_parameter(f'{name}_duration_sec', 0.0)
            self.declare_parameter(f'{name}_button', index)

        self.output_dir = os.path.expanduser(str(self.get_parameter('output_dir').value))
        self.start_button = int(self.get_parameter('start_button').value)
        self.stop_button = int(self.get_parameter('stop_button').value)
        self.sprays = {}
        for name in SPRAY_BUTTONS:
            duration = float(self.get_parameter(f'{name}_duration_sec').value)
            if duration > 0.0:
                self.sprays[name] = (int(self.get_parameter(f'{name}_button').value), duration)

        self.pose = None  # latest (x, y, yaw) in the odom frame
        self.origin = None
        self.file = None
        self.previous = {}

        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value), self.on_odom, 10)
        self.create_subscription(
            Joy, str(self.get_parameter('joy_topic').value), self.on_joy, 10)
        self.get_logger().info(
            f'output_dir={self.output_dir}; enabled spray buttons: '
            f'{ {n: d for n, (_, d) in self.sprays.items()} or "none"}')

    def on_odom(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    def pressed(self, msg, key, index):
        down = 0 <= index < len(msg.buttons) and msg.buttons[index] == 1
        edge = down and not self.previous.get(key, False)
        self.previous[key] = down
        return edge

    def on_joy(self, msg):
        if self.pressed(msg, 'start', self.start_button):
            self.open_file()
        if self.pressed(msg, 'stop', self.stop_button):
            self.close_file()
        for name, (index, duration) in self.sprays.items():
            if self.pressed(msg, name, index):
                self.record(duration)

    def open_file(self):
        if self.pose is None:
            self.get_logger().warn('No odometry received yet; cannot open a file')
            return
        self.close_file()
        try:
            os.makedirs(self.output_dir, exist_ok=True)
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            path = os.path.join(self.output_dir, f'odom{stamp}.txt')
            self.file = open(path, 'w')
        except OSError as exc:
            self.get_logger().error(f'Cannot open record file: {exc}')
            return
        self.origin = self.pose
        self.file.write('# x_m y_m yaw_rad spray_sec (relative to start pose)\n')
        self.file.flush()
        self.get_logger().info(f'Recording to {path}')

    def close_file(self):
        if self.file is not None:
            self.file.close()
            self.file = None
            self.get_logger().info('Record file closed')

    def record(self, duration):
        if self.file is None or self.pose is None:
            return
        ox, oy, oyaw = self.origin
        x, y, yaw = self.pose
        dx, dy = x - ox, y - oy
        c, s = math.cos(oyaw), math.sin(oyaw)
        rel_x = c * dx + s * dy
        rel_y = -s * dx + c * dy
        rel_yaw = wrap_angle(yaw - oyaw)
        self.file.write(f'{rel_x:.3f} {rel_y:.3f} {rel_yaw:.3f} {duration:.2f}\n')
        self.file.flush()

    def destroy_node(self):
        self.close_file()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OdomSprayRecord()
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
