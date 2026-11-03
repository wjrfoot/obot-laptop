#!/usr/bin/env python3
"""Single-stick differential-drive teleoperation for OBot (ROS 2 Jazzy)."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def stick_response(value, deadband, proportional):
    value = clamp(value, -1.0, 1.0)
    if abs(value) <= deadband:
        return 0.0
    if not proportional:
        return 1.0 if value > 0 else -1.0
    return (1.0 if value > 0 else -1.0) * (abs(value) - deadband) / (1.0 - deadband)


class SingleStickMixerNode(Node):
    def __init__(self):
        super().__init__('single_stick_mixer_node')
        defaults = {
            'joy_topic': '/joy', 'cmd_vel_out': '/cmd_vel_teleop',
            'frame_id': 'base_link', 'linear_axis': 1, 'angular_axis': 0,
            'invert_linear': False, 'invert_angular': False,
            'deadband': 0.50, 'proportional': False,
            'track_width': 0.486, 'max_linear': 0.5,
            'low_speed_scale': 0.5, 'fast_mode_button': 5,
            'enable_button': 4, 'require_enable': True,
            'joy_timeout_sec': 0.5,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        for key in defaults:
            setattr(self, key, self.get_parameter(key).value)
        if not (0.0 <= self.deadband < 1.0):
            raise ValueError('deadband must be between 0 and 1 (exclusive of 1)')
        if self.track_width <= 0 or self.max_linear <= 0:
            raise ValueError('track_width and max_linear must be positive')
        if not (0 < self.low_speed_scale <= 1):
            raise ValueError('low_speed_scale must be in (0,1]')
        self.publisher = self.create_publisher(TwistStamped, self.cmd_vel_out, 10)
        self.subscription = self.create_subscription(Joy, self.joy_topic, self.on_joy, 10)
        self.timer = self.create_timer(0.05, self.watchdog)
        self.last_joy_time = None
        self.teleop_active = False
        self.get_logger().info(
            f'Single stick: axes linear={self.linear_axis}, angular={self.angular_axis}; '
            f'deadband={self.deadband}; proportional={self.proportional}; '
            f'max_linear={self.max_linear}; track_width={self.track_width}; '
            f'output={self.cmd_vel_out}'
        )

    @staticmethod
    def button(msg, index):
        return 0 <= index < len(msg.buttons) and bool(msg.buttons[index])

    @staticmethod
    def axis(msg, index):
        return float(msg.axes[index]) if 0 <= index < len(msg.axes) else 0.0

    def publish(self, linear=0.0, angular=0.0):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self.frame_id
        cmd.twist.linear.x = float(linear)
        cmd.twist.angular.z = float(angular)
        self.publisher.publish(cmd)

    def stop(self):
        if self.teleop_active:
            self.publish()
        self.teleop_active = False

    def on_joy(self, msg):
        self.last_joy_time = self.get_clock().now()
        if self.require_enable and not self.button(msg, self.enable_button):
            self.stop()
            return
        # Fail safe on a controller with missing axes.
        if not (0 <= self.linear_axis < len(msg.axes) and
                0 <= self.angular_axis < len(msg.axes)):
            self.stop()
            return
        self.teleop_active = True
        forward = self.axis(msg, self.linear_axis)
        turn = self.axis(msg, self.angular_axis)
        if self.invert_linear:
            forward = -forward
        if self.invert_angular:
            turn = -turn
        forward = stick_response(forward, self.deadband, self.proportional)
        turn = stick_response(turn, self.deadband, self.proportional)
        speed = self.max_linear * (
            1.0 if self.button(msg, self.fast_mode_button) else self.low_speed_scale
        )
        # Convert forward/turn to wheel speeds and normalize so that
        # simultaneous steering never exceeds the configured wheel-speed limit.
        left = forward - turn
        right = forward + turn
        normalization = max(1.0, abs(left), abs(right))
        left = left / normalization * speed
        right = right / normalization * speed
        linear = (left + right) / 2.0
        angular = (right - left) / self.track_width
        self.publish(linear, angular)

    def watchdog(self):
        if self.joy_timeout_sec <= 0 or self.last_joy_time is None:
            return
        elapsed = (self.get_clock().now() - self.last_joy_time).nanoseconds / 1e9
        if elapsed > self.joy_timeout_sec:
            if self.teleop_active:
                self.get_logger().warn('Joystick timeout: stopping')
            self.stop()
            self.last_joy_time = None


def main(args=None):
    rclpy.init(args=args)
    node = SingleStickMixerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
