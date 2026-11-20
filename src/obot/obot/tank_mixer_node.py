#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped

from .tank_mixer_safety import (
    OutputKind,
    TankMixerController,
)


DEFAULT_LOW_SPEED_SCALE = 0.35


class TankMixerNode(Node):
    def __init__(self, **node_kwargs):
        super().__init__("tank_mixer_node", **node_kwargs)

        # ---------------- Topics ----------------
        self.declare_parameter("joy_topic", "/joy")

        # This is an input to the bot-side twist_mux.
        self.declare_parameter("cmd_vel_out", "/cmd_vel_teleop")

        # Frame placed in the TwistStamped header.
        self.declare_parameter("frame_id", "base_link")

        # ---------------- Axes (tank) ----------------
        self.declare_parameter("left_axis", 1)
        self.declare_parameter("right_axis", 4)
        # Left stick sideways alone spins in place. The right stick's
        # sideways axis must also be inside the deadband. -1 disables both.
        self.declare_parameter("left_turn_axis", 0)
        self.declare_parameter("right_turn_axis", 3)

        self.declare_parameter("invert_left", False)
        self.declare_parameter("invert_right", False)

        # ---------------- Feel / scaling ----------------
        # Sticks are non-proportional: above this fraction of
        # travel the stick reads as fully on, otherwise off.
        self.declare_parameter("deadband", 0.50)
        self.declare_parameter("track_width", 0.40)   # meters
        self.declare_parameter("max_linear", 1.0)     # m/s

        # ---------------- Safety / watchdog ----------------
        self.declare_parameter("joy_timeout_sec", 0.5)

        # ---------------- Hold-to-drive (enable) ----------------
        # Choose ONE:
        # - enable_button >= 0  (pressed == 1)
        # - enable_axis >= 0    (value > enable_axis_threshold)
        #
        # Set both to -1 to disable gating.
        self.declare_parameter("enable_button", -1)
        self.declare_parameter("enable_axis", -1)
        self.declare_parameter("enable_axis_threshold", 0.5)
        self.declare_parameter("require_enable", True)

        # ---------------- Speed behavior ----------------
        # Base operation is low speed while enabled.
        # Holding fast_mode_button (right bumper) is a simple
        # high-speed boost: full speed while held, low speed
        # otherwise, regardless of stick direction.
        self.declare_parameter("fast_mode_button", 5)
        self.declare_parameter(
            "low_speed_scale",
            DEFAULT_LOW_SPEED_SCALE
        )

        # Backward-compatible alias for older launch/config files.
        self.declare_parameter(
            "slow_mode_scale",
            DEFAULT_LOW_SPEED_SCALE
        )

        # ---------------- Single-stick low speed ----------------
        # If only one tank stick is engaged, apply additional
        # scale to reduce speed.
        self.declare_parameter(
            "single_stick_low_speed_scale",
            0.50
        )

        # ---------------- Load parameters ----------------
        self.joy_topic = self.get_parameter(
            "joy_topic"
        ).value

        self.cmd_vel_out = self.get_parameter(
            "cmd_vel_out"
        ).value

        self.frame_id = self.get_parameter(
            "frame_id"
        ).value

        self.left_axis = int(
            self.get_parameter("left_axis").value
        )

        self.right_axis = int(
            self.get_parameter("right_axis").value
        )

        self.left_turn_axis = int(
            self.get_parameter("left_turn_axis").value
        )

        self.right_turn_axis = int(
            self.get_parameter("right_turn_axis").value
        )

        self.invert_left = bool(
            self.get_parameter("invert_left").value
        )

        self.invert_right = bool(
            self.get_parameter("invert_right").value
        )

        self.deadband = float(
            self.get_parameter("deadband").value
        )

        self.track_width = float(
            self.get_parameter("track_width").value
        )

        self.max_linear = float(
            self.get_parameter("max_linear").value
        )

        self.joy_timeout_sec = float(
            self.get_parameter("joy_timeout_sec").value
        )

        self.enable_button = int(
            self.get_parameter("enable_button").value
        )

        self.enable_axis = int(
            self.get_parameter("enable_axis").value
        )

        self.enable_axis_threshold = float(
            self.get_parameter(
                "enable_axis_threshold"
            ).value
        )

        self.require_enable = bool(
            self.get_parameter("require_enable").value
        )

        self.fast_mode_button = int(
            self.get_parameter("fast_mode_button").value
        )

        self.low_speed_scale = float(
            self.get_parameter("low_speed_scale").value
        )

        legacy_slow_mode_scale = float(
            self.get_parameter("slow_mode_scale").value
        )

        if (
            self.low_speed_scale == DEFAULT_LOW_SPEED_SCALE
            and legacy_slow_mode_scale != DEFAULT_LOW_SPEED_SCALE
        ):
            self.low_speed_scale = legacy_slow_mode_scale

        self.single_stick_low_speed_scale = float(
            self.get_parameter(
                "single_stick_low_speed_scale"
            ).value
        )

        # ---------------- State ----------------
        self.last_joy_time = None
        self._last_invalid_warning_time = None
        self.controller = TankMixerController(
            left_axis=self.left_axis,
            right_axis=self.right_axis,
            left_turn_axis=self.left_turn_axis,
            right_turn_axis=self.right_turn_axis,
            invert_left=self.invert_left,
            invert_right=self.invert_right,
            deadband=self.deadband,
            track_width=self.track_width,
            max_linear=self.max_linear,
            enable_button=self.enable_button,
            enable_axis=self.enable_axis,
            enable_axis_threshold=self.enable_axis_threshold,
            require_enable=self.require_enable,
            fast_mode_button=self.fast_mode_button,
            low_speed_scale=self.low_speed_scale,
            single_stick_low_speed_scale=self.single_stick_low_speed_scale,
        )

        # ---------------- ROS interfaces ----------------
        self.publisher = self.create_publisher(
            TwistStamped,
            self.cmd_vel_out,
            10
        )

        self._manual_driving = False

        self.subscription = self.create_subscription(
            Joy,
            self.joy_topic,
            self.joy_callback,
            10
        )

        self.timer = self.create_timer(
            0.05,
            self.watchdog
        )

        self.get_logger().info(
            "Tank mixer started | "
            f"axes(L,R)=({self.left_axis},{self.right_axis}) "
            f"stick_threshold={self.deadband} "
            f"enable_button={self.enable_button} "
            f"enable_axis={self.enable_axis} "
            f"fast_mode_button={self.fast_mode_button} "
            f"low_speed_scale={self.low_speed_scale} "
            f"single_stick_scale="
            f"{self.single_stick_low_speed_scale} "
            f"cmd_vel_out={self.cmd_vel_out} "
            f"frame_id={self.frame_id}"
        )

        self.get_logger().info(
            "Non-proportional sticks: above "
            f"{self.deadband * 100:.0f}% travel is full on, "
            "below is off. Holding fast_mode_button boosts to "
            "full speed."
        )
        if self.controller.configuration_error:
            self.get_logger().error(
                'Tank mixer disabled by invalid configuration: '
                f'{self.controller.configuration_error}')

    def _warn_invalid_joy(self, warning):
        now = time.monotonic()
        if (self._last_invalid_warning_time is None
                or now - self._last_invalid_warning_time >= 2.0):
            self.get_logger().warn(
                f'Ignoring malformed or unsafe Joy input: {warning}')
            self._last_invalid_warning_time = now

    def _publish_twist(
        self,
        linear_x: float = 0.0,
        angular_z: float = 0.0
    ):
        """
        Publish a stamped velocity command.

        The timestamp is generated immediately before publishing so
        downstream twist_mux and diff_drive_controller receive a fresh
        command timestamp.
        """
        msg = TwistStamped()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.header.frame_id = self.frame_id

        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)

        self.publisher.publish(msg)

    # ---------- Callbacks ----------

    def joy_callback(self, msg: Joy):
        self.last_joy_time = self.get_clock().now()
        axes = getattr(msg, 'axes', None)
        buttons = getattr(msg, 'buttons', None)
        output = self.controller.process(axes, buttons)
        if output.warning:
            self._warn_invalid_joy(output.warning)
        if output.kind is OutputKind.STOP:
            if self._manual_driving:
                self._publish_twist()
                self._manual_driving = False
        elif output.kind is OutputKind.DRIVE:
            moving = (
                abs(output.linear_x) > 1e-9
                or abs(output.angular_z) > 1e-9
            )
            if moving:
                self._manual_driving = True
                self._publish_twist(
                    linear_x=output.linear_x,
                    angular_z=output.angular_z)
            elif self._manual_driving:
                self._publish_twist()
                self._manual_driving = False

    def watchdog(self):
        if self.joy_timeout_sec <= 0:
            return

        if self.last_joy_time is None:
            return

        now = self.get_clock().now()

        dt = (
            now - self.last_joy_time
        ).nanoseconds / 1e9

        if dt > self.joy_timeout_sec:
            output = self.controller.timeout()
            if output.kind is OutputKind.STOP:
                self.get_logger().info(
                    "Watchdog timeout: "
                    f"no Joy message for {dt:.3f}s "
                    f"(limit={self.joy_timeout_sec:.3f}s); "
                    "publishing stop"
                )

                self._publish_twist()

            self.last_joy_time = None


def main():
    rclpy.init()

    node = TankMixerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send one final stop command before shutting down.
        try:
            node._publish_twist()
        except Exception:
            pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
