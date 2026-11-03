#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped


DEFAULT_LOW_SPEED_SCALE = 0.35


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def digitize_stick(x, threshold):
    """
    Non-proportional (on/off) stick response:
      |x| > threshold -> full +/-1
      otherwise -> 0
    """
    if abs(x) > threshold:
        return 1.0 if x > 0 else -1.0
    return 0.0


class TankMixerNode(Node):
    def __init__(self):
        super().__init__("tank_mixer_node")

        # ---------------- Topics ----------------
        self.declare_parameter("joy_topic", "/joy")

        # This is an input to the bot-side twist_mux.
        self.declare_parameter("cmd_vel_out", "/cmd_vel_teleop")

        # Frame placed in the TwistStamped header.
        self.declare_parameter("frame_id", "base_link")

        # ---------------- Axes (tank) ----------------
        self.declare_parameter("left_axis", 1)
        self.declare_parameter("right_axis", 4)

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

        # ---------------- ROS interfaces ----------------
        self.publisher = self.create_publisher(
            TwistStamped,
            self.cmd_vel_out,
            10
        )

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

    # ---------- Helpers ----------

    def _get_axis(self, msg: Joy, idx: int) -> float:
        if idx < 0:
            return 0.0

        if idx >= len(msg.axes):
            self.get_logger().warn(
                f"Axis index {idx} out of range "
                f"(len={len(msg.axes)})"
            )
            return 0.0

        return float(msg.axes[idx])

    def _get_button(self, msg: Joy, idx: int) -> bool:
        if idx < 0:
            return False

        if idx >= len(msg.buttons):
            self.get_logger().warn(
                f"Button index {idx} out of range "
                f"(len={len(msg.buttons)})"
            )
            return False

        return bool(msg.buttons[idx])

    def _is_enabled(self, msg: Joy) -> bool:
        # If gating is off, always enabled.
        if not self.require_enable:
            return True

        # Button gating.
        if self.enable_button >= 0:
            return self._get_button(
                msg,
                self.enable_button
            )

        # Axis gating (e.g. trigger).
        if self.enable_axis >= 0:
            val = self._get_axis(
                msg,
                self.enable_axis
            )
            return val > self.enable_axis_threshold

        # If require_enable is True but no control is configured,
        # treat as disabled for safety.
        return False

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

        # Hold-to-drive.
        enabled = self._is_enabled(msg)

        if not enabled:
            # On deadman release, publish exactly one stop command,
            # then remain silent so twist_mux can hand control to Nav2.
            if getattr(self, "_teleop_was_enabled", False):
                self._publish_twist()

            self._teleop_was_enabled = False
            return

        self._teleop_was_enabled = True

        # Read tank axes.
        L = self._get_axis(
            msg,
            self.left_axis
        )

        R = self._get_axis(
            msg,
            self.right_axis
        )

        if self.invert_left:
            L *= -1.0

        if self.invert_right:
            R *= -1.0

        # Non-proportional sticks: fully on above threshold,
        # off below it.
        L = digitize_stick(
            clamp(L, -1.0, 1.0),
            self.deadband
        )

        R = digitize_stick(
            clamp(R, -1.0, 1.0),
            self.deadband
        )

        # Base behavior: low speed when enabled.
        low_speed_scale = (
            self.max_linear *
            self.low_speed_scale
        )

        single_stick_active = (
            (L == 0.0) != (R == 0.0)
        )

        # High-speed mode: holding fast_mode_button (right
        # bumper) simply boosts to full speed while held.
        fast_pressed = self._get_button(
            msg,
            self.fast_mode_button
        )

        scale = (
            self.max_linear
            if fast_pressed
            else low_speed_scale
        )

        # If only one stick is engaged, run at reduced low speed.
        if single_stick_active:
            scale = (
                low_speed_scale *
                self.single_stick_low_speed_scale
            )

        # Tank mixing to linear/angular velocity.
        v = (L + R) * 0.5 * scale

        tw = (
            self.track_width
            if abs(self.track_width) > 1e-6
            else 1e-6
        )

        w = (R - L) / tw * scale

        self._publish_twist(
            linear_x=v,
            angular_z=w
        )

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
            if getattr(self, "_teleop_was_enabled", False):
                self.get_logger().info(
                    "Watchdog timeout: "
                    f"no Joy message for {dt:.3f}s "
                    f"(limit={self.joy_timeout_sec:.3f}s); "
                    "publishing stop"
                )

                self._publish_twist()

            self._teleop_was_enabled = False
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
