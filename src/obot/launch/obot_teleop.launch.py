from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    joy_topic = LaunchConfiguration("joy_topic")
    cmd_vel_joy = LaunchConfiguration("cmd_vel_joy")
    left_axis = LaunchConfiguration("left_axis")
    right_axis = LaunchConfiguration("right_axis")
    invert_left = LaunchConfiguration("invert_left")
    invert_right = LaunchConfiguration("invert_right")
    deadband = LaunchConfiguration("deadband")
    track_width = LaunchConfiguration("track_width")
    max_linear = LaunchConfiguration("max_linear")

    fast_mode_button = LaunchConfiguration("fast_mode_button")
    low_speed_scale = LaunchConfiguration("low_speed_scale")
    slow_mode_scale = LaunchConfiguration("slow_mode_scale")
    single_stick_low_speed_scale = LaunchConfiguration(
        "single_stick_low_speed_scale"
    )
    return LaunchDescription([
        # PS4 / joystick input
        DeclareLaunchArgument("joy_topic", default_value="/joy"),

        # Intermediate Twist topic from the generic SprayBot tank mixer.
        DeclareLaunchArgument("cmd_vel_joy", default_value="/cmd_vel_joy"),

        # Tank-stick mapping
        DeclareLaunchArgument("left_axis", default_value="1"),
        DeclareLaunchArgument("right_axis", default_value="4"),
        DeclareLaunchArgument("invert_left", default_value="false"),
        DeclareLaunchArgument("invert_right", default_value="false"),
        # Non-proportional sticks: above this fraction of travel
        # is full on, below is off.
        DeclareLaunchArgument("deadband", default_value="0.50"),

        # OBot calibrated drivetrain geometry.
        DeclareLaunchArgument("track_width", default_value="0.486"),

        # Conservative initial teleop limit. Raise later after testing.
        DeclareLaunchArgument("max_linear", default_value="1.5"),

        # Speed-mode controls: holding fast_mode_button (right
        # bumper) boosts to full speed.
        DeclareLaunchArgument("fast_mode_button", default_value="5"),
        DeclareLaunchArgument("low_speed_scale", default_value="0.50"),
        DeclareLaunchArgument("slow_mode_scale", default_value="0.50"),
        DeclareLaunchArgument(
            "single_stick_low_speed_scale",
            default_value="0.50",
        ),

        # Spray seconds per PS4 face button; 0 disables the button.
        DeclareLaunchArgument("circle_spray_sec", default_value="0.0"),
        DeclareLaunchArgument("triangle_spray_sec", default_value="0.0"),
        DeclareLaunchArgument("square_spray_sec", default_value="0.0"),
        DeclareLaunchArgument(
            "spray_valve_state_topic", default_value="/obot/valve/state"),
        DeclareLaunchArgument(
            "joy_feedback_topic", default_value="/joy/set_feedback"),

        # Existing robot-independent SprayBot tank mixer.
        # Publishes geometry_msgs/msg/TwistStamped on /cmd_vel_teleop.
        Node(
            package="obot",
            executable="tank_mixer_node",
            name="tank_mixer_node",
            output="screen",
            parameters=[{
                "joy_topic": joy_topic,
                "cmd_vel_out": "/cmd_vel_teleop",
                "left_axis": left_axis,
                "right_axis": right_axis,
                "invert_left": invert_left,
                "invert_right": invert_right,
                "deadband": deadband,
                "track_width": track_width,
                "max_linear": max_linear,
                "joy_timeout_sec": 0.5,

                "enable_button": 4,
                "require_enable": True,

                "fast_mode_button": fast_mode_button,
                "low_speed_scale": low_speed_scale,
                "slow_mode_scale": slow_mode_scale,
                "single_stick_low_speed_scale": single_stick_low_speed_scale,
            }],
        ),

        # Circle, Square and Triangle start timed sprays; Cross cancels.
        Node(
            package="obot_valve_controller",
            executable="ps4_valve_client",
            name="ps4_valve_client",
            output="screen",
            parameters=[{
                "joy_topic": joy_topic,
                "action_name": "/obot/spray",
                "valve_state_topic": LaunchConfiguration("spray_valve_state_topic"),
                "feedback_topic": LaunchConfiguration("joy_feedback_topic"),
                "triangle_button": 2,
                "circle_duration_sec": ParameterValue(
                    LaunchConfiguration("circle_spray_sec"), value_type=float),
                "square_duration_sec": ParameterValue(
                    LaunchConfiguration("square_spray_sec"), value_type=float),
                "triangle_duration_sec": ParameterValue(
                    LaunchConfiguration("triangle_spray_sec"), value_type=float),
            }],
        ),

    ])
