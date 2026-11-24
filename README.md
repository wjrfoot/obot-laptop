# OBot Laptop

## OBot-2 watchdog routing discrepancy

On the inspected `pi-bot-2` deployment, `obot2_watchdog` published health
telemetry but no `/watchdog_lock` or `/cmd_vel_watchdog` topic. The running
`twist_mux` had joystick, motion-test, and navigation inputs but no configured
watchdog lock. This differs from the fail-closed watchdog-lock configuration
present in the robot repository. Investigate and reconcile the deployed
configuration separately before relying on the watchdog to inhibit motion.
The watchdog's motor fault monitoring and recovery should not be confused with
a verified ROS velocity-command lock.

## Camera motion controls

The lower front-camera overlay contains click-to-drive controls on the left and
hold-to-move controls on the right. Releasing a direction, moving outside its
button, or losing window focus requests a stop. A directional press cancels
the current click motion and waits for acknowledgement before moving; canceled
motions do not resume. The robot-side motion node remains the sole publisher
to its configured `twist_mux` input. A short manual-command lease stops
refreshing motion if the viewer or communication stops. Joystick priority and
the deployed watchdog behavior are unchanged.

The shared speed slider selects 0.2–2.5 m/s by default, but the viewer clamps
commands to 0.20 m/s and 0.35 rad/s by default. The `SPEED` line shows selected
and viewer-capped linear speeds, respectively; the robot may clamp them further.
Rotation is calculated from wheel
separation as `2 * linear_speed / wheel_separation` and capped in rad/s.
Viewer parameters `directional.min_speed_mps` (0.2),
`directional.max_speed_mps` (2.5), `directional.default_speed_mps` (0.2),
`directional.linear_limit_mps` (0.2), `directional.angular_limit_rps` (0.35),
and `directional.wheel_separation_m` (0.486) configure these values. The robot
node independently enforces `max_viewer_linear_speed` (0.2) and
`max_viewer_angular_speed` (0.35); changing only the viewer caps cannot raise
the robot-side limits. `directional.enable_motion` defaults to true; set it
false for a non-moving diamond. Existing `click.enable_motion` defaults to
false and controls click-to-drive separately. To enable click-to-drive, start
the viewer with `--ros-args -p click.enable_motion:=true` (alongside any
desired speed parameters).

These changes require matching rebuilt `obot_msgs` interfaces and the updated
motion node on the robot and viewer on the laptop. The revised service is not
wire-compatible with the previous `ClickDrive` service. Motion requests have a
short ROS-time deadline, so robot and laptop clocks must be synchronized; late
requests are rejected. The revised interface and motion node were deployed to
pi-bot-2, and the directional controls were exercised on the robot. Test stop
and takeover behavior in a controlled setting before use. `obot-view2cams`
itself requires no changes.

## LiDAR operator overlay

The front view displays a compact `FRONT`, `LEFT`, `RIGHT` panel in feet.
`FRONT` is the median forward centerline distance; the sides are the nearest
returns in the left and right thirds of the forward corridor, in robot
coordinates. Distances are forward from the configured bumper plane, not from
the LiDAR. Green means over 5 ft, yellow at most 5 ft, and red less than 2 ft,
subject to hysteresis. A `NEAR` line appears if a closer central corridor return
would otherwise be hidden by the centerline median. No obstacle positions are
projected onto the fisheye image; the manual steering markers are unchanged.

The previous zero-distance warning could be produced by returns behind the
bumper plane: negative clearances were clamped to zero. A live scan contained
such returns in the center corridor, possibly from the robot itself. Returns
behind that plane are now excluded rather than clamped; returns at or in front
of the plane remain eligible, including very close obstacles. This exclusion
can also hide a real obstacle behind the bumper plane, so it is not a
substitute for identifying the robot's actual self-occlusion.

Approaching distances update immediately; receding distances are smoothed
(25% per valid scan). Warning severity increases immediately and relaxes with a
0.1 m margin. Brief missing returns may retain the last distance for at most
0.25 seconds, marked with `~`; after that they show `--`. Stale scans never
retain a displayed distance. The selected turn is shown as a compact `TURN`
indicator at the upper left.

The overlay is a visual aid, **not** collision avoidance or an emergency stop:
it does not control motion or the watchdog. A missing warning does not mean
the way is clear. `--` means there is no usable return for that region.
`LIDAR COVERAGE UNKNOWN` means at least one 10-degree bin in the forward
180-degree half-plane has no valid range return. This is a coarse indication,
not proof that the sensor has failed: invalid or infinite ranges may occur
normally. Rear returns are not required. Even when the indication is absent,
one valid return per bin does not establish that the corridor is clear.
Stale scans and unavailable transforms are explicitly indicated.
The scanner's blind spots, reflective surfaces, field of view, and obstacles
outside its scan plane can prevent detection.

Startup ROS parameters and defaults:

| Parameter | Default |
|---|---:|
| `lidar.enabled` | `true` |
| `lidar.topic` | `/scan` |
| `lidar.center_half_angle_deg` | `2.0` |
| `lidar.robot_width_m` | `0.5334` |
| `lidar.safety_margin_m` | `0.0762` per side |
| `lidar.warning_distance_m` | `1.524` |
| `lidar.danger_distance_m` | `0.6096` |
| `lidar.stale_timeout_s` | `0.5` |
| `lidar.front_bumper_x_m` | `0.30572` |

The bumper default is 0.260 m for the LiDAR's X position plus 0.04572 m
(1.8 inches) from the LiDAR rotational center to the bumper leading edge.
Set `lidar.front_bumper_x_m` to match the actual mounting if it changes. The
viewer requires the scan frame's TF to `base_link` at the scan timestamp and
synchronized ROS clocks on the robot and laptop. Confirm the scanner mounting
yaw, bumper reference, centerline distance, and left/right warnings with known
stationary objects before using the overlay; also verify stale and TF-error
indications with the robot stationary. Disable it with
`--ros-args -p lidar.enabled:=false` if needed.

ROS 2 software, firmware, utilities, and configuration used on the OBot development and operator laptop (`ros-24`).

This repository contains the laptop-side portion of the SprayBot/OBot system. Robot-side software running on `pi-bot-1` is maintained separately in the `obot-robot` repository.

## Platform

- Ubuntu 24.04
- ROS 2 Jazzy
- `ROS_DOMAIN_ID=5`
- CycloneDDS
- Development/operator computer: `ros-24`

## Repository Layout

- `src/obot` — OBot application nodes and launch files
- `src/obot_bringup` — laptop-side SLAM, localization, and Nav2 bringup
- `src/obot_description` — OBot URDF/Xacro and visualization configuration
- `src/obot_msgs` — OBot-specific ROS messages
- `firmware/cyd_spray_controller` — ESP32/CYD spray-controller and status-display firmware
- `firmware/esp32_spray_controller` — headless ESP32-WROOM-32 spray-valve controller firmware
- `maps` — saved SLAM maps
- `bin` — OBot operator, diagnostic, mapping, and test utilities
- `config/laptop` — portable laptop configuration templates
- `doc` — SprayBot installation documentation
- `systemd` — reserved for applicable service definitions

Generated colcon directories (`build`, `install`, and `log`) are not stored in Git.

## Robot Repository

Software that runs directly on the OBot Raspberry Pi is maintained separately:

https://github.com/wjrfoot/obot-robot

The robot repository contains the robot control bringup, ODrive/CAN integration support, robot-side diagnostics, watchdog, systemd configuration, and external dependency patches.

## CycloneDDS

The laptop uses CycloneDDS for ROS 2 communication.

The active CycloneDDS configuration is machine-specific because it identifies the Wi-Fi interface used for the OBot ROS network. The live configuration is therefore not stored in this repository.

A portable template is provided under:

`config/laptop/cyclonedds_wifi.example.xml`

Configure the appropriate network interface when installing the software on another laptop.

## CYD Spray Controller

The CYD firmware is located under:

`firmware/cyd_spray_controller`

The local firmware configuration containing Wi-Fi credentials is intentionally excluded from Git.

Use:

`firmware/cyd_spray_controller/include/config.example.h`

as the template for creating the local:

`firmware/cyd_spray_controller/include/config.h`

The CYD participates in the ROS 2 system through micro-ROS and provides status display and spray-valve control functionality.

## ESP32 Spray Controller (headless)

Firmware: `firmware/esp32_spray_controller`

Copy `include/config.example.h` to `include/config.h` (excluded from Git) and set, for each
network in `WIFI_NETWORKS`, the SSID, password, and the IP of the laptop running the agent. The
board tries the networks in order until one connects. Build and flash with
`pio run -e esp32-wroom32 -t upload`.

Start the micro-ROS agent on the laptop (UDP port 8888, `ROS_DOMAIN_ID=5`):

    ros2 launch obot micro_ros_agent.launch.py

Optional arguments: `port:=8888 domain_id:=5`.

### PS4 spray buttons

`obot_teleop.launch.py` also starts `joy_spray_node`. Each face button publishes a timed
command to `/spray/valve_command`; a button whose duration is 0 (the default) is disabled:

    ros2 launch obot obot_teleop.launch.py cross_spray_sec:=2.0 circle_spray_sec:=0.5

Arguments: `cross_spray_sec`, `circle_spray_sec`, `triangle_spray_sec`, `square_spray_sec`.

### Session recording (odomsprayrecord)

`odomsprayrecord` logs spray events to a text file per session. L2 opens a new
`odom<YYYYmmdd_HHMMSS>.txt` in `output_dir` and R2 closes it. While open, each press of an
enabled spray button (duration > 0) appends a line `x y yaw spray_sec`, with the pose relative to
the odom pose when the file was opened (metres, degrees).

    ros2 launch obot odomsprayrecord.launch.py output_dir:=~/odom_records cross_spray_sec:=2.0

Arguments: `output_dir`, `odom_topic` (default `/diff_drive_controller/odom`), `joy_topic`, and
the four `*_spray_sec` values (keep them the same as for `obot_teleop.launch.py`).

## Build

    cd ~/Workspaces/obot_ws
    source /opt/ros/jazzy/setup.bash
    colcon build --symlink-install
    source install/setup.bash

## Documentation

The current installation guide is maintained at:

`doc/SprayBot_Installation_Guide.docx`

The filename remains constant. Git commit history and tags provide document version history.

## Git Workflow

Before making changes:

    cd ~/Workspaces/obot_ws
    git pull
    git status

After testing a change:

    git status
    git add <files>
    git commit -m "Describe the change"
    git push
