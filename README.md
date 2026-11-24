# OBot Laptop

## Camera telemetry dashboard

Build `obot` and run `obot-view2cams` (or `bin/obot-view2cams` from this
repository). The combined camera window starts maximized; it can
still be restored or resized. The dashboard appears below the rear view. Click
`POSITION` to choose Odom or Map; unavailable or stale transforms are
shown as unavailable, never substituted with another frame. Click
`RESET ORIGIN` to zero the selected frame **for display only**. Odom and Map
keep independent origins until the viewer exits. This does not reset ROS
odometry or modify TF. If TF is unavailable or stale, reset is refused.

The motion command comes from `/diff_drive_controller/cmd_vel`; actual
motion is the wheel-odometry estimate from `/diff_drive_controller/odom`,
not a measurement of ground speed. Wheel RPM comes from `/joint_states`,
matched by joint name. For operator clarity, the left wheel's displayed sign
is inverted; the right wheel retains its joint sign. The `/joint_states`
data itself is unchanged. DC bus voltages are
motor-controller measurements, not battery-voltage measurements. The panel
does not control motors and does not replace collision avoidance or an E-stop.

The OBot-2 watchdog must run the version publishing
`/obot2/watchdog/bus_voltage/left`, `/obot2/watchdog/bus_voltage/right`
(`std_msgs/Float32`) and `/obot2/watchdog/health` (`std_msgs/String`).
Until that version is installed and running on the robot, those fields
show `--` and `No Data`. Its health is `NORMAL`, `RECOVERING`, `FAULT` or
`NO_DATA`; normal requires fresh CAN, heartbeat and error telemetry for both
controllers. Values disappear after the configured freshness timeout.
Existing passive-monitor topics can instead be selected with the voltage
topic parameters, but must be configured for the correct robot node IDs.

ROS parameters can be supplied with `--ros-args -p`, for example:

    ros2 run obot view2cams --ros-args -p telemetry.stale_timeout_s:=3.0 -p telemetry.robot_frame:=base_link

Parameters include `telemetry.enabled` (default true), `telemetry.refresh_hz`
(5.0), `telemetry.stale_timeout_s` (2.0), `telemetry.command_topic`,
`telemetry.odom_topic`, `telemetry.joint_states_topic`, `telemetry.odom_frame`
(`odom`), `telemetry.map_frame` (`map`), `telemetry.robot_frame` (`base_link`),
`telemetry.default_position_source` (`odom`), `telemetry.left_joint`,
`telemetry.right_joint`, `telemetry.left_voltage_topic`,
`telemetry.right_voltage_topic`, and `telemetry.health_topic`. Joint defaults
are `left_drive_wheel_joint` and `right_drive_wheel_joint`.

To check the dashboard without moving the robot, start the viewer, confirm
both camera feeds and guidance markers, compare the telemetry to the ROS
topics, select Map and verify unavailable transforms are reported, then reset
the Odom display origin and check that ROS odometry remains unchanged. Stop a
telemetry publisher to confirm its readings become stale. Test physical motion
only with appropriate safety precautions.

### OBot-2 watchdog routing discrepancy

On the inspected `pi-bot-2` deployment, `obot2_watchdog` published health
telemetry but no `/watchdog_lock` or `/cmd_vel_watchdog` topic. The running
`twist_mux` had joystick, motion-test, and navigation inputs but no configured
watchdog lock. This differs from the fail-closed watchdog-lock configuration
present in the robot repository. Investigate and reconcile the deployed
configuration separately before relying on the watchdog to inhibit motion.
The watchdog's motor fault monitoring and recovery should not be confused with
a verified ROS velocity-command lock.

### Camera motion controls

Point-and-click/point-and-turn is the preferred manual operating interface.
Operational testing found tank steering less satisfactory than anticipated;
the PS4 remains available as an optional alternative.

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

On `ros-24`, both the repository and `~/bin/obot-view2cams` launchers enable
click-to-drive, set the viewer's linear limit and slider maximum to 1.0 m/s,
and set the center marker thickness to 4 pixels and height fraction to 0.4.
After updating the repository launcher, install it with:

    install -m 755 ~/Workspaces/obot_ws/bin/obot-view2cams ~/bin/obot-view2cams

The Pi's separate `max_viewer_linear_speed` limit must also permit the
requested speed. Selecting a target does not move the robot: select TURN to
rotate, then enter a distance and select GO. CANCEL stops or cancels the
operation. The directional diamond takes priority over click-to-drive.

The front view's SPRAY panel uses square, circle and triangle icons for the
corresponding server-configured timed sprays; the X icon cancels a spray
started by this viewer. The displayed valve state is the robot's commanded
state, not physical position feedback. Stale state shows UNKNOWN. The server
accepts one spray at a time; it can reject a request while another client owns
an active spray. The viewer cannot cancel a goal started by the PS4 client.
Build the robot workspace's `obot_valve_interfaces` package before starting
the viewer. The launcher sources only that package's environment hook, then
the laptop workspace and its `obot_msgs` and `obot` hooks. If the interface is
unavailable, camera and motion functions remain available but the spray
panel reports DEPENDENCY MISSING.

Both workspaces contain a Python distribution named `obot`. Sourcing the full
robot workspace as an underlay while building the laptop workspace can record
that underlay in the generated laptop setup; if the robot distribution is
found first, the viewer's entry-point loader fails with `StopIteration`. Build
the laptop workspace from a fresh shell with only ROS sourced, as in the
Build section below. The launcher uses the laptop `local_setup.bash` to avoid
following a stale generated underlay chain and explicitly prioritizes the
laptop packages even when the calling shell already sourced the robot.

These changes require matching rebuilt `obot_msgs` interfaces and the updated
motion node on the robot and viewer on the laptop. The revised service is not
wire-compatible with the previous `ClickDrive` service. Motion requests have a
short ROS-time deadline, so robot and laptop clocks must be synchronized; late
requests are rejected. The revised interface and motion node were deployed to
pi-bot-2. On 2026-11-23, live operational testing on OBot-2 confirmed that
the viewer's valve controls, directional diamond, speed slider, TURN, CANCEL
and GO worked, and motion used the selected slider speed. This is the
operator's reported result for that test, not a certification of all operating
conditions.

### LiDAR operator overlay

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

## Odometry path in RViz

The robot-side `obot_path_recorder` starts with `obot_base.launch.py` and
samples `/diff_drive_controller/odom` (`nav_msgs/msg/Odometry`). It publishes
the bounded trajectory on `/obot/path` (`nav_msgs/msg/Path`) at 2 Hz with
reliable, transient-local QoS. Its defaults are 0.05 m, 0.15 rad, and 10,000
poses; configure `minimum_distance_m`, `minimum_angle_rad`, `max_points`,
`publish_rate_hz`, `odom_topic`, and `path_topic` as ROS parameters on the
recorder. The startup defaults are in
`obot_bringup/config/path_recorder.yaml`; supply an alternative file with
`ros2 launch obot_bringup obot_base.launch.py path_recorder_config:=/absolute/path/to/parameters.yaml`.
These parameters are read at startup. Invalid values prevent the recorder
from starting.

After building and deploying the robot-side changes, build `obot_description`
on the laptop and open the separate RViz profile:

    ros2 run rviz2 rviz2 -d "$(ros2 pkg prefix obot_description)/share/obot_description/rviz/obot_path.rviz"

The profile keeps the Grid and uses the `odom` fixed frame and a yellow Path
display. To configure an existing RViz setup instead, add a Path display for
`/obot/path`, set reliability to Reliable and durability to Transient Local,
and set Fixed Frame to the odometry frame. Reduce any Odometry display's
history to one pose if arrows accumulate.

The viewer's **CLEAR PATH** button calls `/obot/clear_path`
(`std_srvs/srv/Trigger`) asynchronously. To clear manually:

    clear
    ros2 service call /obot/clear_path std_srvs/srv/Trigger '{}'

Clearing affects only recorded visualization history, not odometry, TF or the
dashboard's independent **RESET ORIGIN**. The next valid odometry pose starts
a new path. A changed odometry frame or a backward timestamp jump greater than
one second clears the prior path automatically.

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

### Optional PS4 joystick operation

On the inspected `ros-24` installation, `joy_ps4.service` is enabled and
starts `joy_node` automatically at boot with 20 Hz autorepeat. It is
independent of the viewer and steering launches. Those launches are started
manually; neither starts another joystick driver. The service is machine-local
and is not supplied in this repository. On another machine, install the ROS 2
Jazzy `joy` package, configure the ROS domain and CycloneDDS environment, and
install an appropriate joystick service or start one `joy_node` manually. Do
not run two drivers for the same controller. Check the service with:

    systemctl status joy_ps4.service
    systemctl is-enabled joy_ps4.service

To pair a PS4 DualShock controller, enable Bluetooth on the laptop, hold SHARE
and PS until the light flashes, then select **Wireless Controller** in the
desktop Bluetooth settings. Alternatively connect it by USB. Confirm that the
driver publishes `/joy` and the selected input device exists before operating.
Pairing or a running `joy_node` alone does not start a steering mixer.

For optional tank steering, source both workspaces and start the launch
manually:

    cd ~/Workspaces/obot_ws
    source /opt/ros/jazzy/setup.bash
    source ~/Workspaces/obot-robot/install/setup.bash
    source install/setup.bash
    ros2 launch obot obot_teleop.launch.py

It starts the tank mixer and a PS4 spray action client, not `joy_node` or a
separate rumble node. The action client provides valve and watchdog rumble.
Stop the launch with Ctrl+C. Circle, Square and Triangle spray durations
default to zero (disabled); override `circle_spray_sec`, `square_spray_sec`,
or `triangle_spray_sec` to enable them. Cross cancels. Set durations only as
appropriate for the installed valve server.

For optional single-stick steering, start
`ros2 launch obot obot_single_stick.launch.py`; it starts the mixer only.
For spray controls with that mode, separately run
`ros2 launch obot_valve_controller ps4_valve_client.launch.py`. Its YAML
configuration and defaults differ from the tank launch; check the installed
configuration before use.

F1 (button 4) must remain held for motion. Both tank sticks must be in the
existing deadband before motion can be enabled; if F1 is pressed with a stick
displaced, the mixer publishes zero commands until both sticks are neutral.
It publishes another zero command when neutral is reached, and motion may begin
on a subsequent joystick message. Releasing F1 disarms teleop, publishes a stop
if teleop owned control, then goes silent so `twist_mux` can hand control back
to Nav2. Re-pressing F1 while either stick remains displaced will not restart
motion. After joystick timeout, motion requires a fresh F1 release/press and
neutral sticks. The physical E-stop remains the emergency-stop mechanism;
software interlocks are not a substitute for it.

The mixer keeps `require_enable` for direct node configuration. When true, a
valid enable button or axis is mandatory; missing or invalid controls fail
closed. When false, the deadman gate is disabled. The OBot-2 launch fixes
`require_enable:=true` and F1 button 4; do not disable or remap this interlock
for normal OBot-2 operation.

The existing `obot-start-single` and `obot-start-skid` scripts start a mixer,
an additional spray client and a viewer; the current tank launch already starts
its own spray client, so the skid variant can start duplicate clients. Avoid
running it until that existing integration is reviewed. Script copies under
`~/bin` may differ from the repository versions; check them before use.

### PS4 spray buttons

The tank launch's `ps4_valve_client` sends requests to `/obot/spray` and
provides rumble from valve-state and watchdog feedback. Rumble requires a
controller and driver that support force feedback. The existing
`joy_spray_node` and `spray_rumble_node` remain available separately for
installations using their legacy `/spray/*` topics; they are not started by
the current tank launch. Do not start multiple spray clients for one joystick.

### Session recording (odom_spray_record)

`odom_spray_record` logs spray events to a text file per session. L2 opens a new
`odom<YYYYmmdd_HHMMSS>.txt` in `output_dir` and R2 closes it. While open, each press of an
enabled spray button (duration > 0) appends a line `x y yaw spray_sec`, with the pose relative to
the odom pose when the file was opened (feet by default, degrees). Set `distance_unit:=meters`
to report x/y in metres instead.

    ros2 launch obot odom_spray_record.launch.py output_dir:=~/odom_records cross_spray_sec:=2.0

Arguments: `output_dir`, `odom_topic` (default `/diff_drive_controller/odom`), `joy_topic`,
`distance_unit` (`feet` by default, or `meters`), and the four `*_spray_sec` values.
Configure the recorder's durations to match the spray setup in use.

## Build

On the laptop, build in a clean environment with only ROS sourced. Do **not**
source the robot workspace or its valve-interface package hook before running
colcon: colcon interprets the interface's isolated `AMENT_PREFIX_PATH` entry
as its entire parent workspace and records that workspace as an underlay.
The viewer needs the valve interface at runtime, not to build `obot`; the
launcher sources the interface hook after the build.

    env -i HOME="$HOME" USER="$USER" PATH=/usr/local/bin:/usr/bin:/bin \
      bash --noprofile --norc -c '
        set -e
        source /opt/ros/jazzy/setup.bash
        cd "$HOME/Workspaces/obot_ws"
        colcon build --symlink-install
      '

Run this command again for subsequent builds. It also regenerates the
workspace setup chain without a previously recorded robot underlay. Start the
viewer using `obot-view2cams` in the normal operator environment; the clean
build command does not change that environment's ROS network settings.

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
