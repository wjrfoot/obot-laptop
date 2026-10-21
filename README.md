# OBot Laptop

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
- `firmware/obot_cyd_status` — CYD status/diagnostics display (scaffold; see `doc/OBot_CYD_Interfaces.md`)
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
