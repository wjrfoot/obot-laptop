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
