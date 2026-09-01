# CYD spray controller firmware

PlatformIO firmware for an ESP32-2432S028R "Cheap Yellow Display" (CYD) board
used as:

1. A status display for the robot: CAN bus health and controller-active state.
2. A GPIO-driven spray valve solenoid controller (via relay/MOSFET).

It joins ROS 2 as a micro-ROS node over WiFi/UDP, talking to a micro-ROS agent
running on the PC/robot computer.

## Topics

| Topic                       | Type              | Direction | Purpose                                   |
|------------------------------|-------------------|-----------|--------------------------------------------|
| `/status/can_bus_healthy`     | `std_msgs/Bool`   | sub       | CAN bus status tile                        |
| `/status/controller_active`   | `std_msgs/Bool`   | sub       | Controller-active status tile              |
| `/spray/valve_command`        | `std_msgs/Float32`| sub       | Duration (s) to hold the valve open; `<=0` closes it immediately |
| `/spray/valve_state`          | `std_msgs/Bool`   | pub       | Current valve open/closed feedback         |

`obot`'s `spray_mission_console` publishes `/spray/valve_command` automatically
whenever a waypoint's spray is triggered, so the console and this board work
together out of the box (as long as the topic names match).

## Hardware notes

- Target board: ESP32-2432S028R (ILI9341 240x320 SPI display, resistive touch
  not used yet). Pin mapping is set via `build_flags` in `platformio.ini`
  (TFT_eSPI "no-config" build) — adjust if your CYD variant wires the panel
  differently, or if colors look inverted (`-DTFT_INVERSION_ON=1`).
- The valve relay/MOSFET signal is driven from `VALVE_RELAY_PIN` (default
  GPIO 22) in `include/config.h`. Set `VALVE_ACTIVE_HIGH` to `0` for
  active-low relay modules.

## Setup

1. Edit `include/config.h`:
   - `WIFI_SSID` / `WIFI_PASSWORD`
   - `AGENT_IP` / `AGENT_PORT` — the machine running `micro_ros_agent`
   - `VALVE_RELAY_PIN` / `VALVE_ACTIVE_HIGH` if your wiring differs
2. Run the micro-ROS agent (matching the `board_microros_distro` in
   `platformio.ini`, currently `jazzy`):
   ```
   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
   ```
3. Build and flash with PlatformIO:
   ```
   pio run -t upload
   pio device monitor
   ```

## Future work

Touch input (XPT2046) is not wired up yet; it's reserved for manual on-screen
control of the valve and other future inputs.
