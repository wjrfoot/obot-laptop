# ESP32 spray controller firmware

PlatformIO firmware for a generic ESP32-WROOM-32 development board used as a
headless spray valve controller. It provides:

1. A GPIO-driven spray valve controller (via a relay/MOSFET module).
2. Status logging over USB serial.

It joins ROS 2 as a micro-ROS node over WiFi/UDP, talking to a micro-ROS agent
running on the PC/robot computer.

## Topics

| Topic                       | Type              | Direction | Purpose                                   |
|------------------------------|-------------------|-----------|--------------------------------------------|
| `/status/can_bus_healthy`     | `std_msgs/Bool`   | sub       | CAN bus status logging                     |
| `/status/controller_active`   | `std_msgs/Bool`   | sub       | Controller-active status logging           |
| `/spray/valve_command`        | `std_msgs/Float32`| sub       | Duration (s) to hold the valve open; `<=0` closes it immediately |
| `/spray/valve_state`          | `std_msgs/Bool`   | pub       | Current valve open/closed feedback         |

`obot`'s `spray_mission_console` publishes `/spray/valve_command` automatically
whenever a waypoint's spray is triggered, so the console and this board work
together out of the box (as long as the topic names match).

## Hardware notes

- Target board: ESP32-WROOM-32 development board (`esp32dev` PlatformIO target).
- The relay/MOSFET control signal is driven from `VALVE_RELAY_PIN` (default
   GPIO 27) in `include/config.h`. Set `VALVE_ACTIVE_HIGH` to `0` for
   active-low modules. GPIO output is 3.3 V; verify the module recognizes this
   signal level. Do not drive a valve directly from an ESP32 GPIO.
- Connect the controller ground to the relay module control ground unless its
   input is electrically isolated. Power the module from a supply at the
   module's rated voltage; verify the carrier-board VIN voltage before using it.
- The reported `/spray/valve_state` is the commanded GPIO state, not physical
   valve-position feedback.

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
   pio run -e esp32-wroom32 -t upload
   pio device monitor
   ```

The firmware closes the valve when its requested duration expires or the
micro-ROS agent disconnects. If the firmware encounters a fatal micro-ROS
error, it also drives the output to the configured closed state.
