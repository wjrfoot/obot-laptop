# obot_cyd_status (CYD status and diagnostics display)

**Status: scaffold only. No firmware source yet.** Stage 4 of the CYD monitoring project.

Target: ESP32-2432S028R "Cheap Yellow Display" (240x320 portrait, ILI9341, touch). Two pages,
STATUS and DIAGNOSTICS, fed by `/obot/status` and `/obot/diagnostics` over micro-ROS.
The CYD is read-only: it has no valve output and never publishes `/spray/valve_command`.

- `platformio.ini` — copied from the original CYD project (TFT_eSPI pins, micro-ROS/jazzy, partitions).
- `include/config.example.h` — WiFi networks, topics, timeouts. Copy to `include/config.h` (git-ignored).
- Interfaces and data flow: [`doc/OBot_CYD_Interfaces.md`](../../doc/OBot_CYD_Interfaces.md)
- Messages: `src/obot_msgs/msg/ObotStatus.msg`, `ObotDiagnostics.msg`

History: the original CYD display + valve firmware is tagged `pre-cyd-status-original-cyd`.
The headless valve variant lives in `firmware/esp32_spray_controller`.
