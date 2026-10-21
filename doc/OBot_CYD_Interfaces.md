# OBot CYD status — interfaces and data flow (phase 1, monitoring only)

Status: **design**. Nothing here is implemented except the message definitions in
`src/obot_msgs/msg/`. Phase 1 never sends CAN commands or touches drivetrain control.

## Data flow

```
 ODrive S1 x2 --CAN(can0)--> [passive SocketCAN receive]
 ip link / vcgencmd / /proc  ----------------------------> obot_status_node  (Pi, new, read-only)
 /scan, cameras, odom, /spray/valve_state, Nav2 lifecycle ->      |
                                                                   |  /obot/status        (1 Hz)
                                                                   |  /obot/diagnostics   (1 Hz)
                                                                   v
                                         micro_ros_agent (laptop, udp4 :8888, domain 5)
                                                                   |
                                                                   v
                                              CYD (obot_cyd_status firmware, read-only)
                                                   |  /obot/cyd/command (reset extrema request)
                                                   +------------------> obot_status_node (Pi)

 Valve ESP32 --/spray/valve_state--> obot_status_node (freshness) and CYD (display)
```

The CYD never publishes to `/spray/valve_command` and has no valve GPIO.

## Topics

| Topic | Type | Dir (from Pi node) | Rate | Notes |
|---|---|---|---|---|
| `/obot/status` | `obot_msgs/msg/ObotStatus` | pub | 1 Hz | page 1 state codes |
| `/obot/diagnostics` | `obot_msgs/msg/ObotDiagnostics` | pub | 1 Hz | page 2 numbers |
| `/obot/cyd/command` | `std_msgs/msg/UInt8` | sub | event | `1` = reset session extrema (CYD asks after on-screen confirmation) |
| `/obot/reset_session_extrema` | `std_srvs/srv/Trigger` | service | event | same action for laptop use; handled on the Pi |
| `/spray/valve_state` | `std_msgs/msg/Bool` | sub | 10 Hz (ESP32) | existing; CYD also subscribes directly |
| `/scan` | `sensor_msgs/msg/LaserScan` | sub | | freshness only |
| `/front_camera/image_raw/compressed`, `/rear_camera/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | sub | | freshness only (use a sparse/BEST_EFFORT subscription) |
| `/diff_drive_controller/odom` | `nav_msgs/msg/Odometry` | sub | 50 Hz | freshness, control mode hint |
| `/cmd_vel_teleop`, `/cmd_vel_nav` | `geometry_msgs/msg/TwistStamped` | sub | | control mode: most recent fresh source |
| Nav2 `lifecycle_manager_navigation` | `nav2_msgs/srv/ManageLifecycleNodes` (`is_active`) / `lifecycle_msgs/srv/GetState` | client | 1 Hz | ACTIVE/INACTIVE/UNKNOWN |

Existing and unchanged: `/obot/watchdog_event` (String, from `obot-watchdog`).
Legacy `/status/can_bus_healthy` and `/status/controller_active` have no publisher today and are
**retired** by this design; the new CYD firmware does not subscribe to them.

Names do not collide with anything found in either repo (`/obot/*` is only used by the watchdog
event topic). QoS: `/obot/status` and `/obot/diagnostics` RELIABLE, depth 1 (micro-ROS best-effort
is an option if WiFi loss causes backlog).

## Freshness and timeouts (initial values, tunable by parameter)

| Item | Source | STALE if no message for |
|---|---|---|
| LIDAR | `/scan` | 2 s |
| Camera 1 / 2 | compressed image | 3 s |
| Odometry | odom | 1 s |
| Valve feedback | `/spray/valve_state` | 1 s (published at 10 Hz) |
| Motor heartbeat | ODrive heartbeat frame (0x001 / 0x021) | 1 s |
| Nav2 / localization lifecycle | service reply | 3 s; failure = UNKNOWN |
| `/obot/status` on the CYD | CYD-side timer | 3 s -> screen shows "LINK STALE" |

"Node exists" is never treated as healthy. Source timestamps use the Pi clock; the CYD displays age
since it last received a message, not an absolute time.

## ODrive data sources (to verify live before Stage 2; do not guess)

Passive listen only on `can0` for the left (node_id 0) and right (node_id 1) S1s:
heartbeat (axis error + axis state), bus voltage/current, temperature, and error frames where the
installed firmware sends them cyclically. The existing `obot-watchdog` requests errors with
`cansend ... #R` (a transmit); the new node must not transmit. Which frames are cyclic and at what
rate must be confirmed against the installed S1 firmware and bus load. Voltage is reported as sampled
extrema only; transient peaks may be missed.

## Persistence (Pi, outside colcon log dir)

`~/obot_logs/`: fault log (JSONL, fsync on fault), all-time peaks, bounded ring buffer of recent
voltage samples with a short pre/post-fault window. Session extrema reset clears only session values;
the fault log and all-time peaks are untouched. Reset is executed on the Pi.

## Not in phase 1

Clear-fault / recovery service (design in the spec only; the button shows DISABLED), any CAN
transmit, ros2_control changes, braking-resistor or E-stop changes.

## Notes / risks

- `obot_msgs` exists in both `obot_ws` and `obot-robot`; the new messages must be copied into the
  robot repo (it builds on the Pi) before the Pi node can use them.
- micro-ROS firmware needs these messages built into its library (`extra_packages`); the fixed-size
  fields were chosen so that no dynamic allocation is needed.
- The CYD touch controller pins are unverified for this unit; confirm on hardware in Stage 4.
- `obot_s1_monitor` named in the spec was not found in either repo.
