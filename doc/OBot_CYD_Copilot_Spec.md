# OBot CYD status and diagnostics — Copilot implementation brief

## Project context
- ROS 2 Jazzy on Ubuntu 24.04, `ROS_DOMAIN_ID=5`, CycloneDDS.
- Robot Pi 5 hosts OBot bringup and two ODrive S1 controllers over CAN.
- Existing `obot_s1_monitor` reports bus voltage, temperatures, heartbeat, and errors.
- CYD is the existing 2.8-inch ESP32 240×320 touchscreen, previously used with micro-ROS.
- A **separate ESP32** controls the spray valve. CYD must **not** actuate the valve.
- Reuse current workspaces, code conventions, launch files and firmware framework. Do not introduce KSUbot-specific code.

## First task: inspect, report, then implement incrementally
1. Inspect the workspace and identify the current CYD firmware, display/touch driver, micro-ROS transport, `obot_s1_monitor` topics/types, CAN hardware interface, bringup launch files, and existing valve-state topic (if any).
2. Report what exists, missing topics, and any naming conflicts. Do not guess topic types, pin assignments, or ODrive CAN IDs.
3. Propose a minimal change plan and list files to modify before writing code.
4. Implement monitoring and UI only in the first pass; **do not send CAN control commands or change drivetrain behavior**.

## UI: 240×320 portrait touchscreen
Two pages with touch navigation and a visible last-update/connection indicator. Use legible fonts and compact rows; do not rely on color alone. Show UNKNOWN/STALE when telemetry expires; never equate 'node exists' with 'healthy'.

### Page 1 — STATUS
- ROS 2 telemetry link / status aggregator
- Nav2 lifecycle status (ACTIVE / INACTIVE / UNKNOWN)
- SLAM or localization mode and health (not both required)
- LIDAR fresh scan data
- Camera 1 and Camera 2 fresh image data (discover actual topics)
- Motor 1 and Motor 2 CAN heartbeat + axis state + error/ready indication
- CAN interface health, Wi-Fi link health
- Valve state from **separate ESP32**: ON / OFF / UNKNOWN, with fresh feedback where available
- Control mode (manual / Nav2 / unknown), only if determinable from available sources
- Fault count or alert banner and navigation button to diagnostics

### Page 2 — DIAGNOSTICS
- Pi CPU temperature in °C
- Pi current throttling and historical throttling flags; undervoltage flags
- Pi CPU usage if inexpensive to collect
- Left/right ODrive bus voltage (current, observed session max, observed session min)
- Left/right controller temperature where available
- CAN error state/counters where available
- Active errors, disarm reason, last fault timestamp, fault count
- Touch button to reset **session extrema only**, requiring confirmation; historical fault log must remain intact
- Fault-log detail view may be a later enhancement; avoid cramming unreadable text
- Optional future manual clear-fault button must be DISABLED/NOT IMPLEMENTED in phase 1

## Pi-side monitoring
- Prefer extending existing status monitor or adding `obot_status_node` with minimal dependencies.
- Reuse existing ODrive data, do not open a second competing CAN control path.
- Pi temperature: `/sys/class/thermal/thermal_zone0/temp` or `vcgencmd measure_temp`.
- Throttling: `vcgencmd get_throttled`; decode current bits 0–3 and historical bits 16–19; handle unavailable command.
- Actual Pi 5V supply voltage is **not** available from `vcgencmd`; show undervoltage status, not a fabricated numeric voltage.
- Monitor topic freshness and lifecycle states, not merely ROS node presence.
- Use compact micro-ROS-friendly messages, preferably a small set of clearly defined typed status messages; inspect available interfaces before creating custom ones.
- Define timeout thresholds and publish rates explicitly. Target CYD display updates ~1–2 Hz; Pi may collect faster.
- Publish source timestamp and age/validity indicators, accounting for clock synchronization.

## Voltage peaks and fault logging
- Track left/right bus voltage current, minimum and maximum *observed* values independently.
- Preserve session extrema across CYD disconnects. Persist historical faults and all-time peaks outside colcon-generated `~/obot_ws/log` (e.g. `~/obot_logs/`).
- Record controller, timestamp, active error/disarm reason, bus voltage at fault, observed pre-fault max/min, and recovery state.
- Keep a bounded ring buffer of recent voltage samples and persist a short pre/post-fault window.
- Confirm actual voltage reporting rate supported by current ODrive firmware and CAN bus load before changing rates. High-rate CAN samples still may miss short electrical transients; never claim true instantaneous peak detection.
- Use bounded writes / appropriate flushing so power loss does not silently erase the only fault record.
- A reset-extrema request must be handled by the Pi, not only by resetting CYD screen variables.

## Safety and recovery (design only; no implementation in phase 1)
- Clearing ODrive faults may be possible without reboot or restarting bringup, but must be coordinated with the existing `ros2_control` hardware interface.
- Define a future manual recovery service with prerequisites: motors commanded to zero, motion commands inhibited, E-stop verified via actual feedback if available, underlying condition resolved, both controllers healthy, fresh commands required after recovery.
- Do not automatically resume previous Nav2/teleop motion after recovery.
- Overvoltage, overcurrent, thermal, encoder, and gate-driver faults should not auto-reset by default.
- Add rate limits, attempt limits, logs, and operator acknowledgment before any future automatic recovery.

## Deliverables and acceptance checks
1. Inventory of existing relevant files/topics, and implementation plan.
2. Pi status node and tests (including stale/missing topics, missing vcgencmd, error state, peak persistence).
3. CYD firmware for both pages and touch navigation; valve is read-only.
4. Launch/service integration, build instructions, deployment instructions, and rollback procedure.
5. A test procedure that uses synthetic ROS data first, then live hardware with motors disabled.
6. No changes to motor control, CAN command path, braking-resistor settings, or E-stop behavior in phase 1.

## First response requested from Copilot
Before editing any files, list the discovered code paths, current topics and message types, missing data sources, and a concrete staged implementation plan. Ask for approval before making drivetrain-related changes.
