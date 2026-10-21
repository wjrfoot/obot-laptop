"""Passive ODrive S1 bus-voltage and error monitor.

Listens on a SocketCAN interface (receive only, never transmits) for the
ODrive CANSimple messages Get_Bus_Voltage_Current (0x17), Heartbeat (0x01) and
Get_Error (0x03), and publishes the current/min/max bus voltage per controller.
Errors are appended to a persistent JSONL fault log with raw numeric codes.

Topics (std_msgs/Float32, so micro-ROS clients such as the CYD need no custom msgs):
  /obot/bus_voltage/<name>/current  latest voltage; NaN if no fresh frame
  /obot/bus_voltage/<name>/max      max observed since node start (or reset)
  /obot/bus_voltage/<name>/min      min observed since node start (or reset)
  /obot/odrive_error/<name>         (UInt32) heartbeat Axis_Error = active_errors | disarm_reason
Service:
  /obot/reset_bus_voltage_extrema (std_srvs/Trigger)

The S1 must have its bus voltage/current message enabled
(odrv0.can.config.bus_vi_msg_rate_ms > 0). Extrema are sampled values only;
short transients between CAN frames can be missed.

Heartbeat Axis_Error is active_errors OR'd with disarm_reason. Get_Error carries
them separately but is not cyclic, and this node never requests it; if another
tool (e.g. the watchdog) triggers it, the separate values are logged too.
Codes are from the ODrive.Error enum in the ODrive docs (S1 firmware 0.6.x);
verify against the installed firmware.

Fault log: ~/obot_logs/odrive_fault_log.jsonl (fsync on every record), one JSON
object per line with raw codes, decoded names, timestamps and bus voltages.
"""

import json
import os

import math
import socket
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, UInt32
from std_srvs.srv import Trigger

CMD_HEARTBEAT = 0x01
CMD_GET_ERROR = 0x03
CMD_GET_BUS_VOLTAGE_CURRENT = 0x17
CAN_FRAME_FMT = '=IB3x8s'
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x7FF

# ODrive.Error bit masks (ODrive docs, fibre_types/com_odriverobotics_ODrive)
ODRIVE_ERRORS = {
    0x00000001: 'INITIALIZING',
    0x00000002: 'SYSTEM_LEVEL',
    0x00000004: 'TIMING_ERROR',
    0x00000008: 'MISSING_ESTIMATE',
    0x00000010: 'BAD_CONFIG',
    0x00000020: 'DRV_FAULT',
    0x00000040: 'MISSING_INPUT',
    0x00000100: 'DC_BUS_OVER_VOLTAGE',
    0x00000200: 'DC_BUS_UNDER_VOLTAGE',
    0x00000400: 'DC_BUS_OVER_CURRENT',
    0x00000800: 'DC_BUS_OVER_REGEN_CURRENT',
    0x00001000: 'CURRENT_LIMIT_VIOLATION',
    0x00002000: 'MOTOR_OVER_TEMP',
    0x00004000: 'INVERTER_OVER_TEMP',
    0x00008000: 'VELOCITY_LIMIT_VIOLATION',
    0x00010000: 'POSITION_LIMIT_VIOLATION',
    0x00020000: 'REQUESTED_CURRENT_TOO_HIGH',
    0x01000000: 'WATCHDOG_TIMER_EXPIRED',
    0x02000000: 'ESTOP_REQUESTED',
    0x04000000: 'SPINOUT_DETECTED',
    0x08000000: 'BRAKE_RESISTOR_DISARMED',
    0x10000000: 'THERMISTOR_DISCONNECTED',
    0x40000000: 'CALIBRATION_ERROR',
}


def decode_errors(mask):
    names = [name for bit, name in ODRIVE_ERRORS.items() if mask & bit]
    unknown = mask & ~sum(ODRIVE_ERRORS)
    if unknown:
        names.append(f'UNKNOWN_0x{unknown:08X}')
    return names


class OdriveBusVoltageNode(Node):
    def __init__(self):
        super().__init__('odrive_bus_voltage_node')

        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('node_ids', [0, 1])
        self.declare_parameter('names', ['left', 'right'])
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('stale_timeout_s', 1.0)
        self.declare_parameter('fault_log_path', '~/obot_logs/odrive_fault_log.jsonl')

        iface = self.get_parameter('can_interface').value
        node_ids = list(self.get_parameter('node_ids').value)
        names = list(self.get_parameter('names').value)
        if len(node_ids) != len(names):
            raise ValueError('node_ids and names must have the same length')
        self.stale_timeout = float(self.get_parameter('stale_timeout_s').value)

        self.log_path = os.path.expanduser(
            self.get_parameter('fault_log_path').value)
        os.makedirs(os.path.dirname(self.log_path), exist_ok=True)
        self.names = dict(zip(node_ids, names))

        self.lock = threading.Lock()
        self.state = {}
        self.pubs = {}
        for node_id, name in zip(node_ids, names):
            self.state[node_id] = {
                'v': math.nan, 'min': math.nan, 'max': math.nan, 'stamp': 0.0,
                'axis_error': 0, 'axis_state': 0, 'seen_hb': False,
                'ge': None,
            }
            self.pubs[node_id] = {
                key: self.create_publisher(
                    Float32, f'/obot/bus_voltage/{name}/{key}', 10)
                for key in ('current', 'min', 'max')
            }
            self.pubs[node_id]['error'] = self.create_publisher(
                UInt32, f'/obot/odrive_error/{name}', 10)

        self.sock = socket.socket(
            socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((iface,))
        self.sock.settimeout(0.5)
        self.running = True
        self.thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.thread.start()

        self.create_service(
            Trigger, '/obot/reset_bus_voltage_extrema', self._reset)
        period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(period, self._publish)
        self.get_logger().info(
            f'Listening passively on {iface} for ODrive node_ids {node_ids}')

    def _rx_loop(self):
        while self.running:
            try:
                frame = self.sock.recv(CAN_FRAME_SIZE)
            except socket.timeout:
                continue
            except OSError as exc:
                if self.running:
                    self.get_logger().error(f'CAN receive failed: {exc}')
                return
            if len(frame) < CAN_FRAME_SIZE:
                continue
            can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, frame)
            if can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG):
                continue
            can_id &= CAN_SFF_MASK
            node_id, cmd = can_id >> 5, can_id & 0x1F
            entry = self.state.get(node_id)
            if entry is None:
                continue
            if cmd == CMD_GET_BUS_VOLTAGE_CURRENT and dlc >= 4:
                self._on_voltage(entry, data)
            elif cmd == CMD_HEARTBEAT and dlc >= 5:
                self._on_heartbeat(node_id, entry, data)
            elif cmd == CMD_GET_ERROR and dlc >= 8:
                self._on_get_error(node_id, entry, data)

    def _on_voltage(self, entry, data):
        voltage = struct.unpack('<f', data[:4])[0]
        if not math.isfinite(voltage):
            return
        with self.lock:
            entry['v'] = voltage
            entry['stamp'] = time.monotonic()
            if math.isnan(entry['max']) or voltage > entry['max']:
                entry['max'] = voltage
            if math.isnan(entry['min']) or voltage < entry['min']:
                entry['min'] = voltage

    def _on_heartbeat(self, node_id, entry, data):
        axis_error, axis_state = struct.unpack('<IB', data[:5])
        with self.lock:
            previous = entry['axis_error']
            first = not entry['seen_hb']
            entry['axis_error'] = axis_error
            entry['axis_state'] = axis_state
            entry['seen_hb'] = True
        # Log only transitions that add error bits (and a boot-time nonzero)
        if axis_error and (first or axis_error & ~previous):
            self._log_fault(node_id, entry, 'heartbeat',
                            axis_error=axis_error, axis_state=axis_state)

    def _on_get_error(self, node_id, entry, data):
        active, disarm = struct.unpack('<II', data[:8])
        with self.lock:
            changed = entry['ge'] != (active, disarm)
            entry['ge'] = (active, disarm)
        if changed and (active or disarm):
            self._log_fault(node_id, entry, 'get_error',
                            active_errors=active, disarm_reason=disarm)

    def _log_fault(self, node_id, entry, source, **codes):
        with self.lock:
            record = {
                'time_unix': time.time(),
                'time_iso': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
                'controller': self.names[node_id],
                'node_id': node_id,
                'source': source,
                'bus_voltage_v': None if math.isnan(entry['v']) else entry['v'],
                'bus_voltage_age_s': (
                    time.monotonic() - entry['stamp'] if entry['stamp'] else None),
                'bus_voltage_min_v': None if math.isnan(entry['min']) else entry['min'],
                'bus_voltage_max_v': None if math.isnan(entry['max']) else entry['max'],
            }
        record.update(codes)
        for key in ('axis_error', 'active_errors', 'disarm_reason'):
            if key in codes:
                record[key + '_hex'] = f'0x{codes[key]:08X}'
                record[key + '_names'] = decode_errors(codes[key])
        try:
            with open(self.log_path, 'a') as fh:
                fh.write(json.dumps(record) + '\n')
                fh.flush()
                os.fsync(fh.fileno())
        except OSError as exc:
            self.get_logger().error(f'Fault log write failed: {exc}')
        decoded = {k: v for k, v in record.items() if k.endswith('_names')}
        self.get_logger().warn(f"{record['controller']} error: {decoded}")

    def _publish(self):
        now = time.monotonic()
        with self.lock:
            snapshot = {k: dict(v) for k, v in self.state.items()}
        for node_id, entry in snapshot.items():
            fresh = now - entry['stamp'] <= self.stale_timeout
            values = {
                'current': entry['v'] if fresh else math.nan,
                'min': entry['min'],
                'max': entry['max'],
            }
            for key, value in values.items():
                self.pubs[node_id][key].publish(Float32(data=float(value)))
            self.pubs[node_id]['error'].publish(UInt32(data=entry['axis_error']))

    def _reset(self, request, response):
        with self.lock:
            for entry in self.state.values():
                entry['min'] = entry['max'] = entry['v']
        response.success = True
        response.message = 'Bus voltage extrema reset'
        return response

    def close(self):
        self.running = False
        self.thread.join(timeout=1.0)
        self.sock.close()


def main():
    rclpy.init()
    node = OdriveBusVoltageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
