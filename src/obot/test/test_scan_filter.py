"""Integration tests for the installed scan-filter launch and configuration."""

import math
import os
import subprocess
import time

import pytest

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message, serialize_message

from sensor_msgs.msg import LaserScan


@pytest.mark.parametrize('angle_min', [-math.pi, 0.0])
def test_scan_filter_preserves_geometry_and_masks_rear(monkeypatch, angle_min):
    """Run the actual filter chain with both common angular conventions."""
    domain = 180 + os.getpid() % 30
    monkeypatch.setenv('ROS_DOMAIN_ID', str(domain))
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    monkeypatch.delenv('CYCLONEDDS_URI', raising=False)
    env = os.environ.copy()
    process = subprocess.Popen(
        ['ros2', 'launch', 'obot', 'obot_scan_filter.launch.py'],
        env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )
    context = Context()
    node = None
    executor = None
    try:
        rclpy.init(context=context, domain_id=domain)
        node = Node('scan_filter_test', context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        received = []
        node.create_subscription(
            LaserScan, '/scan_filtered', received.append,
            qos_profile_sensor_data)
        publisher = node.create_publisher(LaserScan, '/scan', 10)

        scan = LaserScan()
        scan.header.frame_id = 'laser'
        scan.header.stamp.sec = 123
        scan.header.stamp.nanosec = 456
        scan.angle_min = angle_min
        scan.angle_increment = 2.0 * math.pi / 360
        scan.angle_max = scan.angle_min + 360 * scan.angle_increment
        scan.time_increment = 0.0001
        scan.scan_time = 0.1
        scan.range_min = 0.1
        scan.range_max = 12.0
        scan.ranges = [2.0] * 361
        scan.intensities = [7.0] * 361
        scan = deserialize_message(serialize_message(scan), LaserScan)

        deadline = time.monotonic() + 15.0
        while not received and time.monotonic() < deadline:
            if process.poll() is not None:
                pytest.fail(f'filter exited: {process.stdout.read()}')
            publisher.publish(scan)
            executor.spin_once(timeout_sec=0.1)
        assert received, 'filter did not produce a scan'
        result = received[0]
        for field in (
            'header', 'angle_min', 'angle_max', 'angle_increment',
            'time_increment', 'scan_time', 'range_min', 'range_max',
        ):
            assert getattr(result, field) == getattr(scan, field)
        assert len(result.ranges) == len(scan.ranges)
        assert len(result.intensities) == len(scan.intensities)
        masked = 0
        for i, (distance, intensity) in enumerate(
            zip(result.ranges, result.intensities)
        ):
            angle = scan.angle_min + i * scan.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if 0.436332 < angle < 2.792527:
                assert math.isnan(distance)
                assert intensity == 0.0
                masked += 1
            else:
                assert distance == 2.0
                assert intensity == 7.0
        assert masked == 136
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if context.ok():
            rclpy.shutdown(context=context)
        process.terminate()
        try:
            process.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
            process.communicate()
            pytest.fail('filter launch did not terminate')
