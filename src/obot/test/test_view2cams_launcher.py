"""Verify viewer launcher environment and arguments in installed workspaces."""

import json
import subprocess
from pathlib import Path

import pytest


WORKSPACE = Path(__file__).resolve().parents[3]
HOME = Path.home()
ROBOT = HOME / 'Workspaces/obot-robot'
LAUNCHER = WORKSPACE / 'bin/obot-view2cams'
REQUIRED = (
    WORKSPACE / 'install/obot/share/obot/package.bash',
    WORKSPACE / 'install/obot_msgs/share/obot_msgs/package.bash',
    ROBOT / 'install/obot_valve_interfaces/share/'
    'obot_valve_interfaces/package.bash',
    ROBOT / 'build/obot',
    Path('/opt/ros/jazzy/setup.bash'),
)


@pytest.mark.parametrize('robot_first', [False, True])
def test_launcher_resolves_laptop_and_valve_interface(tmp_path, robot_first):
    """Keep the laptop entry point and valve interface in both environments."""
    if WORKSPACE != HOME / 'Workspaces/obot_ws' or not all(
        p.exists() for p in REQUIRED
    ):
        pytest.skip('requires the built ros-24 laptop and robot workspaces')

    probe = tmp_path / 'probe.py'
    probe.write_text(
        'import importlib.metadata, json, sys\n'
        'from ament_index_python.packages import get_package_prefix\n'
        'from obot_valve_interfaces.action import SprayValve\n'
        'from obot_msgs.srv import ClickDrive\n'
        'dist = importlib.metadata.distribution("obot")\n'
        'print(json.dumps({"args": sys.argv[1:], '
        '"entrypoints": [e.name for e in dist.entry_points], '
        '"prefix": get_package_prefix("obot"), '
        '"valve": SprayValve.__module__, "click": ClickDrive.__module__}))\n'
    )
    bash_env = tmp_path / 'bash_env'
    bash_env.write_text('exec() { /usr/bin/python3 "$PROBE" "$@"; }\n')
    env = {
        'HOME': str(HOME),
        'PATH': '/usr/bin:/bin',
        'BASH_ENV': str(bash_env),
        'PROBE': str(probe),
    }
    if robot_first:
        env['PYTHONPATH'] = ':'.join(
            [str(ROBOT / 'build/obot'), str(WORKSPACE / 'build/obot')])
        env['AMENT_PREFIX_PATH'] = ':'.join(
            [str(ROBOT / 'install/obot'), str(WORKSPACE / 'install/obot')])
        env['COLCON_PREFIX_PATH'] = str(ROBOT / 'install')

    result = subprocess.run(
        ['/bin/bash', str(LAUNCHER), '--log-level', 'warn'],
        env=env, text=True, capture_output=True, check=True,
    )
    data = json.loads(result.stdout)
    assert data['prefix'] == str(WORKSPACE / 'install/obot')
    assert 'view2cams' in data['entrypoints']
    assert data['valve'].startswith('obot_valve_interfaces.')
    assert data['click'].startswith('obot_msgs.')
    assert data['args'] == [
        'ros2', 'run', 'obot', 'view2cams', '--ros-args',
        '-p', 'click.enable_motion:=true',
        '-p', 'directional.linear_limit_mps:=1.0',
        '-p', 'directional.max_speed_mps:=1.0',
        '-p', 'center_thickness:=4',
        '-p', 'center_height_fraction:=0.4',
        '--log-level', 'warn',
    ]


def test_launcher_warns_if_valve_interface_is_missing(tmp_path):
    """Continue launching and warn when the valve interface is not built."""
    if not all(p.exists() for p in (REQUIRED[0], REQUIRED[1], REQUIRED[4])):
        pytest.skip('requires the built laptop workspace and ROS Jazzy')

    home = tmp_path / 'home'
    (home / 'Workspaces').mkdir(parents=True)
    (home / 'Workspaces/obot_ws').symlink_to(
        WORKSPACE, target_is_directory=True)
    bash_env = tmp_path / 'bash_env'
    bash_env.write_text('exec() { printf "viewer launch reached\\n"; }\n')
    result = subprocess.run(
        ['/bin/bash', str(LAUNCHER)],
        env={
            'HOME': str(home), 'PATH': '/usr/bin:/bin',
            'BASH_ENV': str(bash_env),
        },
        text=True, capture_output=True, check=True,
    )
    assert 'valve interface package not found' in result.stderr
    assert 'viewer launch reached' in result.stdout
