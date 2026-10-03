"""Gazebo entry point for all pinned PX4 Harmonic vehicle configurations."""
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def start_gazebo(context):
    prefix = Path(get_package_prefix('uav_gazebo'))
    plugins = Path(get_package_prefix('px4_gz_plugins')) / 'lib/px4_gz_plugins'
    command = [str(prefix / 'lib/uav_gazebo/px4_sitl.py'), 'gazebo',
               '--model', LaunchConfiguration('model').perform(context),
               '--partition', LaunchConfiguration('partition').perform(context),
               '--plugin-dir', str(plugins)]
    if LaunchConfiguration('gui').perform(context).lower() == 'false':
        command.append('--headless')
    if LaunchConfiguration('paused').perform(context).lower() == 'true':
        command.append('--paused')
    return [ExecuteProcess(cmd=command, output='screen')]


def generate_launch_description():
    root = Path(get_package_share_directory('uav_gazebo')) / 'models/px4_gz'
    names = sorted(row['model'] for row in json.loads((root / 'catalog.json').read_text()))
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='x500', choices=names),
        DeclareLaunchArgument('partition', default_value='tustin_px4'),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('paused', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=start_gazebo),
    ])
