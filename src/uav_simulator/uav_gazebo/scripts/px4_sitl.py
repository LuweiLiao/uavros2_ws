#!/usr/bin/env python3
"""List and run the vendored PX4 / Gazebo Harmonic vehicle catalog."""
import argparse
import json
import os
from pathlib import Path
import shlex
import shutil
import tempfile


def resource_root():
    script = Path(__file__).resolve()
    for share in (script.parent.parent, script.parents[2] / 'share/uav_gazebo'):
        candidate = share / 'models/px4_gz'
        if (candidate / 'catalog.json').is_file():
            return candidate
    raise RuntimeError('Cannot locate installed or source px4_gz resources')


def main():
    root = resource_root()
    catalog = {row['model']: row for row in json.loads((root / 'catalog.json').read_text())}
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('list', 'gazebo', 'sitl'))
    parser.add_argument('--model', default='x500', choices=sorted(catalog))
    parser.add_argument('--partition', default='tustin_px4', help='Use the same partition in both terminals')
    parser.add_argument('--plugin-dir', type=Path, help='Directory containing built PX4 Gazebo plugins')
    parser.add_argument('--headless', action='store_true', help='Gazebo server only, with headless sensor rendering')
    parser.add_argument('--paused', action='store_true')
    parser.add_argument('--px4-dir', type=Path, help='PX4 source checkout with build/px4_sitl_default')
    parser.add_argument('--run-dir', type=Path, help='PX4 writable state directory (default: a fresh temporary directory)')
    parser.add_argument('--dry-run', action='store_true', help='Print environment and command without starting a process')
    args = parser.parse_args()
    if args.action == 'list':
        for name, row in sorted(catalog.items()):
            print(f'{name:26} SYS_AUTOSTART={row["airframe_id"]:5}  world={row["world"]}')
        return
    row = catalog[args.model]
    overrides = {'GZ_PARTITION': args.partition}
    if args.action == 'gazebo':
        overrides['GZ_SIM_RESOURCE_PATH'] = str(root / 'models') + (
            ':' + os.environ['GZ_SIM_RESOURCE_PATH'] if os.environ.get('GZ_SIM_RESOURCE_PATH') else '')
        if args.plugin_dir:
            overrides['GZ_SIM_SYSTEM_PLUGIN_PATH'] = str(args.plugin_dir.resolve()) + (
                ':' + os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH'] if os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH') else '')
        search = overrides.get('GZ_SIM_SYSTEM_PLUGIN_PATH', os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')).split(':')
        required = ['libMotorFailurePlugin.so', 'libGenericMotorModelPlugin.so',
                    'libSpacecraftThrusterModelPlugin.so', 'libAirSpeedPlugin.so', 'libOpticalFlowSystem.so']
        if not args.dry_run:
            missing = [lib for lib in required if not any(p and (Path(p) / lib).is_file() for p in search)]
            if missing:
                parser.error('Build px4_gz_plugins and source its setup, or pass --plugin-dir. Missing: ' + ', '.join(missing))
            if not shutil.which('gz'):
                parser.error('Gazebo Harmonic (gz) is not installed')
        command = ['gz', 'sim', '-v', '3']
        if not args.paused:
            command.append('-r')
        if args.headless:
            command.extend(['-s', '--headless-rendering'])
        command.append(str(root / row['world_file']))
    else:
        if not args.px4_dir:
            parser.error('sitl requires --px4-dir pointing to a built PX4 source checkout')
        build = args.px4_dir.resolve() / 'build/px4_sitl_default'
        executable, romfs = build / 'bin/px4', build / 'etc'
        if not args.dry_run:
            if not executable.is_file() or not os.access(executable, os.X_OK):
                parser.error(f'Missing PX4 executable: {executable}. Run make px4_sitl in the PX4 checkout first.')
            if not (romfs / 'init.d-posix/airframes' / row['airframe_file']).is_file():
                parser.error(f'This PX4 build does not include {row["airframe_file"]}; use the documented revision.')
        run_dir = args.run_dir.resolve() if args.run_dir else (
            Path('/tmp/px4-NEW-RUN-DIRECTORY') if args.dry_run else Path(tempfile.mkdtemp(prefix=f'px4-{args.model}-')))
        if not args.dry_run:
            run_dir.mkdir(parents=True, exist_ok=True)
        overrides.update(PX4_GZ_STANDALONE='1', PX4_SYS_AUTOSTART=str(row['airframe_id']),
                         PX4_GZ_WORLD=row['world'], PX4_GZ_MODEL_NAME=row['instance'])
        command = [str(executable), '-s', 'etc/init.d-posix/rcS', '-w', str(run_dir), str(romfs)]
    if args.action == 'sitl':
        print('unset PX4_SIM_MODEL', flush=True)
    print(' '.join(f'{k}={shlex.quote(v)}' for k, v in overrides.items()), flush=True)
    print(shlex.join(command), flush=True)
    if args.dry_run:
        return
    env = dict(os.environ, **overrides)
    if args.action == 'sitl':
        env.pop('PX4_SIM_MODEL', None)
    os.execvpe(command[0], command, env)


if __name__ == '__main__':
    main()
