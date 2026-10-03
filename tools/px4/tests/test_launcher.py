"""Exercise the process boundary without requiring a PX4 firmware binary."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / 'src/uav_simulator/uav_gazebo/scripts/px4_sitl.py'
CATALOG = ROOT / 'src/uav_simulator/uav_gazebo/models/px4_gz/catalog.json'


class LauncherTest(unittest.TestCase):
    def test_every_airframe_binds_existing_model_and_clears_spawn_variable(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            build = root / 'build/px4_sitl_default'
            (build / 'bin').mkdir(parents=True)
            frames = build / 'etc/init.d-posix/airframes'
            frames.mkdir(parents=True)
            binary = build / 'bin/px4'
            binary.write_text('#!/usr/bin/env python3\nimport os, sys, json\n'
                              'print(json.dumps({"args":sys.argv[1:], "env":dict(os.environ)}))\n')
            binary.chmod(0o755)
            for row in json.loads(CATALOG.read_text()):
                with self.subTest(model=row['model']):
                    (frames / row['airframe_file']).touch()
                    run = root / 'state' / row['model']
                    env = dict(os.environ, PX4_SIM_MODEL='stale_model', PX4_GZ_WORLD='wrong_world')
                    proc = subprocess.run(['python3', str(SCRIPT), 'sitl', '--model', row['model'],
                                           '--px4-dir', str(root), '--run-dir', str(run),
                                           '--partition', 'test_partition'], env=env, text=True,
                                          capture_output=True, check=True)
                    output = json.loads(proc.stdout.splitlines()[-1])
                    actual = output['env']
                    self.assertNotIn('PX4_SIM_MODEL', actual)
                    self.assertEqual(actual['PX4_GZ_MODEL_NAME'], row['instance'])
                    self.assertEqual(actual['PX4_GZ_WORLD'], row['world'])
                    self.assertEqual(actual['PX4_SYS_AUTOSTART'], str(row['airframe_id']))
                    self.assertEqual(actual['PX4_GZ_STANDALONE'], '1')
                    self.assertEqual(actual['GZ_PARTITION'], 'test_partition')
                    self.assertEqual(output['args'], ['-s', 'etc/init.d-posix/rcS', '-w', str(run), str(build / 'etc')])

    def test_dry_run_does_not_create_state_or_require_firmware(self):
        with tempfile.TemporaryDirectory() as directory:
            state = Path(directory) / 'does-not-exist'
            subprocess.run(['python3', str(SCRIPT), 'sitl', '--px4-dir', directory,
                            '--run-dir', str(state), '--dry-run'], check=True, capture_output=True)
            self.assertFalse(state.exists())

    def test_unknown_model_and_missing_firmware_fail_before_launch(self):
        for arguments in (['gazebo', '--model', 'not_a_px4_model'],
                          ['sitl', '--px4-dir', '/nonexistent-px4-checkout']):
            with self.subTest(arguments=arguments):
                result = subprocess.run(['python3', str(SCRIPT), *arguments], capture_output=True)
                self.assertEqual(result.returncode, 2)


if __name__ == '__main__':
    unittest.main()
