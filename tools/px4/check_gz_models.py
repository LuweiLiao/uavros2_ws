#!/usr/bin/env python3
"""Check all catalog worlds; optionally run isolated, finite Gazebo smoke tests."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import uuid
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
DEFAULT = ROOT / 'src/uav_simulator/uav_gazebo/models/px4_gz'


def check_assets(root, catalog):
    """Catch omitted airframes, missing visual assets, and accidental network URIs."""
    if {e['airframe_file'] for e in catalog} != {p.name for p in (root / 'airframes').iterdir()}:
        raise ValueError('Catalog does not cover the complete airframe snapshot')
    for line in (root / 'SHA256SUMS').read_text().splitlines():
        expected, name = line.split('  ', 1)
        if hashlib.sha256((root / name).read_bytes()).hexdigest() != expected:
            raise ValueError(f'Asset checksum mismatch: {name}')
    files = [*(root / 'models').glob('*/model.sdf'), *(root / 'generated_worlds').glob('*.sdf')]
    for path in files:
        for element in ET.parse(path).findall('.//uri'):
            uri = element.text.strip()
            if uri.startswith('model://'):
                target = root / 'models' / uri[8:]
            elif '://' not in uri:
                target = path.parent / uri
            else:
                raise ValueError(f'External resource in {path}: {uri}')
            if not target.exists():
                raise ValueError(f'Missing resource in {path}: {uri}')
    for entry in catalog:
        world = ET.parse(root / entry['world_file']).find('world')
        if world.get('name') != entry['world'] or world.findtext('include/name') != entry['instance']:
            raise ValueError(f'World / SITL binding mismatch: {entry["model"]}')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--resources', type=Path, default=DEFAULT)
    p.add_argument('--runtime', action='store_true')
    p.add_argument('--plugin-dir', type=Path)
    p.add_argument('--model', action='append', help='Limit to named models; default: all')
    p.add_argument('--output', type=Path, default=ROOT / '.tmp/px4_sitl/checks')
    args = p.parse_args()
    root = args.resources.resolve()
    catalog = json.loads((root / 'catalog.json').read_text())
    check_assets(root, catalog)
    print('Airframe coverage, checksums, resource closure and SITL bindings: PASS', flush=True)
    if args.model:
        unknown = set(args.model) - {e['model'] for e in catalog}
        if unknown:
            p.error(f'Unknown models: {unknown}')
        catalog = [e for e in catalog if e['model'] in args.model]
    args.output.mkdir(parents=True, exist_ok=True)
    results = []
    for entry in catalog:
        env = dict(os.environ, SDF_PATH=str(root / 'models'),
                   GZ_SIM_RESOURCE_PATH=str(root / 'models'),
                   GZ_PARTITION='px4_check_' + uuid.uuid4().hex)
        if args.plugin_dir:
            env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = str(args.plugin_dir.resolve())
        world = str(root / entry['world_file'])
        log = args.output / (entry['model'] + '.log')
        command = (['gz', 'sim', '-s', '-r', '--headless-rendering', '--iterations', '100', '-v', '3', world]
                   if args.runtime else ['gz', 'sdf', '-k', world])
        with log.open('w') as output:
            proc = subprocess.Popen(command, env=env, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
            timed_out = False
            try:
                code = proc.wait(timeout=45)
            except subprocess.TimeoutExpired:
                timed_out = True
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    code = proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    code = proc.wait()
        content = log.read_text(errors='replace')
        errors = [line for line in content.splitlines() if any(s in line for s in (
            '[Err]', 'Error Code', 'Error [', 'Failed to load', 'Unable to find', 'Segmentation fault'))]
        passed = code == 0 and not timed_out and not errors
        result = {'model': entry['model'], 'passed': passed, 'exit_code': code,
                  'timed_out': timed_out, 'errors': errors}
        results.append(result)
        print(f'{entry["model"]}: {"PASS" if passed else "FAIL"}', flush=True)
        (args.output / 'results.json').write_text(json.dumps(results, indent=2) + '\n')
    return 0 if all(r['passed'] for r in results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
