#!/usr/bin/env python3
"""Import a pinned PX4 Gazebo resource tree and predownloaded Fuel archives."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET
from zipfile import ZipFile

ROOT = Path(__file__).resolve().parents[2]
DEST = ROOT / 'src/uav_simulator/uav_gazebo/models/px4_gz'


def revision(path):
    return subprocess.check_output(['git', '-C', str(path), 'rev-parse', 'HEAD'], text=True).strip()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--px4-source', type=Path, required=True)
    parser.add_argument('--fuel-cache', type=Path, required=True)
    args = parser.parse_args()
    px4 = args.px4_source.resolve()
    upstream = px4 / 'Tools/simulation/gz'
    DEST.mkdir(exist_ok=True, parents=True)
    for directory in ('models', 'worlds'):
        shutil.copytree(upstream / directory, DEST / directory, dirs_exist_ok=True)
    shutil.copy2(upstream / 'LICENSE', DEST / 'LICENSE')
    (DEST / 'airframes').mkdir(exist_ok=True)
    for airframe in (px4 / 'ROMFS/px4fmu_common/init.d-posix/airframes').glob('*_gz_*'):
        shutil.copy2(airframe, DEST / 'airframes' / airframe.name)
    provenance = {'px4_revision': revision(px4), 'models_revision': revision(upstream),
                  'models_url': 'https://github.com/PX4/PX4-gazebo-models', 'fuel': []}
    for name in ('atmos', 'atmos_dual', 'kth_freeflyer'):
        metadata = json.loads((args.fuel_cache / f'{name}.json').read_text())
        archive = args.fuel_cache / f'{name}.zip'
        out = DEST / 'models' / name
        out.mkdir(exist_ok=True)
        with ZipFile(archive) as z:
            for info in z.infolist():
                target = out / info.filename
                if not target.resolve().is_relative_to(out.resolve()):
                    raise ValueError(f'Unsafe archive path: {info.filename}')
                if info.is_dir():
                    target.mkdir(exist_ok=True, parents=True)
                else:
                    target.parent.mkdir(exist_ok=True, parents=True)
                    target.write_bytes(z.read(info))
        entry = {'owner': metadata['owner'], 'name': name, 'version': metadata['version'],
                 'url': f'https://fuel.gazebosim.org/1.0/proque/models/{name}/{metadata["version"]}',
                 'license': metadata['license_name'], 'license_url': metadata['license_url'],
                 'archive_sha256': hashlib.sha256(archive.read_bytes()).hexdigest()}
        provenance['fuel'].append(entry)
        (out / 'FUEL_LICENSE.txt').write_text(
            f"Model: {name}\nAuthor: {entry['owner']}\nSource: {entry['url']}\n"
            f"License: {entry['license']}\n{entry['license_url']}\n"
            'Local modifications: resource URIs and Harmonic plugin library names only.\n')
    metadata = json.loads((args.fuel_cache / 'mecanum_lift.json').read_text())
    wheels = DEST / 'models/rover_mecanum'
    wheel_source = {'owner': metadata['owner'], 'name': metadata['name'],
                    'version': metadata['version'], 'license': metadata['license_name'],
                    'license_url': metadata['license_url'],
                    'url': f'https://fuel.gazebosim.org/1.0/OpenRobotics/models/Mecanum%20lift/{metadata["version"]}',
                    'files': {}}
    for direction in ('left', 'right'):
        filename = f'mecanum_wheel_{direction}.STL'
        source = args.fuel_cache / filename
        shutil.copy2(source, wheels / 'meshes' / filename)
        wheel_source['files'][filename] = hashlib.sha256(source.read_bytes()).hexdigest()
    provenance['fuel'].append(wheel_source)
    (wheels / 'FUEL_LICENSE.txt').write_text(
        'Mecanum wheel meshes (left/right)\nAuthor: OpenRobotics\n'
        f"Source: {wheel_source['url']}\nLicense: {wheel_source['license']}\n"
        f"{wheel_source['license_url']}\nMeshes unmodified. SDF references localized.\n")
    adaptations = []
    colors = {'DarkGrey': '0.2 0.2 0.2 1', 'FlatBlack': '0 0 0 1',
              'Blue': '0 0 1 1', 'Green': '0 1 0 1'}
    for f in sorted((DEST / 'models').glob('*/model.sdf')):
        original = f.read_text()
        s = original
        if re.search(r'\bgz:\w+\s*=', s) and 'xmlns:gz=' not in s:
            s = re.sub(r'<sdf\s', '<sdf xmlns:gz="http://gazebosim.org/schema" ', s, count=1)
        for name, color in colors.items():
            s = re.sub(r'<script>\s*<name>Gazebo/' + name + r'</name>.*?</script>',
                       f'<ambient>{color}</ambient><diffuse>{color}</diffuse>', s, flags=re.S)
        s = s.replace('<uri>x500</uri>', '<uri>model://x500</uri>')
        s = re.sub(r'https://fuel.gazebosim.org/1.0/proque/models/([^/]+)/\d+/files/', r'model://\1/', s)
        s = s.replace('https://fuel.gazebosim.org/1.0/proque/models/kth_freeflyer', 'model://kth_freeflyer')
        s = s.replace('gz-sim-spacecraft-thruster-model-system', 'libSpacecraftThrusterModelPlugin.so')
        s = s.replace('https://fuel.gazebosim.org/1.0/OpenRobotics/models/Mecanum lift/tip/files/',
                      'model://rover_mecanum/')
        if f.parent.name == 'rover_differential':
            s = s.replace("<joint name='cast_wheel_rear_link'", "<joint name='cast_wheel_rear_joint'")
        if s != original:
            f.write_text(s)
            adaptations.append(str(f.relative_to(DEST)))
    provenance['adapted_models'] = adaptations
    (DEST / 'PROVENANCE.json').write_text(json.dumps(provenance, indent=2) + '\n')
    catalog = []
    generated = DEST / 'generated_worlds'
    generated.mkdir(exist_ok=True)
    systems = [('physics', 'Physics'), ('user-commands', 'UserCommands'),
               ('scene-broadcaster', 'SceneBroadcaster'), ('contact', 'Contact'),
               ('imu', 'Imu'), ('air-pressure', 'AirPressure'), ('air-speed', 'AirSpeed'),
               ('apply-link-wrench', 'ApplyLinkWrench'), ('navsat', 'NavSat'),
               ('magnetometer', 'Magnetometer'), ('sensors', 'Sensors')]
    for frame in sorted((DEST / 'airframes').iterdir()):
        number, name = frame.name.split('_gz_', 1)
        content = frame.read_text()
        match = re.search(r'PX4_GZ_WORLD:=([^}]+)', content)
        world_template = match.group(1) if match else 'default'
        tree = ET.parse(DEST / 'worlds' / f'{world_template}.sdf')
        world = tree.getroot().find('world')
        world.set('name', f'px4_{name}')
        # The seabed is scenery fetched from Fuel. Preserve water physics but make
        # the generated vehicle world self-contained; raw upstream world remains.
        for include in list(world.findall('include')):
            if 'fuel.gazebosim.org' in include.findtext('uri', ''):
                world.remove(include)
        existing = {p.get('filename') for p in world.findall('plugin')}
        for filename, classname in systems:
            filename = f'gz-sim-{filename}-system'
            if filename not in existing:
                plugin = ET.SubElement(world, 'plugin', filename=filename, name=f'gz::sim::systems::{classname}')
                if classname == 'Sensors':
                    ET.SubElement(plugin, 'render_engine').text = 'ogre2'
        if name == 'x500_flow':
            ET.SubElement(world, 'plugin', filename='libOpticalFlowSystem.so', name='custom::OpticalFlowSystem')
        include = ET.SubElement(world, 'include')
        ET.SubElement(include, 'uri').text = f'model://{name}'
        ET.SubElement(include, 'name').text = f'{name}_0'
        # Omit pose to retain the upstream model's placement and landing geometry.
        ET.indent(tree, space='  ')
        world_file = f'generated_worlds/{name}.sdf'
        tree.write(DEST / world_file, encoding='utf-8', xml_declaration=True)
        catalog.append({'model': name, 'airframe_id': int(number), 'airframe_file': frame.name,
                        'world': f'px4_{name}', 'world_file': world_file,
                        'instance': f'{name}_0', 'upstream_world': world_template})
    (DEST / 'catalog.json').write_text(json.dumps(catalog, indent=2) + '\n')
    sums = []
    for d in ('models', 'airframes', 'worlds', 'generated_worlds'):
        for f in sorted((DEST / d).rglob('*')):
            if f.is_file():
                sums.append(hashlib.sha256(f.read_bytes()).hexdigest() + '  ' + str(f.relative_to(DEST)))
    (DEST / 'SHA256SUMS').write_text('\n'.join(sums) + '\n')
    print(f'Imported {len(catalog)} PX4 airframes, {len(list((DEST / "models").iterdir()))} resource models')


if __name__ == '__main__':
    main()
