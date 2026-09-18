"""Inspect a corridor with native Gazebo screenshots and owned-process cleanup."""
import importlib.util
import json
import math
import os
import subprocess
from pathlib import Path
import time

WS = Path('/home/llw/Projects/uavros2_ws')
spec = importlib.util.spec_from_file_location('flight', WS/'src/uav_simulator/uav_control/scripts/powerline_perching_sim.py')
flight = importlib.util.module_from_spec(spec)
spec.loader.exec_module(flight)
flight.OUT = Path.home()/'.ros/log'/time.strftime('corridor_preview_%Y%m%d_%H%M%S')
flight.OUT.mkdir(parents=True)
os.environ['GZ_PARTITION'] = 'corridor_' + str(os.getpid())
os.environ['ROS_DOMAIN_ID'] = '73'
report = {'partition': os.environ['GZ_PARTITION'], 'world': 'powerline_corridor', 'screenshots': []}
stamp=time.strftime('%Y%m%d_%H%M%S')
control=WS/'.tmp/powerline_perching'/('corridor_video_'+stamp)
video=Path.home()/('powerline_corridor_'+stamp+'.mp4')
os.environ.update(LD_PRELOAD=str(WS/'.tmp/powerline_perching/pip_recorder/build/libperching_video_helper.so'),
                  UAVROS_GZ_VIDEO_OUTPUT=str(video),UAVROS_GZ_VIDEO_CONTROL=str(control),UAVROS_GZ_WIDE='1')
try:
    flight.start('gazebo', ['ros2', 'launch', 'uav_gazebo', 'powerline_perching.launch', 'world_name:=powerline_corridor'])
    end = time.monotonic()+60
    while time.monotonic() < end:
        listing=subprocess.run(['gz','service','-l'],capture_output=True,text=True,timeout=5)
        if '/gui/screenshot' in listing.stdout and '/gui/move_to/pose' in listing.stdout:
            break
        time.sleep(.2)
    else:
        raise RuntimeError('Gazebo GUI not ready')
    time.sleep(8)
    control.with_suffix('.start').touch()
    time.sleep(4)
    views = [('overview', (65,-80,38), (0,0,8)),
             ('tower', (65,-20,17), (50,0,9)),
             ('hardware', (53,-9,15), (50,-3.35,14))]
    for label, pos, target in views:
        dx,dy,dz = [target[i]-pos[i] for i in range(3)]
        yaw,pitch = math.atan2(dy,dx), math.atan2(-dz, math.hypot(dx,dy))
        q = (-math.sin(pitch/2)*math.sin(yaw/2), math.sin(pitch/2)*math.cos(yaw/2), math.cos(pitch/2)*math.sin(yaw/2), math.cos(pitch/2)*math.cos(yaw/2))
        flight.service('/gui/move_to/pose','gz.msgs.GUICamera', 'pose: {position: {x: %s y: %s z: %s} orientation: {x: %s y: %s z: %s w: %s}}' % (*pos,*q))
        time.sleep(3)
        pictures = Path.home()/'.gz/gui/pictures'
        before = set(pictures.glob('*.png'))
        flight.service('/gui/screenshot','gz.msgs.StringMsg','data: ""')
        time.sleep(2)
        new = sorted(set(pictures.glob('*.png'))-before)
        if not new:
            raise RuntimeError('No screenshot')
        report['screenshots'].append({'view':label, 'path':str(new[-1])})
        with (pictures/'PROVENANCE.txt').open('a') as f:
            f.write(f'\nWorld: powerline_corridor\nGZ_PARTITION: {os.environ["GZ_PARTITION"]}\nTool: Gazebo built-in Screenshot /gui/screenshot\nView: {label}\nOutput: {new[-1]}\n')
    control.with_suffix('.stop').touch()
    end=time.monotonic()+20
    while time.monotonic()<end:
        if video.exists() and 'OnSave returned 1' in (flight.OUT/'gazebo.log').read_text():break
        time.sleep(.2)
    else:raise RuntimeError('Native video not saved')
    report['video']=str(video)
    with (video.parent/'PROVENANCE.txt').open('a') as f:
        f.write(f'\nWorld: powerline_corridor\nGZ_PARTITION: {os.environ["GZ_PARTITION"]}\nTool: Gazebo built-in VideoRecorder, native encoder and OnStart/OnStop/OnSave\nOutput: {video}\nRun: {flight.OUT}\n')
    report['passed'] = True
finally:
    report['cleanup'] = flight.stop()
    report['processes'] = [{'pid':p.pid, 'pgid':p.pid, 'name':n} for n,p,f in flight.children]
    (flight.OUT/'result.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2), flush=True)
