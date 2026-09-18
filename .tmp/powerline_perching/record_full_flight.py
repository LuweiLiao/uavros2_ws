"""Rerun the production controller; capture only with Gazebo VideoRecorder."""
import importlib.util
import json
import math
import os
from pathlib import Path
import time

WS = Path('/home/llw/Projects/uavros2_ws')
spec = importlib.util.spec_from_file_location('flight', WS/'src/uav_simulator/uav_control/scripts/powerline_perching_sim.py')
flight = importlib.util.module_from_spec(spec)
spec.loader.exec_module(flight)
stamp = time.strftime('%Y%m%d_%H%M%S')
output = Path.home()/f'powerline_perching_pip_{stamp}.mp4'
control = WS/'.tmp/powerline_perching'/f'native_video_{stamp}'
state = {'tool': 'Gazebo VideoRecorder with local live PIP extension', 'output': str(output), 'started': False, 'saved': False}
original_start, original_command, original_stop, original_screenshot = flight.start, flight.Companion.command, flight.stop, flight.screenshot
original_phase = flight.Companion.phase

def phase(self, name):
    original_phase(self, name)
    if name == 'GUIDED_VISUAL_DESCENT' and flight.SCENE['detector'] == 'gray':
        x,y,z = 8,-5.5,15.2
        dy = -3.35-y
        yaw,pitch = math.atan2(dy,-x),math.atan2(z-13.53674,math.hypot(x,dy))
        q=(-math.sin(pitch/2)*math.sin(yaw/2),math.sin(pitch/2)*math.cos(yaw/2),math.cos(pitch/2)*math.sin(yaw/2),math.cos(pitch/2)*math.cos(yaw/2))
        flight.service('/gui/move_to/pose','gz.msgs.GUICamera','pose: {position: {x: %s y: %s z: %s} orientation: {x: %s y: %s z: %s w: %s}}' % (x,y,z,*q))

def start(name, cmd, cwd=WS):
    if name == 'camera_bridge':
        cmd = cmd + ['/powerline_perching/vision/annotated_image@sensor_msgs/msg/Image]gz.msgs.Image']
    if name != 'gazebo':
        return original_start(name, cmd, cwd)
    settings = dict(LD_PRELOAD=str(WS/'.tmp/powerline_perching/pip_recorder/build/libperching_video_helper.so'),
                    PERCHING_VIDEO_PLUGIN=str(WS/'.tmp/powerline_perching/pip_recorder/build/libVideoRecorder.so'),
                    UAVROS_GZ_VIDEO_OUTPUT=str(output), UAVROS_GZ_VIDEO_CONTROL=str(control), UAVROS_GZ_WIDE='1',
                    GZ_GUI_PLUGIN_PATH=str(WS/'.tmp/powerline_perching/pip_recorder/build'))
    previous = {key: os.environ.get(key) for key in settings}
    os.environ.update(settings)
    try:
        return original_start(name, cmd, cwd)
    finally:
        for key, value in previous.items():
            if value is None: os.environ.pop(key, None)
            else: os.environ[key] = value

def command(self, cmd, *params):
    if cmd == 400 and params[0] == 1 and not state['started']:
        # Frame the complete 0--3 m flight path and wire without camera cuts.
        x, y, z, tz = 2.5, -4.5, 2.8, 1.6
        target_y = 0
        if flight.SCENE['detector'] == 'gray':
            x,y,z,tz,target_y = 62,-75,30,8,-3.35
        yaw, pitch = math.atan2(target_y-y, -x), math.atan2(z-tz, math.hypot(x,target_y-y))
        q = (-math.sin(pitch/2)*math.sin(yaw/2), math.sin(pitch/2)*math.cos(yaw/2),
             math.cos(pitch/2)*math.sin(yaw/2), math.cos(pitch/2)*math.cos(yaw/2))
        req = 'pose: {position: {x: %s y: %s z: %s} orientation: {x: %s y: %s z: %s w: %s}}' % (x,y,z,*q)
        flight.service('/gui/move_to/pose', 'gz.msgs.GUICamera', req)
        self.pump(2)
        control.with_suffix('.start').touch()
        deadline = time.monotonic()+12
        while time.monotonic()<deadline:
            self.rc(throttle=1000)
            self.pump(.1)
            log = (flight.OUT/'gazebo.log').read_text()
            if 'OnStart returned 1' in log and 'PIP sensor subscription: ' in log:break
        else:raise RuntimeError('Native recorder did not start')
        state.update(started=True, partition=os.environ['GZ_PARTITION'], world=flight.SCENE['world'], camera=req)
        self.pump(2)
        print('NATIVE_RECORDING', output, flush=True)
    return original_command(self, cmd, *params)

def finish():
    if not state['started'] or state['saved']:return
    control.with_suffix('.stop').touch()
    deadline = time.monotonic()+25
    while time.monotonic()<deadline:
        if output.exists() and output.stat().st_size>1000 and 'OnSave returned 1' in (flight.OUT/'gazebo.log').read_text():break
        time.sleep(.2)
    state['saved'] = output.exists() and output.stat().st_size>1000
    if state['saved']:
        with (output.parent/'PROVENANCE.txt').open('a') as f:
            f.write('\nWorld: '+state['world']+'\nGZ_PARTITION: '+state['partition']+'\nTool: Gazebo VideoRecorder local live PIP extension; native OnStart/OnStop/OnSave and encoder\nOutput: '+str(output)+'\nRun: '+str(flight.OUT)+'\nTiming: simulation time; sensor frame chosen at or before scene time, maximum age 200 ms\nPlugin: '+str(WS/'.tmp/powerline_perching/pip_recorder/build/libVideoRecorder.so')+'\n')
    (flight.OUT/'native_video.json').write_text(json.dumps(state,indent=2)+'\n')
    print('NATIVE_SAVED', state['saved'], output, flush=True)

def screenshot(label):
    finish()
    if not state['saved']:raise RuntimeError('Native recording save failed')
    return original_screenshot(label)

def stop():
    try:finish()
    finally:return original_stop()

flight.start, flight.Companion.command = start, command
flight.Companion.phase = phase
flight.stop, flight.screenshot = stop, screenshot
if output.exists():raise RuntimeError('Refusing to overwrite video')
raise SystemExit(flight.main())
