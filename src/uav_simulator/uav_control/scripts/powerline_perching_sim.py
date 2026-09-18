#!/usr/bin/env python3
"""SITL-only Gazebo flight: LOITER takeoff, image-guided perching, normal disarm.

AI-assisted implementation. Requires pymavlink, OpenCV, NumPy and ROS 2 Jazzy.
No hardware endpoint is accepted. Camera targets never use Gazebo model poses.
"""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import time

import cv2
cv2.setNumThreads(1)
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import String
from ros_gz_interfaces.msg import Contacts
from pymavlink import mavutil

WS = Path("/home/llw/Projects/uavros2_ws")
AP = Path("/home/llw/Projects/ardupilot")
OUT = None
SCENES = {
    "test": dict(world="powerline_perching", file="powerline_perching_flight", takeoff=2.8,
                 wire_height=1.6, wire_y=0.0, contact="test_wire", detector="orange"),
    "corridor": dict(world="powerline_corridor", file="powerline_corridor_flight", takeoff=15.0,
                     wire_height=14.787-1000*(math.cosh(.05)-1), wire_y=-3.35,
                     contact="phase_1_0_", detector="gray"),
}
SCENE = SCENES["test"]
# OpenCV wheels set a Qt plugin path incompatible with Gazebo's system Qt.
for key in ("QT_QPA_PLATFORM_PLUGIN_PATH", "QT_QPA_FONTDIR"):
    if "cv2" in os.environ.get(key, ""):
        os.environ.pop(key)
children = []


def start(name, cmd, cwd=WS):
    f = (OUT / (name + ".log")).open("w")
    p = subprocess.Popen(
        cmd, cwd=cwd, stdout=f, stderr=subprocess.STDOUT, start_new_session=True
    )
    children.append((name, p, f))
    return p


def service(name, typ, req):
    r = subprocess.run(
        [
            "gz",
            "service",
            "-s",
            name,
            "--reqtype",
            typ,
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "5000",
            "--req",
            req,
        ],
        capture_output=True,
        text=True,
        timeout=8,
    )
    if "true" not in r.stdout:
        raise RuntimeError(name + ": " + r.stdout)


def screenshot(label):
    yaw = math.atan2(3, -2)
    pitch = math.atan2(2, math.hypot(2, 3))
    q = (
        -math.sin(pitch / 2) * math.sin(yaw / 2),
        math.sin(pitch / 2) * math.cos(yaw / 2),
        math.cos(pitch / 2) * math.sin(yaw / 2),
        math.cos(pitch / 2) * math.cos(yaw / 2),
    )
    service(
        "/gui/move_to/pose",
        "gz.msgs.GUICamera",
        "pose: {position: {x: 2 y: %s z: %s} orientation: {x: %s y: %s z: %s w: %s}}"
        % (SCENE["wire_y"]-3, SCENE["wire_height"]+1.9, *q),
    )
    time.sleep(2)
    directory = Path.home() / ".gz/gui/pictures"
    before = set(directory.glob("*.png"))
    service("/gui/screenshot", "gz.msgs.StringMsg", 'data: ""')
    time.sleep(1)
    files = sorted(set(directory.glob("*.png")) - before)
    if not files:
        raise RuntimeError("Native screenshot missing")
    with (directory / "PROVENANCE.txt").open("a") as f:
        f.write(
            "\nWorld: " + SCENE["world"] + "\nGZ_PARTITION: "
            + os.environ["GZ_PARTITION"]
            + "\nTool: Gazebo built-in Screenshot /gui/screenshot\nStage: "
            + label
            + "\nOutput: "
            + ", ".join(map(str, files))
            + "\n"
        )
    return list(map(str, files))


def stop():
    status = {}
    for name, p, f in reversed(children):
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            try:
                os.killpg(p.pid, sig)
            except ProcessLookupError:
                break
            end = time.monotonic() + 5
            while time.monotonic() < end:
                p.poll()
                try:
                    os.killpg(p.pid, 0)
                except ProcessLookupError:
                    break
                time.sleep(0.1)
            else:
                continue
            break
        p.wait(timeout=5)
        f.close()
        try:
            os.killpg(p.pid, 0)
            status[name] = False
        except ProcessLookupError:
            status[name] = True
    return status


class Companion:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("powerline_vision")
        self.rgb = None
        self.depth = None
        self.K = None
        self.rgb_stamp = 0
        self.depth_stamp = 0
        self.image_wall = 0
        self.contact = False
        self.contact_time = 0
        self.target = None
        self.target_time = 0
        self.mav = None
        self.msgs = {}
        self.state = "START"
        self.last_rc = 0
        self.last_heartbeat = 0
        self.rows = []
        self.truth = None
        self.last_velocity = None
        self.vision_disabled = False
        self.transitions = []
        self.mode_times = {}
        self.node.create_subscription(
            Image,
            "/powerline_perching/down_camera/image",
            self.image,
            qos_profile_sensor_data,
        )
        self.node.create_subscription(
            Image,
            "/powerline_perching/down_camera/depth_image",
            self.image,
            qos_profile_sensor_data,
        )
        self.node.create_subscription(
            CameraInfo,
            "/powerline_perching/down_camera/camera_info",
            lambda m: setattr(self, "K", m.k),
            qos_profile_sensor_data,
        )
        self.node.create_subscription(
            Contacts, "/powerline_perching/contact", self.touch, 10
        )
        # Evaluation only: never read this in detection or flight guidance.
        self.node.create_subscription(
            Odometry, "/powerline_perching/odometry", self.evaluation, 10
        )
        self.target_pub = self.node.create_publisher(
            Vector3Stamped, "/powerline_perching/vision/wire_offset_ned", 10
        )
        self.phase_pub = self.node.create_publisher(
            String, "/powerline_perching/phase", 10
        )
        self.annotated_pub = self.node.create_publisher(
            Image, "/powerline_perching/vision/annotated_image", 10
        )
        self.vision_pixels = None
        self.detector = SCENE["detector"]
        self.node.create_timer(0.05, self.detect)
        self.log = (OUT / "telemetry.jsonl").open("w", buffering=1)

    def evaluation(self, m):
        p = m.pose.pose.position
        v = m.twist.twist.linear
        self.truth = dict(
            x=p.x, y=p.y, z=p.z, speed=math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
        )

    def image(self, m):
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if m.encoding == "32FC1":
            self.depth = np.ndarray(
                (m.height, m.width),
                dtype="<f4",
                buffer=bytes(m.data),
                strides=(m.step, 4),
            ).copy()
            self.depth_stamp = stamp
        elif m.encoding in ("rgb8", "bgr8"):
            a = np.ndarray(
                (m.height, m.width, 3),
                dtype="u1",
                buffer=bytes(m.data),
                strides=(m.step, 3, 1),
            ).copy()
            self.rgb = a if m.encoding == "rgb8" else a[:, :, ::-1]
            self.rgb_stamp = stamp
            self.image_wall = time.monotonic()

    def touch(self, m):
        self.contact = any(
            SCENE["contact"] in c.collision1.name or SCENE["contact"] in c.collision2.name
            for c in m.contacts
        )
        if self.contact:
            self.contact_time = time.monotonic()

    def pump(self, seconds=0.05):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            for name, process, _ in children:
                if process.poll() is not None:
                    raise RuntimeError(f"Simulation process exited: {name}")
            rclpy.spin_once(self.node, timeout_sec=0.001)
            if self.mav:
                for _ in range(80):
                    m = self.mav.recv_match(blocking=False)
                    if m is None:
                        break
                    self.msgs[m.get_type()] = m
                    if m.get_type() == "STATUSTEXT":
                        print("AP:", m.text, flush=True)
                if time.monotonic() - self.last_heartbeat > 1:
                    self.mav.mav.heartbeat_send(6, 8, 0, 0, 4)
                    self.last_heartbeat = time.monotonic()

    def command(self, cmd, *params):
        self.msgs.pop("COMMAND_ACK", None)
        self.mav.mav.command_long_send(
            1, 1, cmd, 0, *(list(params) + [0] * (7 - len(params)))
        )
        end = time.monotonic() + 5
        while time.monotonic() < end:
            self.pump()
            a = self.msgs.get("COMMAND_ACK")
            if a and a.command == cmd:
                if a.result not in (0, 5):
                    raise RuntimeError(f"Command {cmd} rejected {a.result}")
                return
        raise RuntimeError(f"No ACK {cmd}")

    def mode(self, name):
        self.command(176, 1, self.mav.mode_mapping()[name])
        end = time.monotonic() + 6
        while time.monotonic() < end:
            self.pump()
            if (
                self.msgs.get("HEARTBEAT")
                and mavutil.mode_string_v10(self.msgs["HEARTBEAT"]) == name
            ):
                return
        raise RuntimeError("Mode not confirmed " + name)

    def rc(self, roll=1500, pitch=1500, throttle=1500):
        self.mav.mav.rc_channels_override_send(
            1, 1, int(roll), int(pitch), int(throttle), 1500, 65535, 65535, 65535, 65535
        )

    def velocity(self, north, east, down, yaw_rate=0):
        # Ignore positions, accelerations and yaw; enable velocity and yaw rate.
        self.mav.mav.set_position_target_local_ned_send(
            0, 1, 1, 1, 1479, 0, 0, 0, north, east, down, 0, 0, 0, 0, yaw_rate
        )
        self.last_velocity = dict(north=north, east=east, down=down, yaw_rate=yaw_rate)

    def detect(self):
        self.vision_pixels = None
        result = self.detect_wire()
        if self.rgb is not None:
            self.publish_annotation(result)
        return result

    def publish_annotation(self, result):
        image = self.rgb.copy()
        h, w = image.shape[:2]
        centre = (w // 2, h // 2)
        cv2.drawMarker(image, centre, (255, 255, 255), cv2.MARKER_CROSS, 24, 2)
        if result is not None:
            u, v = self.vision_pixels
            image[v, u] = (30, 255, 90)
            pixels = np.column_stack((u, v)).astype(np.float32)
            dx, dy, x, y = cv2.fitLine(pixels, cv2.DIST_L2, 0, 0.01, 0.01).ravel()
            along = (pixels - (x, y)) @ np.array([dx, dy])
            a = tuple(np.rint([x, y] + along.min() * np.array([dx, dy])).astype(int))
            b = tuple(np.rint([x, y] + along.max() * np.array([dx, dy])).astype(int))
            cv2.line(image, a, b, (30, 255, 90), 3, cv2.LINE_AA)
            nearest = np.array([x, y]) + np.dot(np.array(centre) - (x, y), [dx, dy]) * np.array([dx, dy])
            cv2.arrowedLine(image, centre, tuple(np.rint(nearest).astype(int)), (255, 230, 30), 2, tipLength=0.2)
            text = f"WIRE OK | {result['pixels']} px"
            detail = f"Offset {math.hypot(result['north'], result['east']):.3f} m | yaw {math.degrees(result['yaw_error']):+.1f} deg"
            color = (30, 255, 90)
        else:
            text, detail, color = "NO VALID WIRE", "Waiting / out of view / rejected", (255, 190, 50)
        cv2.rectangle(image, (0, 0), (w, 62), (20, 24, 28), -1)
        cv2.putText(image, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2, cv2.LINE_AA)
        cv2.putText(image, detail, (10, 51), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        msg = Image()
        msg.header.stamp.sec = int(self.rgb_stamp)
        msg.header.stamp.nanosec = int(round((self.rgb_stamp - int(self.rgb_stamp)) * 1e9))
        msg.header.frame_id = "down_camera_optical"
        msg.height, msg.width, msg.encoding, msg.step = h, w, "rgb8", w * 3
        msg.data = image.tobytes()
        self.annotated_pub.publish(msg)

    def detect_wire(self):
        if self.vision_disabled:
            return None
        if self.rgb is None or self.depth is None or self.K is None:
            return None
        if time.monotonic() - self.image_wall > 0.5:
            return None
        if abs(self.rgb_stamp - self.depth_stamp) > 0.11:
            return None
        att = self.msgs.get("ATTITUDE")
        if att is None:
            return None
        # Image appearance plus measured depth; never world pose lookup.
        hsv = cv2.cvtColor(self.rgb, cv2.COLOR_RGB2HSV)
        gray = getattr(self, "detector", "orange") == "gray"
        if gray:
            mask = cv2.inRange(hsv, np.array([0, 0, 65]), np.array([179, 80, 255]))
            mask[~(np.isfinite(self.depth) & (self.depth > .045) & (self.depth < 8))] = 0
            _, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
            keep = np.zeros(len(stats), dtype=bool)
            for index, (x, y, width, height, area) in enumerate(stats[1:], 1):
                keep[index] = (25 <= area < mask.size * .15 and
                               max(width, height) > 3 * min(width, height))
            mask = keep[labels].astype(np.uint8) * 255
        else:
            mask = cv2.inRange(hsv, np.array([5, 110, 70]), np.array([30, 255, 255]))
        vv, uu = np.nonzero(mask)
        if len(uu) < 25:
            return None
        dd = self.depth[vv, uu]
        valid = np.isfinite(dd) & (dd > 0.045) & (dd < 8)
        u, v, d = uu[valid], vv[valid], dd[valid]
        if len(u) < 25:
            return None
        fx, fy, cx, cy = self.K[0], self.K[4], self.K[2], self.K[5]
        body = np.column_stack((0.32 - (v - cy) * d / fy, (u - cx) * d / fx, d - 0.568))
        # Reject orange saddle roof and outliers; camera overhang sees forward wire.
        forward = body[:, 0] > 0.24
        if gray:
            self_airframe = ((abs(body[:, 1]) > .16) & (body[:, 0] < .55) &
                             (body[:, 2] > -.20) & (body[:, 2] < .25))
            forward &= ~self_airframe
        body = body[forward]
        u, v = u[forward], v[forward]
        if gray and len(body) >= 25:
            # Separate vertically stacked wires using the nearest supported depth band.
            depths = body[:, 2] + 0.568
            bins = np.floor(depths / 0.10).astype(int)
            ids, counts = np.unique(bins, return_counts=True)
            supported = ids[counts >= 25]
            if not len(supported):
                return None
            near = (supported[0] + .5) * .10
            selected = abs(depths - near) < .18
            body, u, v = body[selected], u[selected], v[selected]
        if len(body) < 20:
            return None
        r, p, y = att.roll, att.pitch, att.yaw
        cr, sr, cp, sp, cyaw, sy = (
            math.cos(r),
            math.sin(r),
            math.cos(p),
            math.sin(p),
            math.cos(y),
            math.sin(y),
        )
        R = np.array(
            [
                [cp * cyaw, sr * sp * cyaw - cr * sy, cr * sp * cyaw + sr * sy],
                [cp * sy, sr * sp * sy + cr * cyaw, cr * sp * sy - sr * cyaw],
                [-sp, sr * cp, cr * cp],
            ]
        )
        points = body @ R.T
        centre = np.median(points, axis=0)
        _, sing, V = np.linalg.svd(points[:, :2] - centre[:2], full_matrices=False)
        if sing[0] < 0.08 or sing[1] / sing[0] > 0.15:
            return None
        direction = V[0]
        nearest = centre[:2] - direction * np.dot(centre[:2], direction)
        yaw = math.atan2(direction[1], direction[0])
        error = (yaw - y + math.pi / 2) % math.pi - math.pi / 2
        result = dict(
            north=float(nearest[0]),
            east=float(nearest[1]),
            down=float(centre[2] + 0.012),
            yaw_error=error,
            pixels=len(body),
            stamp=self.rgb_stamp,
        )
        target_msg = Vector3Stamped()
        target_msg.header.frame_id = "world_ned"
        target_msg.header.stamp.sec = int(self.rgb_stamp)
        target_msg.header.stamp.nanosec = int(self.rgb_stamp % 1 * 1e9)
        target_msg.vector.x = result["north"]
        target_msg.vector.y = result["east"]
        target_msg.vector.z = result["down"]
        self.target_pub.publish(target_msg)
        self.target = result
        self.target_time = time.monotonic()
        self.vision_pixels = (u, v)
        return result

    def record(self):
        lp = self.msgs.get("LOCAL_POSITION_NED")
        att = self.msgs.get("ATTITUDE")
        row = dict(
            wall=time.time(),
            state=self.state,
            vision=self.target,
            contact=self.contact,
            local=lp.to_dict() if lp else None,
            attitude=att.to_dict() if att else None,
            evaluation=self.truth,
            command=self.last_velocity,
        )
        self.log.write(json.dumps(row) + "\n")

    def phase(self, name):
        self.state = name
        self.transitions.append(dict(phase=name, wall=time.time()))
        self.phase_pub.publish(String(data=name))
        print("PHASE", name, flush=True)

    def run(self, inject_vision_loss=False):
        self.mav = mavutil.mavlink_connection("tcp:127.0.0.1:5760", source_system=255)
        self.mav.setup_logfile(str(OUT / "mavlink.tlog"))
        if not self.mav.wait_heartbeat(timeout=20):
            raise RuntimeError("No heartbeat")
        self.mav.mav.request_data_stream_send(1, 1, 0, 20, 1)
        self.phase("PREFLIGHT")
        end = time.monotonic() + 300
        ready = None
        while time.monotonic() < end:
            self.rc(throttle=1000)
            self.pump(0.1)
            ekf = self.msgs.get("EKF_STATUS_REPORT")
            gps = self.msgs.get("GPS_RAW_INT")
            ok = (
                ekf
                and (ekf.flags & 0x33) == 0x33
                and gps
                and gps.fix_type >= 3
                and "LOCAL_POSITION_NED" in self.msgs
            )
            ready = (ready or time.monotonic()) if ok else None
            if ready and time.monotonic() - ready > 8:
                break
        else:
            raise RuntimeError("EKF readiness timeout")
        if self.rgb is None or self.depth is None or self.K is None:
            raise RuntimeError(
                "Camera image/depth/intrinsics not ready; refusing takeoff"
            )
        self.mode("LOITER")
        self.rc(throttle=1000)
        self.command(400, 1)
        self.phase("LOITER_TAKEOFF")
        end = time.monotonic() + 300
        while time.monotonic() < end:
            self.pump(0.05)
            lp = self.msgs["LOCAL_POSITION_NED"]
            height = -lp.z
            self.rc(
                throttle=(
                    1500
                    if height > SCENE["takeoff"] - .15
                    else max(1620, min(1800, 1620 + 150 * (SCENE["takeoff"] - height)))
                )
            )
            self.detect()
            self.record()
            if height > SCENE["takeoff"] - .15 and abs(lp.vz) < 0.18:
                break
        else:
            raise RuntimeError("LOITER takeoff timeout")
        self.phase("LOITER_APPROACH")
        end = time.monotonic() + 120
        while time.monotonic() < end:
            self.pump(0.05)
            lp = self.msgs["LOCAL_POSITION_NED"]
            control = 120 * (-1 - lp.y) - 160 * lp.vy
            roll = 1500 + max(
                -100,
                min(
                    100,
                    control
                    + (35 * math.copysign(1, control) if abs(control) > 8 else 0),
                ),
            )
            self.rc(roll=roll, throttle=1500)
            target = self.detect()
            self.record()
            if target and abs(target["east"]) < 0.2 and abs(lp.vy) < 0.15:
                break
        else:
            raise RuntimeError("Wire acquisition timeout")
        self.rc()
        self.mode("GUIDED")
        self.phase("GUIDED_VISUAL_DESCENT")
        guided_start = time.monotonic()
        end = time.monotonic() + 240
        touch_since = None
        last_print = 0
        while time.monotonic() < end:
            self.pump(0.05)
            lp = self.msgs["LOCAL_POSITION_NED"]
            target = self.detect()
            if inject_vision_loss and time.monotonic() - guided_start > 2:
                self.vision_disabled = True
            if not target or time.monotonic() - self.target_time > 0.4:
                self.velocity(0, 0, 0)
                self.record()
                if time.monotonic() - self.target_time > 3:
                    raise RuntimeError("Visual target lost")
                continue
            n, e = target["north"], target["east"]
            gap = target["down"] + 0.388
            vn = max(-0.25, min(0.25, 0.8 * n - 0.45 * lp.vx))
            ve = max(-0.25, min(0.25, 0.8 * e - 0.45 * lp.vy))
            descend = (
                min(0.18, max(0.035, 0.45 * gap))
                if math.hypot(n, e) < 0.055 and abs(target["yaw_error"]) < 0.07
                else 0
            )
            self.velocity(
                vn, ve, descend, max(-0.12, min(0.12, 0.6 * target["yaw_error"]))
            )
            self.record()
            if time.monotonic() - last_print > 2:
                print("TRACK", target, "gap", gap, flush=True)
                last_print = time.monotonic()
            if self.contact and gap < 0.035:
                touch_since = touch_since or time.monotonic()
                if time.monotonic() - touch_since > 0.4:
                    break
            else:
                touch_since = None
        else:
            raise RuntimeError("Visual descent timeout")
        self.phase("CONTACT_LAND")
        self.mode("LAND")
        end = time.monotonic() + 90
        while time.monotonic() < end:
            self.pump(0.1)
            self.record()
            hb = self.msgs.get("HEARTBEAT")
            if hb and not hb.base_mode & 128:
                break
        else:
            raise RuntimeError("Did not disarm on wire")
        self.phase("PERCHED_DISARMED")
        self.pump(4)
        if not self.contact or not self.truth or abs(self.truth["z"] - (SCENE["wire_height"]-.388)) > 0.03:
            raise RuntimeError("Disarmed but not supported on the wire")
        return dict(
            passed=True,
            last_target=self.target,
            contact=self.contact,
            evaluation=self.truth,
            local=self.msgs["LOCAL_POSITION_NED"].to_dict(),
            phases=self.transitions,
        )


def main():
    global OUT, AP, SCENE
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ardupilot", type=Path, default=AP)
    parser.add_argument("--scene", choices=SCENES, default="test")
    parser.add_argument(
        "--inject-vision-loss",
        action="store_true",
        help="Exercise the stop-descent and LAND abort path",
    )
    args = parser.parse_args()
    SCENE = SCENES[args.scene]
    AP = args.ardupilot.resolve()
    OUT = (
        Path.home() / ".ros/log" / ("perching_flight_" + time.strftime("%Y%m%d_%H%M%S"))
    )
    OUT.mkdir(parents=True, exist_ok=False)
    os.environ["GZ_PARTITION"] = "perching_flight_" + str(os.getpid())
    os.environ["ROS_DOMAIN_ID"] = "73"
    result = {
        "passed": False,
        "output": str(OUT),
        "partition": os.environ["GZ_PARTITION"],
        "world": SCENE["world"],
        "scene_config": SCENE,
        "injected_vision_loss": args.inject_vision_loss,
    }
    c = None

    def interrupted(signum, frame):
        raise RuntimeError("Interrupted; landing and cleaning up")

    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)
    try:
        for port, kind in [
            (9002, socket.SOCK_DGRAM),
            (9003, socket.SOCK_DGRAM),
            (5760, socket.SOCK_STREAM),
        ]:
            with socket.socket(socket.AF_INET, kind) as s:
                s.bind(("127.0.0.1", port))
        result["ardupilot_commit"] = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=AP, text=True
        ).strip()
        result["ardupilot_branch"] = subprocess.check_output(
            ["git", "branch", "--show-current"], cwd=AP, text=True
        ).strip()
        result["sitl_sha256"] = hashlib.sha256(
            (AP / "build/sitl/bin/arducopter").read_bytes()
        ).hexdigest()
        print("OUTPUT", OUT, flush=True)
        start(
            "gazebo",
            [
                "ros2",
                "launch",
                "uav_gazebo",
                "powerline_perching.launch",
                "world_name:=" + SCENE["file"],
            ],
        )
        deadline = time.monotonic() + 90
        while time.monotonic() < deadline:
            text = (OUT / "gazebo.log").read_text()
            if "resolved IMU topic" in text:
                services = subprocess.run(["gz", "service", "-l"], capture_output=True,
                                          text=True, timeout=5).stdout
                if "/gui/screenshot" in services:
                    break
            time.sleep(0.2)
        else:
            raise RuntimeError("Gazebo not ready")
        start(
            "camera_bridge",
            [
                "ros2",
                "run",
                "ros_gz_bridge",
                "parameter_bridge",
                "/powerline_perching/down_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
                "/powerline_perching/down_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
                "/powerline_perching/down_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            ],
        )
        defaults = (
            str(AP / "Tools/autotest/default_params/copter.parm")
            + ","
            + str(AP / "powerline_perching/mav.parm")
        )
        sitl_cwd = OUT / "sitl"
        sitl_cwd.mkdir()
        start(
            "sitl",
            [
                str(AP / "build/sitl/bin/arducopter"),
                "--model",
                "Gazebo",
                "--speedup",
                "1",
                "--defaults",
                defaults,
                "--sim-address=127.0.0.1",
                "--sim-port-in",
                "9003",
                "--sim-port-out",
                "9002",
                "-I0",
            ],
            sitl_cwd,
        )
        time.sleep(2)
        bags_before = set(WS.glob("rosbag2_*"))
        start(
            "rosbag",
            [
                "ros2",
                "bag",
                "record",
                "--storage",
                "mcap",
                "/powerline_perching/odometry",
                "/powerline_perching/contact",
                "/powerline_perching/imu",
                "/powerline_perching/vision/wire_offset_ned",
                "/powerline_perching/phase",
            ],
        )
        result["rosbags"] = [str(p) for p in set(WS.glob("rosbag2_*")) - bags_before]
        c = Companion()
        signal.signal(signal.SIGTERM, interrupted)
        signal.signal(signal.SIGINT, interrupted)
        result.update(c.run(args.inject_vision_loss))
        result["screenshots"] = screenshot("perched_disarmed")
    except Exception as e:
        result["passed"] = False
        result["error"] = repr(e)
        print("ERROR", repr(e), flush=True)
    finally:
        if c and c.mav:
            try:
                if c.msgs.get("HEARTBEAT") and c.msgs["HEARTBEAT"].base_mode & 128:
                    c.mode("LAND")
                    end = time.monotonic() + 350
                    while time.monotonic() < end:
                        c.pump(0.1)
                        if not c.msgs["HEARTBEAT"].base_mode & 128:
                            break
            except Exception as e:
                result["landing_cleanup_error"] = repr(e)
            result["final_armed"] = bool(
                c.msgs.get("HEARTBEAT") and c.msgs["HEARTBEAT"].base_mode & 128
            )
            result["phases"] = c.transitions
            c.log.close()
            c.node.destroy_node()
            rclpy.try_shutdown()
        result["cleanup"] = stop()
        if not all(result["cleanup"].values()):
            result["passed"] = False
            result["cleanup_error"] = "Owned process group still exists"
        if "bags_before" in locals():
            result["rosbags"] = [
                str(p) for p in set(WS.glob("rosbag2_*")) - bags_before
            ]
        result["processes"] = [
            dict(name=n, pid=p.pid, pgid=p.pid, returncode=p.returncode)
            for n, p, f in children
        ]
        (OUT / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result, indent=2), flush=True)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
