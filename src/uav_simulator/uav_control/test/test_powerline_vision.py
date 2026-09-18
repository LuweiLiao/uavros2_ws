"""Projection/sign/freshness checks without Gazebo or an autopilot connection."""

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import time
import unittest

import numpy as np

path = Path(__file__).resolve().parents[1] / "scripts/powerline_perching_sim.py"
spec = importlib.util.spec_from_file_location("perching", path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class VisionTests(unittest.TestCase):
    def fixture(self, offset=0.1, yaw=0):
        c = module.Companion.__new__(module.Companion)
        c.vision_disabled = False
        c.K = [400.0, 0.0, 320.0, 0.0, 400.0, 240.0, 0.0, 0.0, 1.0]
        c.rgb = np.zeros((480, 640, 3), dtype=np.uint8)
        c.depth = np.full((480, 640), np.inf, dtype=np.float32)
        # A line one metre below the camera, along the body X axis.
        col = int(320 + offset * 400)
        c.rgb[30:230, col - 1 : col + 2] = [245, 125, 20]
        c.depth[30:230, col - 1 : col + 2] = 1.0
        c.rgb_stamp = c.depth_stamp = 10.0
        c.image_wall = time.monotonic()
        c.msgs = {"ATTITUDE": SimpleNamespace(roll=0.0, pitch=0.0, yaw=yaw)}
        c.target_pub = SimpleNamespace(publish=lambda msg: None)
        c.annotations = []
        c.annotated_pub = SimpleNamespace(publish=c.annotations.append)
        return c

    def test_annotation_tracks_accepted_pixels_and_clears_on_loss(self):
        c = self.fixture()
        raw = c.rgb.copy()
        c.detect()
        msg = c.annotations[-1]
        marked = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(480, 640, 3)
        self.assertEqual(msg.header.stamp.sec, 10)
        self.assertTrue(np.array_equal(marked[100, 360], [30, 255, 90]))
        self.assertTrue(np.array_equal(c.rgb, raw))
        c.depth[:] = np.nan
        self.assertIsNone(c.detect())
        rejected = np.frombuffer(bytes(c.annotations[-1].data), dtype=np.uint8).reshape(480, 640, 3)
        self.assertTrue(np.array_equal(rejected[100, 360], raw[100, 360]))

    def test_camera_to_body_offset_and_wire_radius(self):
        target = self.fixture().detect()
        self.assertAlmostEqual(target["east"], 0.1, places=3)
        self.assertAlmostEqual(target["north"], 0.0, places=3)
        self.assertAlmostEqual(target["down"], 1.0 - 0.568 + 0.012, places=3)
        self.assertAlmostEqual(target["yaw_error"], 0.0, places=3)

    def test_ned_rotation(self):
        target = self.fixture(yaw=math.pi / 2).detect()
        self.assertAlmostEqual(target["north"], -0.1, places=3)
        self.assertAlmostEqual(target["east"], 0.0, places=3)

    def test_stale_frame_rejected(self):
        c = self.fixture()
        c.image_wall -= 1
        self.assertIsNone(c.detect())

    def test_unsynchronised_depth_rejected(self):
        c = self.fixture()
        c.depth_stamp -= 0.2
        self.assertIsNone(c.detect())

    def test_missing_line_rejected(self):
        c = self.fixture()
        c.rgb[:] = 0
        self.assertIsNone(c.detect())

    def test_invalid_depth_rejected(self):
        c = self.fixture()
        c.depth[:] = np.nan
        self.assertIsNone(c.detect())

    def test_gray_nearest_wire_rejects_lower_layer(self):
        c = self.fixture()
        c.detector = "gray"
        c.rgb[c.rgb[:, :, 0] > 0] = [165, 170, 175]
        c.rgb[30:230, 420:423] = [165, 170, 175]
        c.depth[30:230, 420:423] = 3.5
        target = c.detect()
        self.assertAlmostEqual(target["east"], .1, places=3)
        self.assertAlmostEqual(target["down"], .444, places=3)
        self.assertTrue((c.vision_pixels[0] < 400).all())

    def test_gray_surface_is_not_a_line(self):
        c = self.fixture()
        c.detector = "gray"
        c.rgb[:] = [160, 160, 160]
        c.depth[:] = 1
        self.assertIsNone(c.detect())

    def test_gray_rotor_reflection_is_rejected(self):
        c = self.fixture()
        c.detector = "gray"
        c.rgb[c.rgb[:, :, 0] > 0] = [165, 170, 175]
        # Front rotor at body Y=-0.37, body Z=+0.079; target wire remains at 1 m depth.
        c.rgb[100:230, 15:19] = [175, 175, 175]
        c.depth[100:230, 15:19] = .489
        target = c.detect()
        self.assertAlmostEqual(target["east"], .1, places=3)
        self.assertAlmostEqual(target["down"], .444, places=3)


if __name__ == "__main__":
    unittest.main()
