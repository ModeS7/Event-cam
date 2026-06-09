#!/usr/bin/env python3
"""Guided intrinsic calibration for the EVK4 — a self-contained OpenCV tool.

A window shows the live rendered image with the detected checkerboard. Move
the board around; the tool AUTO-CAPTURES views that improve coverage (the
X / Y / Size / Skew bars fill up). When coverage is good, press `c` to run
OpenCV calibration and write a camera_info YAML that evk4_bringup's
`calibration_url` consumes.

    # 1. camera + renderer (sharp mode is easiest to detect on)
    ros2 launch evk4_bringup evk4.launch.py display_type:=sharp
    # 2. this tool (8x6 = inner corners, 0.025 = square size in metres)
    ros2 run evk4_calibration calibrate --ros-args \
        -p board_size:=8x6 -p square_size:=0.025 \
        -r image_raw:=/event_camera/image_raw

Keys: SPACE force-capture · c calibrate · q quit.

An event camera only "sees" change, so the board shows up best as a FLICKERING
checkerboard on a screen, or a printed board moved briskly. Needs a display
(run on a desktop, or over X-forwarding / VNC).
"""

import math

import cv2
import numpy as np
import yaml

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

# Coverage targets: how wide the spread of each parameter should get before
# that bar is "full". Tuned for a foolproof-but-not-tedious sweep.
_TARGET_SPAN = {'x': 0.6, 'y': 0.6, 'size': 0.35, 'skew': 0.3}
_CAPTURE_DIST = 0.12   # min param-space distance for a new auto-capture
_READY_FRAC = 0.7      # every bar must reach this fraction
_SUBPIX = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
_FIND_FLAGS = (cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE |
               cv2.CALIB_CB_FAST_CHECK)


def _board_params(corners, cols, rows, w, h):
    """Reduce a detection to (x, y, size, skew) in [0,1] for coverage tracking.

    Mirrors what camera_calibration uses: board centre position, how much of
    the frame it fills, and how tilted it is away from fronto-parallel.
    """
    pts = corners.reshape(-1, 2)
    x = float(pts[:, 0].mean()) / w
    y = float(pts[:, 1].mean()) / h
    bw = pts[:, 0].max() - pts[:, 0].min()
    bh = pts[:, 1].max() - pts[:, 1].min()
    size = math.sqrt(max(bw * bh, 1.0) / (w * h))
    # skew from the angle at the top-left corner (90 deg = fronto-parallel)
    tl, tr, bl = pts[0], pts[cols - 1], pts[(rows - 1) * cols]
    a, b = tr - tl, bl - tl
    cos = abs(np.dot(a, b)) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-9)
    skew = min(cos / math.cos(math.radians(30)), 1.0)
    return np.array([x, y, size, skew])


class Calibrator(Node):
    """Collect well-distributed checkerboard views and calibrate on demand."""

    def __init__(self):
        super().__init__('calibrate')
        cols, rows = (int(v) for v in
                      self.declare_parameter('board_size', '8x6').value.split('x'))
        self._grid = (cols, rows)
        self._square = self.declare_parameter('square_size', 0.025).value
        self._min_samples = self.declare_parameter('min_samples', 20).value
        self._camera_name = self.declare_parameter('camera_name', 'event_camera').value
        self._output = self.declare_parameter('output', 'event_camera.yaml').value

        self._bridge = CvBridge()
        self._latest = None          # most recent frame (bgr)
        self._params = []            # per-sample [x,y,size,skew]
        self._imgpoints = []         # per-sample refined corners
        self._size = None            # (w, h)
        self._last_capture_t = 0.0
        self.create_subscription(Image, 'image_raw', self._on_image, 10)
        self.get_logger().info(
            f'calibrate: board {cols}x{rows}, square {self._square} m, '
            f'output {self._output}')

    def _on_image(self, msg):
        self._latest = self._bridge.imgmsg_to_cv2(msg, 'bgr8')

    def tick(self):
        """Detect, auto-capture, and draw the UI. Runs on the main thread."""
        if self._latest is None:
            return
        frame = self._latest
        h, w = frame.shape[:2]
        self._size = (w, h)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, self._grid, _FIND_FLAGS)
        view = frame.copy()
        if found:
            corners = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), _SUBPIX)
            cv2.drawChessboardCorners(view, self._grid, corners, found)
            self._maybe_capture(_board_params(corners, *self._grid, w, h), corners)
        self._draw_overlay(view, found)
        cv2.imshow('evk4 calibrate', view)

    def _maybe_capture(self, params, corners, force=False):
        now = self.get_clock().now().nanoseconds * 1e-9
        if not force:
            if now - self._last_capture_t < 0.3:
                return
            if self._params and min(
                    np.linalg.norm(params - p) for p in self._params) < _CAPTURE_DIST:
                return
        self._params.append(params)
        self._imgpoints.append(corners)
        self._last_capture_t = now
        self.get_logger().info(f'captured view {len(self._params)}')

    def force_capture(self):
        """Capture the current detection regardless of coverage (SPACE)."""
        if self._latest is None:
            return
        gray = cv2.cvtColor(self._latest, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, self._grid, _FIND_FLAGS)
        if found:
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), _SUBPIX)
            h, w = self._latest.shape[:2]
            self._maybe_capture(_board_params(corners, *self._grid, w, h),
                                corners, force=True)

    def _coverage(self):
        """Per-parameter coverage fraction in [0,1] from captured samples."""
        if len(self._params) < 2:
            return {k: 0.0 for k in _TARGET_SPAN}
        arr = np.array(self._params)
        spans = arr.max(axis=0) - arr.min(axis=0)
        keys = ['x', 'y', 'size', 'skew']
        return {k: min(spans[i] / _TARGET_SPAN[k], 1.0) for i, k in enumerate(keys)}

    def ready(self):
        cov = self._coverage()
        return (len(self._params) >= self._min_samples and
                all(v >= _READY_FRAC for v in cov.values()))

    def _draw_overlay(self, view, found):
        cov = self._coverage()
        y0 = 24
        for i, (k, v) in enumerate(cov.items()):
            yb = y0 + i * 22
            cv2.rectangle(view, (10, yb - 12), (210, yb), (60, 60, 60), -1)
            color = (0, 200, 0) if v >= _READY_FRAC else (0, 165, 255)
            cv2.rectangle(view, (10, yb - 12), (10 + int(200 * v), yb), color, -1)
            cv2.putText(view, f'{k:5s}', (216, yb - 1),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        status = (f'samples {len(self._params)}/{self._min_samples}  '
                  + ('READY - press C' if self.ready()
                     else ('board OK' if found else 'show the board')))
        cv2.putText(view, status, (10, view.shape[0] - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        cv2.putText(view, 'SPACE capture  C calibrate  Q quit', (10, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

    def calibrate(self):
        """Run OpenCV calibration and write the camera_info YAML. Returns ok."""
        if len(self._params) < self._min_samples:
            self.get_logger().warning(
                f'only {len(self._params)} views; keep going (need '
                f'{self._min_samples}+ with good coverage)')
            return False
        cols, rows = self._grid
        objp = np.zeros((cols * rows, 3), np.float32)
        objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * self._square
        rms, k, d, _, _ = cv2.calibrateCamera(
            [objp] * len(self._imgpoints), self._imgpoints, self._size, None, None)
        self.get_logger().info(f'calibration RMS reprojection error: {rms:.3f} px')
        self._write_yaml(k, d)
        return True

    def _write_yaml(self, k, d):
        w, h = self._size
        dist = list(np.asarray(d).flatten()[:5])
        dist += [0.0] * (5 - len(dist))
        proj = [k[0, 0], 0.0, k[0, 2], 0.0,
                0.0, k[1, 1], k[1, 2], 0.0,
                0.0, 0.0, 1.0, 0.0]
        calib = {
            'image_width': w, 'image_height': h,
            'camera_name': self._camera_name,
            'camera_matrix': {'rows': 3, 'cols': 3,
                              'data': [float(x) for x in k.flatten()]},
            'distortion_model': 'plumb_bob',
            'distortion_coefficients': {'rows': 1, 'cols': 5,
                                        'data': [float(x) for x in dist]},
            'rectification_matrix': {'rows': 3, 'cols': 3,
                                     'data': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                              0.0, 0.0, 1.0]},
            'projection_matrix': {'rows': 3, 'cols': 4,
                                  'data': [float(x) for x in proj]},
        }
        with open(self._output, 'w') as f:
            yaml.safe_dump(calib, f, default_flow_style=None, sort_keys=False)
        self.get_logger().info(
            f'wrote {self._output} — relaunch with '
            f'calibration_url:=$(pwd)/{self._output}')


def main(args=None):
    """Run the interactive calibration loop until the user quits."""
    rclpy.init(args=args)
    node = Calibrator()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.005)
            node.tick()
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            if key == ord(' '):
                node.force_capture()
            elif key == ord('c') and node.calibrate():
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
