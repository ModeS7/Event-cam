#!/usr/bin/env python3
"""Guided intrinsic calibration for the EVK4 — a headless ROS 2 node.

Uses a blinking ASYMMETRIC CIRCLE GRID (open docs/circle_grid.html fullscreen
on a monitor). Circle centers are centroids of many events, which makes them
robust to the speckle and soft edges of event-rendered images — the reason
every modern event-camera calibrator (E-Calib, eKalibr) uses circles rather
than checkerboards.

The node has no window of its own: it publishes its annotated view (detected
grid + coverage bars) on `~/overlay`, watched with the same viewer used
everywhere else in this repo. It AUTO-CAPTURES views that improve coverage,
and once coverage is good it calibrates by itself, writes a camera_info YAML
that evk4_bringup's `calibration_url` consumes, logs the RMS, and exits.

    # terminal 1 -- camera + renderer, with your tuned params (tuning.md)
    ros2 launch evk4_bringup evk4.launch.py params_file:=$HOME/my_params.yaml
    # terminal 2 -- the calibrator (finishes and exits by itself)
    ros2 run evk4_calibration calibrate --ros-args \\
        -r image_raw:=/event_camera/image_raw
    # terminal 3 -- watch progress (any machine with ROS)
    ros2 run rqt_image_view rqt_image_view /calibrate/overlay

Move the camera to cover the field of view: near/far, all four image
corners, tilted. Ctrl+C aborts without writing anything.
"""

import math
import threading
import time

import cv2
import numpy as np
import yaml

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

# Detection runs beside the camera pipeline, often on a small board; without
# a cap OpenCV parallelizes its calls across every core and the calibrator
# starves the renderer and viewer.
cv2.setNumThreads(2)

# Coverage targets: how wide the spread of each parameter should get before
# that bar is "full". Tuned for a foolproof-but-not-tedious sweep.
_TARGET_SPAN = {'x': 0.6, 'y': 0.6, 'size': 0.35, 'skew': 0.3}
_CAPTURE_DIST = 0.12   # min param-space distance for a new auto-capture
_READY_FRAC = 0.7      # every bar must reach this fraction


def _make_blob_detector():
    """Blob detector tuned for event-rendered dots: bright, ragged blobs.

    The blurred polarity-contrast image is nearly binary (bright events on
    black), so 2-3 threshold passes suffice — the default 17 make detection
    ~6x more expensive for nothing.
    """
    p = cv2.SimpleBlobDetector_Params()
    p.filterByColor = True
    p.blobColor = 255              # dots are bright on the contrast image
    p.minThreshold = 40
    p.maxThreshold = 121
    p.thresholdStep = 40
    p.filterByArea = True
    p.minArea = 10                 # detection runs at half resolution
    p.maxArea = 10000
    p.filterByCircularity = False  # event blobs have ragged outlines
    p.filterByConvexity = False
    p.filterByInertia = False
    return cv2.SimpleBlobDetector_create(p)


def _refine_centers(gray, centers):
    """Refine scaled-up grid centers with local intensity centroids.

    A dot's center IS the centroid of its (blurred) event blob, so a
    windowed centroid at full resolution recovers the precision of a
    full-resolution search at a fraction of its cost. The window is sized
    from the grid's own spacing, so near and far boards both work.
    """
    pts = centers.reshape(-1, 2)
    d = pts[:, None, :] - pts[None, :, :]
    dist = np.sqrt((d ** 2).sum(-1))
    np.fill_diagonal(dist, np.inf)
    nnd = float(np.median(dist.min(axis=1)))   # nearest-neighbor pitch
    win = int(np.clip(0.35 * nnd, 6, 40))
    h, w = gray.shape
    out = []
    for x, y in pts:
        x0, y0 = int(round(x)), int(round(y))
        x1, x2 = max(0, x0 - win), min(w, x0 + win + 1)
        y1, y2 = max(0, y0 - win), min(h, y0 + win + 1)
        patch = gray[y1:y2, x1:x2].astype(np.float32)
        m = patch.sum()
        if m < 1e-3:
            out.append((float(x), float(y)))
            continue
        ys, xs = np.mgrid[y1:y2, x1:x2]
        out.append((float((xs * patch).sum() / m),
                    float((ys * patch).sum() / m)))
    return np.asarray(out, np.float32).reshape(-1, 1, 2)


def _grid_params(centers, cols, rows, w, h):
    """Reduce a detection to (x, y, size, skew) in [0,1] for coverage tracking.

    Mirrors what camera_calibration uses: grid centre position, how much of
    the frame it fills, and how tilted it is away from fronto-parallel.
    """
    pts = centers.reshape(-1, 2)
    x = float(pts[:, 0].mean()) / w
    y = float(pts[:, 1].mean()) / h
    bw = pts[:, 0].max() - pts[:, 0].min()
    bh = pts[:, 1].max() - pts[:, 1].min()
    size = math.sqrt(max(bw * bh, 1.0) / (w * h))
    # skew from the angle at the first corner (90 deg = fronto-parallel)
    tl, tr, bl = pts[0], pts[cols - 1], pts[(rows - 1) * cols]
    a, b = tr - tl, bl - tl
    cos = abs(np.dot(a, b)) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-9)
    skew = min(cos / math.cos(math.radians(30)), 1.0)
    return np.array([x, y, size, skew])


class Calibrator(Node):
    """Collect well-distributed circle-grid views, then calibrate and exit."""

    def __init__(self):
        super().__init__('calibrate')
        cols, rows = (int(v) for v in
                      self.declare_parameter('grid_size', '5x17').value.split('x'))
        self._grid = (cols, rows)
        # OpenCV asymmetric-grid convention: the vertical row pitch; the
        # horizontal pitch within a row is twice this. Only affects absolute
        # scale (extrinsics), not the intrinsics used for rectification.
        self._spacing = self.declare_parameter('spacing', 0.02).value
        self._min_samples = self.declare_parameter('min_samples', 20).value
        self._camera_name = self.declare_parameter('camera_name', 'event_camera').value
        self._output = self.declare_parameter('output', 'event_camera.yaml').value
        # Log a per-cycle stage/latency breakdown (performance debugging).
        self._debug_timing = self.declare_parameter('debug_timing', False).value

        self._bridge = CvBridge()
        self._blob = _make_blob_detector()
        self._latest = None          # most recent (frame bgr, header)
        self._frame_seq = 0          # bumped per received frame
        self._params = []            # per-sample [x,y,size,skew]
        self._imgpoints = []         # per-sample circle centers
        self._size = None            # (w, h)
        self._last_capture_t = 0.0
        self._done = False
        # Detection runs on a worker thread so a slow search can never
        # block the node's callbacks.
        self._det_lock = threading.Lock()
        self._det = (False, None)    # latest (found, centers)
        self._running = True
        self._pub = self.create_publisher(Image, '~/overlay', 10)
        self.create_subscription(Image, 'image_raw', self._on_image, 10)
        self._worker = threading.Thread(target=self._detect_loop, daemon=True)
        self._worker.start()
        self.get_logger().info(
            f'calibrate: asymmetric circle grid {cols}x{rows}, '
            f'spacing {self._spacing} m, output {self._output} — watch '
            f'progress on {self.get_name()}/overlay')

    # ----------------------------------------------------------------- input
    def _on_image(self, msg):
        frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        self._latest = (frame, msg.header)
        self._size = (msg.width, msg.height)
        self._frame_seq += 1

    @staticmethod
    def _event_contrast(frame):
        """Event activity as a bright-on-dark image, polarity-independent.

        The renderer colors ON events blue and OFF events red; |B - R| is
        bright wherever events happened, whatever their polarity — so the
        blinking dots are bright blobs in both blink phases.
        """
        return cv2.absdiff(frame[:, :, 0], frame[:, :, 2])

    # ------------------------------------------------------------- detection
    def _detect_loop(self):
        """Worker: find the circle grid in each NEW frame, auto-capture.

        Empty frames (between blinks) are decided in microseconds and pass
        straight through to the overlay, so the view tracks the raw stream;
        only frames with activity pay for a half-resolution grid search,
        and a hit is refined by full-resolution local centroids instead of
        a second search. Finishes the calibration once coverage is good.
        """
        seen = -1
        n_dots = self._grid[0] * self._grid[1]
        flags = cv2.CALIB_CB_ASYMMETRIC_GRID | cv2.CALIB_CB_CLUSTERING
        while self._running:
            if self._latest is None or self._frame_seq == seen:
                time.sleep(0.005)
                continue
            seen = self._frame_seq
            frame, header = self._latest
            t0 = time.perf_counter()
            tl = {'blobs': -1, 'blob': 0.0, 'grid': 0.0, 'refine': 0.0,
                  'publish': 0.0}
            contrast = self._event_contrast(frame)
            found, centers = False, None
            if cv2.mean(contrast)[0] > 0.5:   # any real activity?
                gray = cv2.GaussianBlur(contrast, (9, 9), 0)
                small = cv2.resize(gray, None, fx=0.5, fy=0.5,
                                   interpolation=cv2.INTER_AREA)
                t1 = time.perf_counter()
                blobs = self._blob.detect(small)
                tl['blob'] = time.perf_counter() - t1
                tl['blobs'] = len(blobs)
                # Clutter guard: a frame with far more (or fewer) blobs
                # than dots cannot yield a clean grid, and search cost
                # explodes.
                if n_dots // 2 <= len(blobs) <= 4 * n_dots:
                    t1 = time.perf_counter()
                    found, centers_s = cv2.findCirclesGrid(
                        small, self._grid, flags=flags,
                        blobDetector=self._blob)
                    tl['grid'] = time.perf_counter() - t1
                    if found:
                        t1 = time.perf_counter()
                        centers = _refine_centers(gray, centers_s * 2.0)
                        tl['refine'] = time.perf_counter() - t1
            if found:
                h, w = frame.shape[:2]
                self._maybe_capture(
                    _grid_params(centers, *self._grid, w, h), centers)
            with self._det_lock:
                self._det = (bool(found), centers if found else None)
            # Annotated view, drawn on the exact frame that was analyzed so
            # markers always sit on the dots they were found in. Lazy:
            # composed only while someone watches.
            if self._pub.get_subscription_count() > 0:
                t1 = time.perf_counter()
                # The overlay is a progress view, published at HALF
                # resolution: a Python node serializing full-resolution
                # images costs ~10x what the C++ renderer pays and the
                # viewer starts dropping frames. Calibration itself uses
                # full-resolution coordinates regardless.
                view = cv2.resize(frame, None, fx=0.5, fy=0.5,
                                  interpolation=cv2.INTER_AREA)
                if found:
                    cv2.drawChessboardCorners(
                        view, self._grid, centers * 0.5, True)
                self._draw_overlay(view, bool(found))
                out = self._bridge.cv2_to_imgmsg(view, 'bgr8')
                out.header = header
                self._pub.publish(out)
                tl['publish'] = time.perf_counter() - t1
            if self._debug_timing:
                age = (self.get_clock().now().nanoseconds
                       - header.stamp.sec * 10**9
                       - header.stamp.nanosec) * 1e-9
                self.get_logger().info(
                    f'cycle {(time.perf_counter() - t0) * 1000:6.0f} ms | '
                    f'blobs {tl["blobs"]:4d} | blob {tl["blob"] * 1000:5.0f} '
                    f'grid {tl["grid"] * 1000:5.0f} '
                    f'refine {tl["refine"] * 1000:4.0f} '
                    f'publish {tl["publish"] * 1000:4.0f} ms | '
                    f'frame age {age:5.2f} s')
            if not self._done and self.ready():
                self._done = True
                self._finish()
                return

    def _maybe_capture(self, params, centers):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_capture_t < 0.3:
            return
        if self._params and min(
                np.linalg.norm(params - p) for p in self._params) < _CAPTURE_DIST:
            return
        self._params.append(params)
        self._imgpoints.append(centers)
        self._last_capture_t = now
        self.get_logger().info(f'captured view {len(self._params)}')

    def stop(self):
        self._running = False
        self._worker.join(timeout=2.0)

    # -------------------------------------------------------------- coverage
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
                  + ('READY - calibrating' if self.ready()
                     else ('grid OK' if found else 'show the blinking circle grid')))
        cv2.putText(view, status, (10, view.shape[0] - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        cv2.putText(view, 'auto-captures distinct views; finishes by itself',
                    (10, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

    # ----------------------------------------------------------- calibration
    def _finish(self):
        """Run OpenCV calibration, write the YAML, log the RMS, and exit."""
        cols, rows = self._grid
        # Asymmetric grid object points (OpenCV convention): row pitch =
        # spacing, odd rows offset by spacing, columns 2*spacing apart.
        objp = np.zeros((cols * rows, 3), np.float32)
        for i in range(rows):
            for j in range(cols):
                objp[i * cols + j] = ((2 * j + i % 2) * self._spacing,
                                      i * self._spacing, 0)
        self.get_logger().info(
            f'coverage complete — calibrating on {len(self._imgpoints)} views '
            '(takes a moment)...')
        # FIX_K3: with a narrow-FOV lens the 6th-order radial term is
        # unconstrained by realistic coverage and overfits wildly.
        rms, k, d, _, _ = cv2.calibrateCamera(
            [objp] * len(self._imgpoints), self._imgpoints, self._size, None, None,
            flags=cv2.CALIB_FIX_K3)
        self.get_logger().info(f'calibration RMS reprojection error: {rms:.3f} px')
        self._write_yaml(k, d, rms)
        rclpy.shutdown()

    def _write_yaml(self, k, d, rms):
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
            f.write(f'# RMS reprojection error: {rms:.3f} px '
                    f'({len(self._imgpoints)} views)\n')
            yaml.safe_dump(calib, f, default_flow_style=None, sort_keys=False)
        self.get_logger().info(
            f'wrote {self._output} — relaunch with '
            f'calibration_url:=$(pwd)/{self._output}')


def main(args=None):
    """Run until calibration completes (or Ctrl+C aborts)."""
    rclpy.init(args=args)
    node = Calibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
