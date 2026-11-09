#!/usr/bin/env python3
"""Score stereo depth (cf_drone stereo_disparity) against Gazebo ground-truth depth, binned by true range.

The ground-truth depth camera shares the left camera's pose and intrinsics, so the images are pixel-aligned.
Every 10 s writes OUT.json and OUT.png: per range bin, share of pixels with a stereo depth, median and p90
absolute error; plus phantom depth (stereo depth where the truth is sky). At the frames in SNAPSHOTS it also
writes OUT_frame<N>.png: left image, true depth, stereo depth and error side by side.
With dump_every N > 0, every N-th frame is saved raw to OUT_frames/<N>.npz (left, right, truth, stereo, fx) for
scoring other stereo methods offline on the same images.
"""
import json
import os

import matplotlib
import numpy as np
import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

BINS = [0, 5, 10, 20, 30, 50, 75, 100, 150, 250, 1000]  # [m]
KEEP = 100000  # error samples kept per bin
SNAPSHOTS = (150, 600)  # frame numbers (10 Hz: 15 s and 60 s after start)


def to_array(msg):
    return np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)


class StereoEval(Node):
    def __init__(self):
        super().__init__('stereo_eval')
        self.out = self.declare_parameter('out', 'stereo_eval').value
        stereo = Subscriber(self, Image, self.declare_parameter('stereo_topic', '/depth').value)
        truth = Subscriber(self, Image, self.declare_parameter('truth_topic', '/stereo/depth/image').value)
        left = Subscriber(self, Image, self.declare_parameter('left_topic', '/stereo/left/image').value)
        right = Subscriber(self, Image, self.declare_parameter('right_topic', '/stereo/right/image').value)
        self.dump_every = self.declare_parameter('dump_every', 0).value
        self.fx = None
        self.create_subscription(CameraInfo, '/stereo/left/camera_info', lambda m: setattr(self, 'fx', m.k[0]), 1)
        self.sync = ApproximateTimeSynchronizer([stereo, truth, left, right], queue_size=10, slop=0.05)
        self.sync.registerCallback(self.on_pair)
        n = len(BINS) - 1
        self.pixels, self.valid = np.zeros(n, int), np.zeros(n, int)
        self.errors = [np.empty(0) for _ in range(n)]
        self.sky_pixels = self.phantoms = self.frames = 0
        self.rng = np.random.default_rng(0)
        self.create_timer(10.0, self.write)

    def on_pair(self, stereo_msg, truth_msg, left_msg, right_msg):
        stereo, truth = to_array(stereo_msg), to_array(truth_msg)
        if stereo.shape != truth.shape:
            self.get_logger().error(f'size mismatch {stereo.shape} vs {truth.shape}', once=True)
            return
        self.frames += 1
        left = np.frombuffer(left_msg.data, np.uint8).reshape(left_msg.height, left_msg.width, 3)
        if self.frames in SNAPSHOTS:
            self.snapshot(stereo, truth, left)
        if self.dump_every and self.frames % self.dump_every == 0 and self.fx:
            os.makedirs(self.out + '_frames', exist_ok=True)
            right = np.frombuffer(right_msg.data, np.uint8).reshape(right_msg.height, right_msg.width, 3)
            np.savez_compressed(f'{self.out}_frames/{self.frames:05d}.npz', left=left, right=right, truth=truth,
                                stereo=stereo, fx=self.fx)
        sky = ~np.isfinite(truth) | (truth <= 0)
        self.sky_pixels += int(sky.sum())
        self.phantoms += int((sky & np.isfinite(stereo)).sum())
        bin_index = np.digitize(truth, BINS) - 1
        for i in range(len(BINS) - 1):
            in_bin = ~sky & (bin_index == i)
            has = in_bin & np.isfinite(stereo)
            self.pixels[i] += int(in_bin.sum())
            self.valid[i] += int(has.sum())
            err = np.abs(stereo[has] - truth[has])
            if len(err):
                self.errors[i] = np.concatenate([self.errors[i], err])
                if len(self.errors[i]) > KEEP:
                    self.errors[i] = self.rng.choice(self.errors[i], KEEP, replace=False)

    def write(self):
        if not self.frames:
            return
        rows = []
        for i in range(len(BINS) - 1):
            e = self.errors[i]
            rows.append({'range_m': [BINS[i], BINS[i + 1]], 'pixels': int(self.pixels[i]),
                         'valid_pct': 100.0 * self.valid[i] / self.pixels[i] if self.pixels[i] else None,
                         'median_err_m': float(np.median(e)) if len(e) else None,
                         'p90_err_m': float(np.percentile(e, 90)) if len(e) else None})
        result = {'frames': self.frames, 'bins': rows,
                  'phantom_pct_of_sky': 100.0 * self.phantoms / self.sky_pixels if self.sky_pixels else None}
        with open(self.out + '.json', 'w') as f:
            json.dump(result, f, indent=1)
        self.plot(rows)

    def snapshot(self, stereo, truth, left):
        error = np.abs(stereo - truth)
        fig, axes = plt.subplots(1, 4, figsize=(20, 3.6))
        panels = [(left, None, 'left camera'), (truth, 'viridis', 'true depth [m]'),
                  (stereo, 'viridis', 'stereo depth [m] (white: no depth)'), (error, 'magma', 'depth error [m]')]
        for ax, (image, cmap, title) in zip(axes, panels):
            limits = {'magma': (0, 30), 'viridis': (0, 250)}.get(cmap, (None, None))
            shown = ax.imshow(image, cmap=cmap, vmin=limits[0], vmax=limits[1])
            if cmap:
                fig.colorbar(shown, ax=ax, fraction=0.03)
            ax.set_title(title)
            ax.axis('off')
        fig.savefig(f'{self.out}_frame{self.frames}.png', dpi=100, bbox_inches='tight')
        plt.close(fig)

    def plot(self, rows):
        used = [r for r in rows if r['pixels']]
        centers = [sum(r['range_m']) / 2 for r in used]
        fig, (a, b) = plt.subplots(1, 2, figsize=(11, 4))
        a.plot(centers, [r['valid_pct'] for r in used], 'o-')
        a.set_xlabel('true range [m]')
        a.set_ylabel('pixels with stereo depth [%]')
        b.plot(centers, [r['median_err_m'] for r in used], 'o-', label='median')
        b.plot(centers, [r['p90_err_m'] for r in used], 's--', label='p90')
        b.set_xlabel('true range [m]')
        b.set_ylabel('absolute depth error [m]')
        b.set_yscale('log')
        b.legend()
        fig.suptitle(f'{self.frames} frames')
        fig.savefig(self.out + '.png', dpi=110, bbox_inches='tight')
        plt.close(fig)


def main():
    rclpy.init()
    node = StereoEval()
    try:
        rclpy.spin(node)
    finally:
        node.write()


if __name__ == '__main__':
    main()
