"""Score stereo methods on the raw frames stereo_eval.py saves (dump_every), against Gazebo's true depth.

Methods: sgm = cf_drone stereo_disparity (VPI SGM with its confidence/speckle filters), saved with the frames;
ffs = Fast-FoundationStereo (NVlabs, CVPR 2026) run here at full resolution and at half (its README advises
width < 1000), each also with "+LR", a left-right consistency check (the model has no confidence output).
Pixels are split per frame: 'pillar' = true 3D point > 3 m off the ground plane (RANSAC fit to the true depth),
'ground' = the rest. False obstacles: share of pixels with no true surface within FAR (sky, or ground past the depth
camera's ~900 m clip) that get a stereo depth under NEAR, i.e. a phantom inside the planning horizon.
Run with the Fast-FoundationStereo venv (tools/install_ffs.sh), on frames saved by run_experiment.sh with DUMP_EVERY:
  external/ffs-venv/bin/python tools/stereo_methods.py --out DIR RUN_DIR:BASELINE [RUN_DIR:BASELINE ...]
"""
import argparse
import glob
import json
import os
import sys
import time

import cv2
import matplotlib
import numpy as np
import torch

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

BINS = [0, 50, 75, 100, 150, 250, 400, 1000]  # true range [m]
PILLAR_OFF_GROUND = 3.0  # [m]
FAR, NEAR = 300.0, 150.0  # [m] false obstacle: true surface beyond FAR (or none), stereo depth under NEAR
FFS_SCALES = (1.0, 0.5)
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FFS_REPO = os.path.join(REPO, 'external/Fast-FoundationStereo')
FFS_MODEL = os.path.join(REPO, 'external/ffs-weights/c-ffs/model_best_bp2_serialize.pth')  # tools/install_ffs.sh


def ground_plane_mask(truth, fx, rng):
    """True where the true 3D point is more than PILLAR_OFF_GROUND from the dominant (ground) plane."""
    h, w = truth.shape
    v, u = np.mgrid[0:h, 0:w]
    with np.errstate(invalid='ignore'):  # sky is inf
        points = np.stack([(u - w / 2) / fx * truth, (v - h / 2) / fx * truth, truth], -1)
    finite = np.isfinite(truth) & (truth > 0)
    sample = points[finite][rng.choice(finite.sum(), min(4000, finite.sum()), replace=False)]
    best, best_count = None, -1
    for _ in range(200):
        a, b, c = sample[rng.choice(len(sample), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-6:
            continue
        n /= np.linalg.norm(n)
        count = int((np.abs((sample - a) @ n) < 1.0).sum())
        if count > best_count:
            best, best_count = (n, a), count
    n, a = best
    with np.errstate(invalid='ignore'):  # sky points are inf
        distance = np.abs((points - a) @ n)
    return finite & (distance > PILLAR_OFF_GROUND)


class FastFoundationStereo:
    def __init__(self, repo, model_file):
        sys.path.insert(0, repo)
        from core.utils.utils import InputPadder  # noqa: E402 (repo on the path first)
        self.padder_class = InputPadder
        torch.autograd.set_grad_enabled(False)
        self.model = torch.load(model_file, map_location='cpu', weights_only=False)
        self.model.args.valid_iters = 8
        self.model.args.max_disp = 192
        if 'normalize' not in self.model.args:  # the Hugging Face checkpoint lacks it; the repo's exporter defaults True
            self.model.args.normalize = True
        self.model.cuda().eval()

    def disparity(self, left, right, scale):
        """Left-view disparity [px] at full resolution, and the forward time [s]."""
        h, w = left.shape[:2]
        small = [cv2.resize(image, None, fx=scale, fy=scale) if scale != 1 else image for image in (left, right)]
        tensors = [torch.as_tensor(image).cuda().float()[None].permute(0, 3, 1, 2) for image in small]
        padder = self.padder_class(tensors[0].shape, divis_by=32, force_square=False)
        tensors = padder.pad(*tensors)
        torch.cuda.synchronize()
        started = time.monotonic()
        with torch.amp.autocast('cuda', enabled=True, dtype=torch.float16):
            disp = self.model.forward(*tensors, iters=8, test_mode=True, optimize_build_volume='pytorch1')
        torch.cuda.synchronize()
        seconds = time.monotonic() - started
        disp = padder.unpad(disp.float()).cpu().numpy().reshape(small[0].shape[:2])
        if scale != 1:
            disp = cv2.resize(disp, (w, h), interpolation=cv2.INTER_LINEAR) / scale
        return disp, seconds

    def depths(self, left, right, fx, baseline, scale):
        """Depth [m] (NaN: invalid) without and with a left-right check, and the forward times [s].
        The check matches the mirrored pair too (the right view's disparity) and keeps pixels where both agree
        within 1 px at the network's resolution; the model has no confidence output of its own."""
        disp, seconds = self.disparity(left, right, scale)
        disp_right, more = self.disparity(np.ascontiguousarray(right[:, ::-1]), np.ascontiguousarray(left[:, ::-1]), scale)
        disp_right = disp_right[:, ::-1]
        h, w = disp.shape
        u = np.arange(w)[None, :]
        invalid = (disp <= 0) | (u - disp < 0)  # no match / not visible in the right image
        matched = np.clip(np.round(u - disp).astype(int), 0, w - 1)
        inconsistent = np.abs(disp - disp_right[np.arange(h)[:, None], matched]) > 1.0 / scale
        with np.errstate(divide='ignore'):
            plain = np.where(invalid, np.nan, fx * baseline / disp)
        checked = np.where(inconsistent, np.nan, plain)
        return (plain, seconds), (checked, seconds + more)


def score(depth, truth, masks):
    out = {}
    for name, mask in masks.items():
        if name == 'far':
            out['false_obstacle_pct'] = 100.0 * float((depth[mask] < NEAR).mean()) if mask.any() else None
            continue
        rows = []
        for lo, hi in zip(BINS, BINS[1:]):
            m = mask & (truth >= lo) & (truth < hi)
            has = m & np.isfinite(depth)
            err = np.abs(depth[has] - truth[has])
            rows.append({'pixels': int(m.sum()), 'with_depth': int(has.sum()), 'err': err})
        out[name] = rows
    return out


def merge(results):
    """Per-frame scores -> per bin: share with depth, median / p90 abs error."""
    merged = {}
    for name in ('pillar', 'ground'):
        rows = []
        for i, (lo, hi) in enumerate(zip(BINS, BINS[1:])):
            pixels = sum(r[name][i]['pixels'] for r in results)
            with_depth = sum(r[name][i]['with_depth'] for r in results)
            err = np.concatenate([r[name][i]['err'] for r in results])
            rows.append({'range_m': [lo, hi], 'pixels': pixels,
                         'with_depth_pct': 100.0 * with_depth / pixels if pixels else None,
                         'median_err_m': float(np.median(err)) if len(err) else None,
                         'p90_err_m': float(np.percentile(err, 90)) if len(err) else None})
        merged[name] = rows
    false = [r['false_obstacle_pct'] for r in results if r['false_obstacle_pct'] is not None]
    merged['false_obstacle_pct'] = float(np.mean(false)) if false else None
    return merged


def example(path, left, truth, depths, pillar):
    fig, axes = plt.subplots(2, len(depths) + 1, figsize=(5 * (len(depths) + 1), 6.5))
    axes[0][0].imshow(left)
    axes[0][0].set_title('left camera')
    axes[1][0].imshow(np.where(pillar, truth, np.nan), cmap='viridis', vmin=0, vmax=400)
    axes[1][0].set_title('true depth, pillar pixels')
    for col, (name, depth) in enumerate(depths.items(), 1):
        shown = axes[0][col].imshow(depth, cmap='viridis', vmin=0, vmax=400)
        axes[0][col].set_title(f'{name} depth [m] (white: none)')
        err = axes[1][col].imshow(np.where(pillar, np.abs(depth - truth), np.nan), cmap='magma', vmin=0, vmax=30)
        axes[1][col].set_title(f'{name} error on pillars [m]')
    fig.colorbar(shown, ax=axes[0].tolist(), fraction=0.02)
    fig.colorbar(err, ax=axes[1].tolist(), fraction=0.02)
    for ax in axes.flat:
        ax.axis('off')
    fig.savefig(path, dpi=80, bbox_inches='tight')
    plt.close(fig)


CAMERAS = {'stereo_pillars_zedx40': 'ZED X, 4 mm lens (1920x1200, 12 cm)',
           'stereo_pillars_zedx22': 'ZED X, 2.2 mm lens (1920x1200, 12 cm)',
           'stereo_pillars_pair1m': 'plane pair (960x540, 90 deg, 1 m)'}
PLOTTED = ('sgm', 'ffs x1 +LR', 'ffs x0.5 +LR')  # without the check: same on pillars, more false obstacles


def plot(summary, path):
    runs = list(summary)
    fig, axes = plt.subplots(2, len(runs), figsize=(5.5 * len(runs), 7.5), squeeze=False, sharey='row')
    for col, run in enumerate(runs):
        for method in PLOTTED:
            merged = summary[run]['methods'][method]
            rows = [r for r in merged['pillar'] if r['pixels'] > 1000]
            centers = [sum(r['range_m']) / 2 for r in rows]
            label = f"{method} (false obstacles {merged['false_obstacle_pct']:.1f} %)"
            axes[0][col].plot(centers, [r['with_depth_pct'] for r in rows], 'o-', label=label)
            axes[1][col].plot(centers, [r['median_err_m'] for r in rows], 'o-', label=label)
        axes[0][col].set_title(CAMERAS.get(run, run))
        axes[0][col].set_ylabel('pillar pixels with depth [%]')
        axes[1][col].set_ylabel('median depth error on pillars [m]')
        axes[1][col].set_yscale('log')
        axes[1][col].set_xlabel('true range [m]')
        axes[1][col].legend(fontsize=8)
        for ax in axes[:, col]:
            ax.grid(alpha=0.3)
            ax.axvspan(60, 150, color='gray', alpha=0.1)
    fig.suptitle('Depth on the pillars, same frames: cf_drone SGM vs Fast-FoundationStereo (+LR: left-right check). '
                 f'Grey: 60-150 m, where a plane must see. False obstacles: depth < {NEAR:.0f} m where nothing is '
                 f'within {FAR:.0f} m')
    fig.savefig(path, dpi=100, bbox_inches='tight')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--ffs-repo', default=FFS_REPO)
    parser.add_argument('--ffs-model', default=FFS_MODEL)
    parser.add_argument('--out', required=True)
    parser.add_argument('runs', nargs='+', help='RUN_DIR:BASELINE')
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)
    ffs = FastFoundationStereo(args.ffs_repo, args.ffs_model)
    rng = np.random.default_rng(0)
    summary = {}
    for spec in args.runs:
        run_dir, baseline = spec.rsplit(':', 1)
        run, baseline = os.path.basename(run_dir.rstrip('/')), float(baseline)
        files = sorted(glob.glob(os.path.join(run_dir, 'stereo_eval_frames', '*.npz')))
        per_method, times, best_pillar = {}, {}, 0.0
        for k, f in enumerate(files):
            d = np.load(f)
            truth, fx = d['truth'], float(d['fx'])
            pillar = ground_plane_mask(truth, fx, rng)
            sky = ~np.isfinite(truth) | (truth <= 0)
            masks = {'pillar': pillar, 'ground': ~sky & ~pillar, 'far': sky | (truth > FAR)}
            depths = {'sgm': d['stereo']}
            for scale in FFS_SCALES:
                name = f'ffs x{scale:g}'
                try:
                    for variant, (depth, seconds) in zip((name, name + ' +LR'), ffs.depths(d['left'], d['right'], fx, baseline, scale)):
                        depths[variant] = depth
                        if k:  # the first call compiles
                            times.setdefault(variant, []).append(seconds)
                except torch.cuda.OutOfMemoryError:
                    torch.cuda.empty_cache()
                    print(f'{run} {name}: out of GPU memory at {truth.shape[1]}x{truth.shape[0]}')
            for name, depth in depths.items():
                per_method.setdefault(name, []).append(score(depth, truth, masks))
            if pillar.mean() >= max(best_pillar, 0.01):  # example: the frame with the most pillar pixels
                best_pillar = pillar.mean()
                example(os.path.join(args.out, f'{run}_example.png'), d['left'], truth, depths, pillar)
            print(f'{run}: frame {k + 1}/{len(files)}, pillar pixels {pillar.mean() * 100:.1f} %', flush=True)
        summary[run] = {'frames': len(files), 'baseline_m': baseline,
                        'methods': {name: merge(results) for name, results in per_method.items()},
                        'ffs_forward_ms': {name: 1000 * float(np.median(t)) for name, t in times.items()}}
    with open(os.path.join(args.out, 'stereo_methods.json'), 'w') as f:
        json.dump(summary, f, indent=1)
    plot(summary, os.path.join(args.out, 'stereo_methods.png'))


if __name__ == '__main__':
    main()
