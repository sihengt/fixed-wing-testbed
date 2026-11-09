"""How Fast-FoundationStereo holds up when the sim frames get real-camera faults (same frames and scoring as
stereo_methods.py, full resolution with the left-right check).

Faults: 'camera' = blur (sigma 0.7 px), sensor noise (sigma 3 of 255) and a 5 % gain difference between the eyes;
then on top of that a calibration error: the right image moved 0.5 px vertically (rectification error) or
+-0.5 px horizontally (a disparity offset: + reads nearer, - reads farther).
Run with the Fast-FoundationStereo venv (tools/install_ffs.sh):
  external/ffs-venv/bin/python tools/stereo_degrade.py --out FILE.json RUN_DIR:BASELINE [RUN_DIR:BASELINE ...]
"""
import argparse
import glob
import json
import os

import cv2
import matplotlib
import numpy as np

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

from stereo_methods import FAR, FFS_MODEL, FFS_REPO, FastFoundationStereo, ground_plane_mask, merge, score


def shifted(image, dx, dy):
    m = np.float32([[1, 0, dx], [0, 1, dy]])
    return cv2.warpAffine(image, m, image.shape[1::-1], flags=cv2.INTER_LANCZOS4, borderMode=cv2.BORDER_REFLECT)


def camera(image, gain, rng):
    blurred = cv2.GaussianBlur(image.astype(np.float32), (0, 0), 0.7) * gain
    return np.clip(blurred + rng.normal(0, 3, image.shape), 0, 255).astype(np.uint8)


FAULTS = {  # name: (camera faults on, right image shift dx, dy [px])
    'clean': (False, 0, 0),
    'camera': (True, 0, 0),
    'camera + 0.5 px vertical': (True, 0, 0.5),
    'camera + 0.5 px nearer': (True, -0.5, 0),  # right image left = more disparity = nearer
    'camera + 0.5 px farther': (True, 0.5, 0),
}


COLORS = ('#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4')  # categorical slots 1-5, fixed order
MARKERS = ('o', 's', '^', 'D', 'v')
TITLES = {'stereo_pillars_zedx40': 'ZED X 4 mm (12 cm baseline, 1920x1200)', 'stereo_pillars_pair1m': '1 m pair (960x540, 90 deg)'}


def plot(summary, path):
    fig, axes = plt.subplots(1, len(summary), figsize=(6 * len(summary), 4.6), sharey=True, squeeze=False)
    for ax, (run, faults) in zip(axes[0], summary.items()):
        for (name, merged), color, marker in zip(faults.items(), COLORS, MARKERS):
            rows = [r for r in merged['pillar'] if r['pixels'] > 1000 and 50 <= r['range_m'][0] < 250]
            ax.plot([sum(r['range_m']) / 2 for r in rows], [r['median_err_m'] for r in rows], marker=marker, color=color, lw=2,
                    ms=7, label=f"{name} (false obstacles {merged['false_obstacle_pct']:.0f} %)")
        ax.set(title=TITLES.get(run, run), xlabel='true range [m]', yscale='log')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8, loc='upper center', bbox_to_anchor=(0.5, -0.16), ncol=1)
    axes[0][0].set_ylabel('median depth error on pillars [m]')
    fig.suptitle('Fast-FoundationStereo + LR check with real-camera faults (camera = blur, noise, 5 % gain difference)')
    fig.savefig(path, dpi=100, bbox_inches='tight')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--ffs-repo', default=FFS_REPO)
    parser.add_argument('--ffs-model', default=FFS_MODEL)
    parser.add_argument('--out', required=True)
    parser.add_argument('runs', nargs='+', help='RUN_DIR:BASELINE')
    args = parser.parse_args()
    ffs = FastFoundationStereo(args.ffs_repo, args.ffs_model)
    rng = np.random.default_rng(0)
    summary = {}
    for spec in args.runs:
        run_dir, baseline = spec.rsplit(':', 1)
        run = os.path.basename(run_dir.rstrip('/'))
        files = sorted(glob.glob(os.path.join(run_dir, 'stereo_eval_frames', '*.npz')))
        per_fault = {}
        for k, f in enumerate(files):
            d = np.load(f)
            truth, fx = d['truth'], float(d['fx'])
            pillar = ground_plane_mask(truth, fx, rng)
            sky = ~np.isfinite(truth) | (truth <= 0)
            masks = {'pillar': pillar, 'ground': ~sky & ~pillar, 'far': sky | (truth > FAR)}
            for name, (faults, dx, dy) in FAULTS.items():
                left, right = d['left'], d['right']
                if faults:
                    left, right = camera(left, 1.0, rng), camera(right, 1.05, rng)
                if dx or dy:
                    right = shifted(right, dx, dy)
                _, (depth, _) = ffs.depths(left, right, fx, float(baseline), 1.0)
                per_fault.setdefault(name, []).append(score(depth, truth, masks))
            print(f'{run}: frame {k + 1}/{len(files)}', flush=True)
        summary[run] = {name: merge(results) for name, results in per_fault.items()}
    with open(args.out, 'w') as f:
        json.dump(summary, f, indent=1)
    plot(summary, args.out.replace('.json', '.png'))
    for run, faults in summary.items():
        print(f'\n{run}: pillar median error [m] (share with depth) at 50-75 / 75-100 / 100-150 / 150-250 m, false obstacles')
        for name, merged in faults.items():
            cells = [f"{r['median_err_m']:.1f} ({r['with_depth_pct']:.0f} %)" for r in merged['pillar'][1:5]]
            print(f"  {name:26s} {' / '.join(cells)}  false {merged['false_obstacle_pct']:.1f} %")


if __name__ == '__main__':
    main()
