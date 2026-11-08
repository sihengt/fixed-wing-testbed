"""Plot the planned path against the flown track from a PX4 log, and print the cross-track error."""
import argparse
import json

import matplotlib
import numpy as np
from pyulog import ULog

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

NAVIGATION_STATE_OFFBOARD = 14
JOINED_ERROR = 5.0  # [m] the track counts as on the path once it first comes this close


def densify(path, step=0.5):
    """Points every `step` metres along a polyline, so point distance approximates line distance."""
    out = [path[0]]
    for a, b in zip(path[:-1], path[1:]):
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
        out.extend(a + (b - a) * (np.arange(1, n + 1)[:, None] / n))
    return np.array(out)


def cross_track_error(track, path):
    dense = densify(path)
    return np.array([np.min(np.linalg.norm(dense - p, axis=1)) for p in track])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('plan', help='plan JSON written by the planner node (plan_file)')
    parser.add_argument('ulog', help='PX4 log (.ulg) of the flight')
    parser.add_argument('-o', '--out', default='flight.png')
    args = parser.parse_args()

    with open(args.plan) as f:
        plan = json.load(f)
    path = np.array(plan['path'])

    log = {d.name: d.data for d in ULog(args.ulog, ['vehicle_local_position', 'vehicle_status']).data_list}
    lp, vs = log['vehicle_local_position'], log['vehicle_status']
    status_index = np.clip(np.searchsorted(vs['timestamp'], lp['timestamp'], side='right') - 1, 0, None)
    offboard = vs['nav_state'][status_index] == NAVIGATION_STATE_OFFBOARD
    track = np.c_[lp['x'], lp['y']][offboard]
    if len(track) == 0:
        raise SystemExit('No offboard flight in this log')

    error = cross_track_error(track, path)
    joined = np.argmax(error < JOINED_ERROR) if (error < JOINED_ERROR).any() else len(error)
    on_path = error[joined:]
    if len(on_path):
        print(f'Cross-track error on path ({len(on_path)} samples): mean {on_path.mean():.1f} m, '
              f'p95 {np.percentile(on_path, 95):.1f} m, max {on_path.max():.1f} m')

    fig, ax = plt.subplots(figsize=(8, 8))
    # Map view: east to the right, north up.
    for x, y, r in plan['obstacles']:
        ax.add_patch(plt.Circle((y, x), r, color='tab:red', alpha=0.3))
        ax.add_patch(plt.Circle((y, x), r + plan['margin'], color='tab:red', fill=False, linestyle='--'))
    if plan['bounds']:
        x0, x1, y0, y1 = plan['bounds']
        ax.add_patch(plt.Rectangle((y0, x0), y1 - y0, x1 - x0, fill=False, color='gray', linestyle=':'))
    ax.plot(path[:, 1], path[:, 0], label='planned', lw=2)
    ax.plot(track[:joined + 1, 1], track[:joined + 1, 0], label='approach', color='tab:gray', lw=1)
    ax.plot(track[joined:, 1], track[joined:, 0], label='flown', color='tab:orange', lw=1)
    ax.set_xlabel('east [m]')
    ax.set_ylabel('north [m]')
    ax.set_aspect('equal')
    ax.legend()
    fig.savefig(args.out, dpi=120, bbox_inches='tight')
    print(f'Wrote {args.out}')


if __name__ == '__main__':
    main()
