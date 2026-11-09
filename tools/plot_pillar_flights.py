"""Top view of closed-loop flights in the pillars or forest world, from offboard start to the end of the log.
Pillars come from the first run's plan.json (written by droan_bridge / super_bridge for that world).

Usage: python3 tools/plot_pillar_flights.py OUT.png RUN_DIR:LABEL [RUN_DIR:LABEL ...]   (RUN_DIR from run_experiment.sh)
"""
import json
import sys

import matplotlib
import numpy as np
from pyulog import ULog

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

MARGIN = 15.0  # [m] the planners' no-fly margin


def track(run_dir):
    u = ULog(f'{run_dir}/flight.ulg', ['vehicle_local_position', 'vehicle_status'])
    lp, st = u.get_dataset('vehicle_local_position').data, u.get_dataset('vehicle_status').data
    offboard = st['timestamp'][np.argmax(st['nav_state'] == 14)]
    m = lp['timestamp'] > offboard
    return lp['y'][m], lp['x'][m]  # east, north


def main():
    out, runs = sys.argv[1], [a.rsplit(':', 1) for a in sys.argv[2:]]
    pillars = [(east, north, r) for north, east, r in json.load(open(f'{runs[0][0]}/plan.json'))['obstacles']]
    fig, ax = plt.subplots(figsize=(11, 6))
    for x, y, r in pillars:
        ax.add_patch(plt.Circle((x, y), r, color='tab:red', alpha=0.5))
        ax.add_patch(plt.Circle((x, y), r + MARGIN, color='tab:red', fill=False, ls='--'))
    ax.plot([250, 1200], [0, 0], 'k:', lw=1, label='route (waypoints 250 m, 1200 m east)')
    for run_dir, label in runs:
        east, north = track(run_dir)
        clear = min(np.min(np.hypot(east - px, north - py)) - r for px, py, r in pillars)
        ax.plot(east, north, lw=1.5, label=f'{label} (closest pillar edge {clear:.0f} m)')
    ax.set_aspect('equal')
    ax.set(xlim=(50, 1350), ylim=(-480, 320))  # the route; the loops after the goal run off this view
    ax.set_xlabel('east [m]')
    ax.set_ylabel('north [m]')
    ax.grid(alpha=0.3)
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.1), ncol=2, fontsize=9)
    ax.set_title(f'Closed-loop flights past {len(pillars)} pillars (from offboard start; loops after the goal cut off)')
    fig.savefig(out, dpi=110, bbox_inches='tight')


if __name__ == '__main__':
    main()
