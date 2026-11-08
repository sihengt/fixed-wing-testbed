"""Plan the same tour with each algorithm and compare the plans (offline, no flight).

Usage: ros2 run fw_planner bench_planners CONFIG [--algorithms a,b] [--rrt-runs N] [--rrt-time S] [-o OUT.json]
jps3d_* need `ros2 launch cf_adapters jps3d_world.launch.py config:=CONFIG` running.

Per plan: planning time, length, worst turn (max curvature x turn radius: > 1 is tighter than the plane can
fly), heading turned tighter than the radius, clearance to the no-fly circles, share of the path inside the
margin, and the worst heading error at the goals (grid planners ignore goal headings).
"""
import argparse
import json
import math
import time

import rclpy
import yaml

from . import planners, rrt_star
from .dubins import sample
from .jps_client import GetPlanClient
from .planner_node import triples
from .world import World

SAMPLE_STEP = 1.0  # [m]


def metrics(start, segments, goals, radius, obstacles, margin):
    points = sample(start, segments, SAMPLE_STEP)
    clearances = [min(math.hypot(x - ox, y - oy) - r for ox, oy, r in obstacles) if obstacles else math.inf
                  for x, y, _, _ in points]
    heading_errors = []
    for gx, gy, gpsi in goals:
        x, y, psi, _ = min(points, key=lambda p: math.hypot(p[0] - gx, p[1] - gy))
        heading_errors.append(abs(math.degrees(math.atan2(math.sin(psi - gpsi), math.cos(psi - gpsi)))))
    return {
        'length_m': sum(length for _, length in segments),
        'max_turn_ratio': max(abs(k) for k, _ in segments) * radius,
        'too_tight_turn_deg': math.degrees(sum(abs(k) * length for k, length in segments if abs(k) * radius > 1 + 1e-6)),
        'min_clearance_m': min(clearances),
        'inside_margin_pct': 100.0 * sum(c < margin - SAMPLE_STEP for c in clearances) / len(clearances),
        'goal_heading_err_deg': max(heading_errors),
    }


def plot(plans, goals, obstacles, bounds, margin, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, len(plans), figsize=(5 * len(plans), 5.5), squeeze=False)
    for ax, (algorithm, (start, segments)) in zip(axes[0], plans.items()):
        # Map view: east to the right, north up.
        for x, y, r in obstacles:
            ax.add_patch(plt.Circle((y, x), r, color='tab:red', alpha=0.3))
            ax.add_patch(plt.Circle((y, x), r + margin, color='tab:red', fill=False, linestyle='--'))
        x0, x1, y0, y1 = bounds
        ax.add_patch(plt.Rectangle((y0, x0), y1 - y0, x1 - x0, fill=False, color='gray', linestyle=':'))
        points = sample(start, segments, SAMPLE_STEP)
        ax.plot([p[1] for p in points], [p[0] for p in points], lw=1.5)
        for gx, gy, gpsi in goals:
            ax.arrow(gy, gx, 25 * math.sin(gpsi), 25 * math.cos(gpsi), width=3, color='k')
        ax.set_title(f'{algorithm}: {sum(length for _, length in segments):.0f} m')
        ax.set_aspect('equal')
        ax.set_xlabel('east [m]')
    axes[0][0].set_ylabel('north [m]')
    fig.savefig(out, dpi=110, bbox_inches='tight')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('config')
    parser.add_argument('--algorithms', default=','.join(planners.ALGORITHMS))
    parser.add_argument('--rrt-runs', type=int, default=10)
    parser.add_argument('--rrt-time', type=float, default=1.0, help='[s] per leg')
    parser.add_argument('-o', '--output')
    parser.add_argument('--plot', help='PNG with the first plan of each algorithm')
    args = parser.parse_args()

    world_cfg = yaml.safe_load(open(args.config))['planner']['ros__parameters']
    goals = [(x, y, math.radians(h)) for x, y, h in triples(world_cfg['goals'])]
    obstacles = triples(world_cfg['obstacles'])
    radius, margin = world_cfg['turn_radius'], world_cfg['margin']
    world = World(obstacles, world_cfg['bounds'], margin)
    rrt_star.seed(1)

    rclpy.init()
    node = rclpy.create_node('bench_planners')
    results, plans = [], {}
    for algorithm in args.algorithms.split(','):
        for run in range(args.rrt_runs if algorithm == 'rrt_star' else 1):
            jps = GetPlanClient(node, world_cfg['altitude'], obstacles) if algorithm.startswith('jps3d') else None
            started = time.monotonic()
            try:
                start, segments = planners.plan(algorithm, goals, radius, world, jps and jps.leg, args.rrt_time)
            except RuntimeError as error:
                results.append({'algorithm': algorithm, 'run': run, 'error': str(error)})
                print(f'{algorithm} run {run}: FAILED {error}')
                continue
            seconds = time.monotonic() - started - (jps.wait_seconds if jps else 0.0)  # not the wait for the map
            row = {'algorithm': algorithm, 'run': run, 'plan_ms': 1000 * seconds,
                   'free': world.path_free(start, segments), **metrics(start, segments, goals, radius, obstacles, margin)}
            results.append(row)
            plans.setdefault(algorithm, (start, segments))
            print(' '.join(f'{k}={v:.2f}' if isinstance(v, float) else f'{k}={v}' for k, v in row.items()))
    node.destroy_node()
    rclpy.shutdown()
    if args.plot:
        plot(plans, goals, obstacles, world_cfg['bounds'], margin, args.plot)
    if args.output:
        with open(args.output, 'w') as f:
            json.dump({'config': args.config, 'rrt_time': args.rrt_time, 'results': results}, f, indent=1)
