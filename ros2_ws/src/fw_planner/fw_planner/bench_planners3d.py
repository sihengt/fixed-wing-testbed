"""Compare 3D tours (offline): 3D Hybrid A* (Dubins airplane) vs cf_drone's jps3d in 3D, plus the 2D plan.

Usage: ros2 run fw_planner bench_planners3d CONFIG [-o OUT.json] [--plot OUT.png]
jps3d needs `ros2 launch cf_adapters jps3d_world.launch.py config:=CONFIG dim:=3` running.

- hybrid2d: our 2D Hybrid A* at one altitude, every no-fly zone treated as infinitely tall (reference)
- hybrid3d: 3D Hybrid A*, climbs within max_climb, flies over low zones
- jps3d_raw: jps3d's 3D grid path (corners, any climb angle)
- jps3d_guided: jps3d's 3D route as waypoints for 3D Hybrid A*
Per plan: time, length, worst turn (curvature x R), steepest climb vs the limit, 3D clearance, goal headings.
"""
import argparse
import json
import math
import time

import rclpy
import yaml

from . import planner3d
from .dubins import sample as sample2d
from .hybrid_astar import plan as plan2d
from .jps_client import GetPlanClient
from .world import World, plan_tour

SAMPLE_STEP = 1.0
GUIDE_SPACING = 3.0  # [turn radii] between jps3d waypoints given to Hybrid A*


def chunks(values, n):
    return [tuple(values[i:i + n]) for i in range(0, len(values), n)]


def clearance(x, y, h, cylinders):
    """3D distance to the nearest cylinder (x, y, r, bottom, top), 0 inside."""
    best = math.inf
    for ox, oy, r, bottom, top in cylinders:
        dx = max(0.0, math.hypot(x - ox, y - oy) - r)
        dz = max(0.0, bottom - h, h - top)
        best = min(best, math.hypot(dx, dz))
    return best


def metrics(points, goals, radius, max_climb, cylinders, margin):
    """points: (x, y, h, psi) every ~1 m. Turn radius and climb angle from consecutive points."""
    length = turns = steepest = 0.0
    too_steep = 0
    for (x0, y0, h0, p0), (x1, y1, h1, p1) in zip(points, points[1:]):
        flat = math.hypot(x1 - x0, y1 - y0)
        length += math.hypot(flat, h1 - h0)
        if flat > 1e-6:
            turns = max(turns, abs(math.atan2(math.sin(p1 - p0), math.cos(p1 - p0))) / flat * radius)
            angle = math.degrees(math.atan2(abs(h1 - h0), flat))
            steepest = max(steepest, angle)
            too_steep += angle > math.degrees(max_climb) + 0.5
    clear = [clearance(x, y, h, cylinders) for x, y, h, _ in points]
    heading_errors = []
    for gx, gy, gh, gpsi in goals:
        p = min(points, key=lambda q: math.hypot(q[0] - gx, q[1] - gy))
        heading_errors.append(abs(math.degrees(math.atan2(math.sin(p[3] - gpsi), math.cos(p[3] - gpsi)))))
    return {'length_m': length, 'max_turn_ratio': turns, 'steepest_climb_deg': steepest,
            'too_steep_pct': 100.0 * too_steep / max(1, len(points) - 1), 'min_clearance_m': min(clear),
            'inside_margin_pct': 100.0 * sum(c < margin - SAMPLE_STEP for c in clear) / len(clear),
            'goal_heading_err_deg': max(heading_errors)}


def polyline_points(corners):
    """Corner points (x, y, h) -> (x, y, h, psi) every SAMPLE_STEP metres (heading along each leg)."""
    closed = corners + corners[:1]
    out = []
    for a, b in zip(closed, closed[1:]):
        psi = math.atan2(b[1] - a[1], b[0] - a[0])
        n = max(1, math.ceil(math.dist(a, b) / SAMPLE_STEP))
        out += [tuple(a[k] + (b[k] - a[k]) * i / n for k in range(3)) + (psi,) for i in range(n)]
    return out


def guided(corners, goal_indices, goals, radius, world, max_climb):
    closed = corners + corners[:1]
    segments = []
    for g, (start, end) in enumerate(zip(goal_indices, goal_indices[1:] + [len(corners)])):
        nxt = goals[(g + 1) % len(goals)]
        waypoints = [goals[g]]
        for i in range(start + 1, end):
            x, y, h = closed[i]
            if math.hypot(x - waypoints[-1][0], y - waypoints[-1][1]) >= GUIDE_SPACING * radius \
                    and math.hypot(nxt[0] - x, nxt[1] - y) >= GUIDE_SPACING * radius:
                waypoints.append((x, y, h, math.atan2(closed[i + 1][1] - y, closed[i + 1][0] - x)))
        waypoints.append(nxt)
        current = waypoints[0]
        for i, waypoint in enumerate(waypoints[1:], 1):
            try:
                segments += planner3d.plan(current, waypoint, radius, world, max_climb, max_expansions=5000)
                current = waypoint
            except RuntimeError:
                if i == len(waypoints) - 1:
                    raise
    return segments


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('config')
    parser.add_argument('-o', '--output')
    parser.add_argument('--plot')
    args = parser.parse_args()
    cfg = yaml.safe_load(open(args.config))['planner']['ros__parameters']
    goals = [(x, y, h, math.radians(psi)) for x, y, h, psi in chunks(cfg['goals3d'], 4)]
    cylinders = chunks(cfg['obstacles3d'], 5)
    radius, margin, max_climb = cfg['turn_radius'], cfg['margin'], math.radians(cfg['max_climb_deg'])
    world3 = planner3d.World3D(cylinders, cfg['bounds'], tuple(cfg['altitudes']), margin)

    rclpy.init()
    node = rclpy.create_node('bench_planners3d')
    jps = GetPlanClient(node, 0.0, cylinders)
    results, plans = [], {}

    def run(name, make_points):
        started, waited = time.monotonic(), jps.wait_seconds
        try:
            points = make_points()
        except RuntimeError as error:
            print(f'{name}: FAILED {error}')
            results.append({'algorithm': name, 'error': str(error)})
            return
        seconds = time.monotonic() - started - (jps.wait_seconds - waited)
        row = {'algorithm': name, 'plan_ms': 1000 * seconds,
               **metrics(points, goals, radius, max_climb, cylinders, margin)}
        results.append(row)
        plans[name] = points
        print(' '.join(f'{k}={v:.2f}' if isinstance(v, float) else f'{k}={v}' for k, v in row.items()))

    # 2D reference: one altitude (the first goal's), all zones infinitely tall; goal altitudes are ignored.
    altitude = goals[0][2]
    world2 = World([c[:3] for c in cylinders], cfg['bounds'], margin)
    run('hybrid2d', lambda: [(x, y, altitude, psi) for x, y, psi, _ in sample2d(
        (goals[0][0], goals[0][1], goals[0][3]),
        plan_tour([(g[0], g[1], g[3]) for g in goals], lambda a, b: plan2d(a, b, radius, world2)), SAMPLE_STEP)])
    run('hybrid3d', lambda: planner3d.sample(goals[0], planner3d.plan_tour(goals, radius, world3, max_climb=max_climb),
                                             SAMPLE_STEP))
    corners, goal_indices = [], []
    jps_started = time.monotonic()
    for a, b in zip(goals, goals[1:] + goals[:1]):
        goal_indices.append(len(corners))
        corners += jps.leg(a, b)[:-1]
    jps_seconds = time.monotonic() - jps_started - jps.wait_seconds
    run('jps3d_raw', lambda: polyline_points(corners))
    results[-1]['plan_ms'] = 1000 * jps_seconds
    run('jps3d_guided', lambda: planner3d.sample(goals[0], guided(corners, goal_indices, goals, radius, world3, max_climb),
                                                 SAMPLE_STEP))
    if results[-1].get('plan_ms') is not None:
        results[-1]['plan_ms'] += 1000 * jps_seconds
    node.destroy_node()
    rclpy.shutdown()

    if args.plot:
        plot(plans, goals, cylinders, cfg, args.plot)
    if args.output:
        with open(args.output, 'w') as f:
            json.dump({'config': args.config, 'results': results}, f, indent=1)


def plot(plans, goals, cylinders, cfg, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, len(plans), figsize=(5 * len(plans), 8), gridspec_kw={'height_ratios': [3, 1]},
                             squeeze=False)
    low, high = cfg['altitudes']
    for col, (name, points) in enumerate(plans.items()):
        top, side = axes[0][col], axes[1][col]
        for x, y, r, bottom, height in cylinders:
            tall = height > high
            top.add_patch(plt.Circle((y, x), r, color='tab:red' if tall else 'tab:orange', alpha=0.35))
            top.text(y, x, '' if tall else f'{height:.0f} m', ha='center', va='center', fontsize=7)
        sc = top.scatter([p[1] for p in points], [p[0] for p in points], c=[p[2] for p in points], s=1, cmap='viridis',
                         vmin=low, vmax=high)
        for gx, gy, gh, gpsi in goals:
            top.arrow(gy, gx, 25 * math.sin(gpsi), 25 * math.cos(gpsi), width=3, color='k')
        top.set_aspect('equal')
        top.set_title(name)
        side.plot([p[2] for p in points], lw=1)
        side.axhspan(low, high, color='gray', alpha=0.1)
        side.set_xlabel('distance along the tour [m]')
        side.set_ylabel('altitude [m]')
    fig.colorbar(sc, ax=axes[0].tolist(), label='altitude [m]', shrink=0.6)
    fig.suptitle('3D tours: red = taller than the flight band (go around), orange = low (fly over, top altitude shown)')
    fig.savefig(out, dpi=100, bbox_inches='tight')
