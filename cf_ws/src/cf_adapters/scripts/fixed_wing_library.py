#!/usr/bin/env python3
"""Prototype of a plane-feasible trajectory library for DROAN, compared with DROAN's own generator.

DROAN drives the velocity vector to each library direction with snap feedback (droan_library_check.py has the
exact copy). A plane cannot: it flies at about constant speed and turns at most V/R. Here each trajectory is
a commanded turn rate (|w| <= V/R, reached through a first-order roll lag) that may level out half way, and a
climb angle within +-MAX_CLIMB. Everything stays the same for DROAN's collision check: positions over time.

Writes OUT.png (both libraries from above and from the side) and OUT.json (feasibility, coverage).
Usage: fixed_wing_library.py [--out PREFIX]
"""
import argparse
import json
import math

import numpy as np

from droan_library_check import directions, generate, plane_check

SPEED, RADIUS, ROLL_TAU = 15.0, 40.0, 0.5  # [m/s], [m], [s] roll response of PX4's fixed-wing attitude loop
MAX_CLIMB = math.radians(10)
DT, HORIZON = 0.1, 4.0  # as DROAN's config


def fixed_wing_library(turn_steps=13, climb_steps=3):
    """Positions, velocities, accelerations [n, steps, 3] for turn rate x (turn whole horizon | then level) x climb."""
    w_max = SPEED / RADIUS
    out_p, out_v, out_a = [], [], []
    for w_cmd in np.linspace(-w_max, w_max, turn_steps):
        for level_after in ((HORIZON, HORIZON / 2) if w_cmd else (HORIZON,)):
            for climb in np.linspace(-MAX_CLIMB, MAX_CLIMB, climb_steps):
                pos, heading, w = np.zeros(3), math.pi / 2, 0.0  # start along +y, like the DROAN check
                ps, vs, accs = [], [], []
                for k in range(int(HORIZON / DT)):
                    target = w_cmd if k * DT < level_after else 0.0
                    w += (target - w) * DT / ROLL_TAU
                    heading += w * DT
                    vel = SPEED * np.array([math.cos(climb) * math.cos(heading), math.cos(climb) * math.sin(heading),
                                            math.sin(climb)])
                    acc = SPEED * w * math.cos(climb) * np.array([-math.sin(heading), math.cos(heading), 0.0])
                    pos = pos + vel * DT
                    ps.append(pos)
                    vs.append(vel)
                    accs.append(acc)
                out_p.append(ps)
                out_v.append(vs)
                out_a.append(accs)
    return np.array(out_p), np.array(out_v), np.array(out_a)


def coverage(pos, cell=5.0):
    """Ground area [m^2] the library sweeps (5 m cells), a proxy for how many escape routes it offers."""
    cells = {(int(x // cell), int(y // cell)) for x, y in pos[:, :, :2].reshape(-1, 2)}
    return len(cells) * cell * cell


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--out', default='fixed_wing_library')
    args = parser.parse_args()

    droan_p, droan_v, droan_a = generate(directions(-10, 11, 5, -90, 91, 15, SPEED), (0.0, SPEED, 0.0), DT, HORIZON)
    fw_p, fw_v, fw_a = fixed_wing_library()
    libraries = {'DROAN generator, plane-shaped sweep': (droan_p, droan_v, droan_a),
                 'fixed-wing arcs (prototype)': (fw_p, fw_v, fw_a)}
    report = {}
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(13, 9), gridspec_kw={'height_ratios': [3, 1]})
    for col, (name, (p, v, a)) in enumerate(libraries.items()):
        check = plane_check(v, a, 30)
        speed = np.linalg.norm(v, axis=2)
        unit = v / speed[..., None]
        lateral = np.linalg.norm(a - np.sum(a * unit, axis=2, keepdims=True) * unit, axis=2)
        ok = (speed.min(1) >= 12.0) & (lateral.max(1) <= 9.81 * math.tan(math.radians(30)))
        report[name] = {'trajectories': len(p), 'feasible_pct_30deg_bank': check['all_ok_pct'],
                        'ground_coverage_m2': coverage(p), 'feasible_coverage_m2': coverage(p[ok]) if ok.any() else 0}
        top, side = axes[0][col], axes[1][col]
        for traj, good in zip(p, ok):
            color = 'tab:green' if good else 'tab:red'
            top.plot(traj[:, 0], traj[:, 1], color=color, lw=0.7)
            side.plot(traj[:, 1], traj[:, 2], color=color, lw=0.7)
        top.set_title(f'{name}\n{check["all_ok_pct"]:.0f} % plane-feasible, '
                      f'{report[name]["feasible_coverage_m2"] / 1000:.1f}k m^2 covered by feasible ones')
        top.set_aspect('equal')
        top.set_xlim(-60, 60)
        top.set_ylim(-5, 65)
        top.set_xlabel('x [m] (from above)')
        side.set_xlabel('y [m] (from the side)')
        side.set_ylim(-12, 12)
    axes[0][0].set_ylabel('y [m] (initial heading)')
    axes[1][0].set_ylabel('z [m]')
    fig.suptitle('4 s trajectory libraries at 15 m/s (green: plane can fly it at 30 deg bank, red: cannot)')
    fig.savefig(args.out + '.png', dpi=110, bbox_inches='tight')
    with open(args.out + '.json', 'w') as f:
        json.dump(report, f, indent=1)
    print(json.dumps(report, indent=1))


if __name__ == '__main__':
    main()
