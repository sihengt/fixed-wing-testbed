#!/usr/bin/env python3
"""Can a plane fly DROAN's trajectory library? Rebuild it like DROAN does and check plane limits.

The generator is a copy of droan_gl shaders/trajectory.cs (snap feedback toward each library velocity) and the
direction sweep in TrajectoryGenerator.cpp. It is first checked against a dump from DROAN's own GPU code
(test_trajectory_generator -> /tmp/droan_trajectories.bin), then run with plane speed.

Plane limits: speed >= STALL, lateral acceleration <= g tan(bank), climb angle <= MAX_CLIMB.
Usage: droan_library_check.py [--dump /tmp/droan_trajectories.bin] [--out PREFIX]
"""
import argparse
import json
import math

import numpy as np

K_V, K_A, K_J, SNAP_LIM = 8.165, 8.1096, 4.027, 100.0  # from trajectory.cs
G = 9.81
STALL = 12.0  # [m/s] margin above the plane's stall
MAX_CLIMB = math.radians(15)


def directions(min_pitch, max_pitch, pitch_step, min_yaw, max_yaw, yaw_step, speed):
    """TrajectoryGenerator.cpp: pitch outer loop, yaw inner; pitch +-90 gives one vertical trajectory."""
    out = []
    for p in np.arange(min_pitch, max_pitch, pitch_step, dtype=np.float32):
        pr = np.float32(p * math.pi / 180)
        if abs(p) == 90:
            out.append((0.0, 0.0, math.sin(pr) * speed))
            continue
        for y in np.arange(min_yaw, max_yaw, yaw_step, dtype=np.float32):
            yr = np.float32(y * math.pi / 180)
            out.append((math.cos(pr) * math.sin(yr) * speed, math.cos(pr) * math.cos(yr) * speed, math.sin(pr) * speed))
    return np.array(out, dtype=np.float32)


def generate(vel_desired, initial_vel, dt, ht):
    """trajectory.cs for all directions at once: returns positions, velocities, accelerations [n, steps, 3]."""
    n, steps = len(vel_desired), int(np.float32(ht) / np.float32(dt))
    dt = np.float32(dt)
    pos = np.zeros((n, 3), np.float32)
    vel = np.tile(np.asarray(initial_vel, np.float32), (n, 1))
    acc, jerk = np.zeros((n, 3), np.float32), np.zeros((n, 3), np.float32)
    out_p, out_v, out_a = [], [], []
    for _ in range(steps):
        snap = np.clip(-K_V * (vel - vel_desired) - K_A * acc - K_J * jerk, -SNAP_LIM, SNAP_LIM).astype(np.float32)
        jerk = jerk + snap * dt
        acc = acc + jerk * dt
        vel = vel + acc * dt
        pos = pos + vel * dt
        out_p.append(pos.copy())
        out_v.append(vel.copy())
        out_a.append(acc.copy())
    return np.stack(out_p, 1), np.stack(out_v, 1), np.stack(out_a, 1)


def plane_check(vel, acc, bank_deg):
    speed = np.linalg.norm(vel, axis=2)
    unit = vel / np.maximum(speed[..., None], 1e-6)
    lateral = np.linalg.norm(acc - np.sum(acc * unit, axis=2, keepdims=True) * unit, axis=2)
    climb = np.abs(np.arcsin(np.clip(vel[..., 2] / np.maximum(speed, 1e-6), -1, 1)))
    a_max = G * math.tan(math.radians(bank_deg))
    ok_speed = speed.min(1) >= STALL
    ok_turn = lateral.max(1) <= a_max
    ok_climb = climb.max(1) <= MAX_CLIMB
    return {'trajectories': len(vel), 'bank_deg': bank_deg,
            'speed_ok_pct': 100 * ok_speed.mean(), 'turn_ok_pct': 100 * ok_turn.mean(),
            'climb_ok_pct': 100 * ok_climb.mean(), 'all_ok_pct': 100 * (ok_speed & ok_turn & ok_climb).mean(),
            'min_speed_median': float(np.median(speed.min(1))),
            'max_lateral_acc_median': float(np.median(lateral.max(1)))}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--dump', default='/tmp/droan_trajectories.bin')
    parser.add_argument('--out', default='droan_library')
    args = parser.parse_args()
    report = {}

    # 1. Same parameters as DROAN's DumpToBinaryFile test: does the copy match the GPU output?
    dump = np.fromfile(args.dump, dtype=np.float32).reshape(-1, 50, 8)  # vec4 pos + vec4 (seen, 0, 0, speed)
    pos, vel, _ = generate(directions(-30, 31, 15, 0, 360, 15, 1.0), (1, 0, 0), 0.1, 5.0)
    report['copy_vs_gpu'] = {'trajectories': int(dump.shape[0]),
                             'max_pos_diff_m': float(np.abs(pos - dump[:, :, :3]).max()),
                             'max_speed_diff': float(np.abs(np.linalg.norm(vel, axis=2) - dump[:, :, 7]).max())}

    # 2. cf_drone's condo library at plane speed. 3. A plane-shaped library: +-90 deg yaw, +-10 deg pitch.
    speed, start = 15.0, (0.0, 15.0, 0.0)  # flying along +y = library yaw 0
    libraries = {'condo_at_15ms': (-90, 91, 15, 0, 360, 15), 'plane_shaped': (-10, 11, 5, -90, 91, 15)}
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(12, 6))
    for ax, (name, sweep) in zip(axes, libraries.items()):
        pos, vel, acc = generate(directions(*sweep, speed), start, 0.1, 4.0)
        report[name] = {bank: plane_check(vel, acc, bank) for bank in (30, 45)}
        ok = plane_check(vel, acc, 30)
        feasible = (np.linalg.norm(vel, axis=2).min(1) >= STALL)
        for traj, good in zip(pos, feasible):
            ax.plot(traj[:, 0], traj[:, 1], color='tab:green' if good else 'tab:red', lw=0.6)
        ax.set_title(f'{name}: {ok["all_ok_pct"]:.0f} % plane-feasible (30 deg bank)')
        ax.set_aspect('equal')
        ax.set_xlabel('x [m]')
    axes[0].set_ylabel('y [m] (initial heading)')
    fig.savefig(args.out + '.png', dpi=110, bbox_inches='tight')
    with open(args.out + '.json', 'w') as f:
        json.dump(report, f, indent=1)
    print(json.dumps(report, indent=1))


if __name__ == '__main__':
    main()
