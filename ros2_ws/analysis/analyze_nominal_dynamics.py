#!/usr/bin/env python3
"""Inspect original flatness demand with fixed zero flap sum.

This isolates reference geometry and the configured alpha/rate limits. It is
not a full actuator feasibility proof: the live controller uses measured flap
sum and closed-loop forces, and angular acceleration is differentiated at 5 ms.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/phoenix_nominal_matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from phoenix_tailsitter_control import reference_trajectories
from phoenix_tailsitter_control.config import PhoenixHoverConfig
from phoenix_tailsitter_control.flatness_control import FlatnessAttitudeController
from phoenix_tailsitter_control.paper_trajectory_mission import make_reference


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    cfg = PhoenixHoverConfig()
    flat = FlatnessAttitudeController(cfg)
    result = {'interpretation': __doc__,
              'reference_source_sha256': hashlib.sha256(
                  Path(reference_trajectories.__file__).read_bytes()).hexdigest(),
              'profiles': {}}
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10,
                         'svg.fonttype': 'none'})
    for name in ('differential-turn', 'circular-knife-edge', 'knife-edge-transition'):
        curve = make_reference(name)
        time = np.arange(.005, curve.total_duration - .005, .005)
        q = np.array([1., 0., 0., 0.])
        rates, thrust, yaw_rates = [], [], []
        for t in time:
            _, v, a, j, yaw, rate = curve.sample(t)
            force = cfg.mass * (a - np.array([0., 0., cfg.gravity]))
            q, total, roll, pitch = flat.attitude_and_thrust(
                force, v, 0., yaw, q, return_angles=True)
            rates.append(flat.feedforward_rates(v, a, j, yaw, rate, force, roll, pitch, 0.))
            thrust.append(total)
            yaw_rates.append(rate)
        rates = np.asarray(rates)
        acceleration = np.gradient(rates, .005, axis=0)
        result['profiles'][name] = {
            'flap_sum_assumption_rad': 0., 'dt_s': .005,
            'peak_yaw_rate_deg_s': float(np.rad2deg(np.max(np.abs(yaw_rates)))),
            'peak_body_rates_rad_s': np.abs(rates).max(axis=0).tolist(),
            'peak_body_angular_acceleration_rad_s2': np.abs(acceleration).max(axis=0).tolist(),
            'configured_angular_acceleration_limit_rad_s2': cfg.angular_acceleration_limit.tolist(),
            'nominal_angular_acceleration_above_limit_fraction': np.mean(
                np.abs(acceleration) > cfg.angular_acceleration_limit, axis=0).tolist(),
            'nominal_thrust_minmax_n': [float(min(thrust)), float(max(thrust))],
        }
        np.savez_compressed(args.output_dir / f'{name}_nominal_dynamics.npz',
                            time=time, rates=rates, acceleration=acceleration,
                            thrust=thrust, yaw_rates=yaw_rates)
        if name == 'differential-turn':
            fig, axes = plt.subplots(2, 3, figsize=(13, 7), sharex=True,
                                     layout='constrained')
            for k, axis in enumerate('xyz'):
                axes[0, k].plot(time, rates[:, k], color='#007F86')
                axes[0, k].set(title=f'TS {axis}', ylabel='Nominal body rate (rad/s)')
                axes[1, k].plot(time, acceleration[:, k], color='#007F86',
                                label='Differentiated nominal rate')
                for sign in (-1, 1):
                    axes[1, k].axhline(sign * cfg.angular_acceleration_limit[k],
                                      color='#E68A2E', linestyle='--',
                                      label='Controller limit' if sign == 1 else None)
                axes[1, k].set(ylabel='Angular acceleration (rad/s^2)',
                               xlabel='Original tracking time (s)')
                axes[1, k].legend(fontsize=8)
            for ax in axes.flat:
                ax.grid(alpha=.2)
            fig.suptitle('Original differential turn: flatness demand at zero flap sum\n'
                         'Geometry diagnostic; not a complete actuator feasibility proof')
            fig.savefig(args.output_dir / 'differential_turn_nominal_demand.png', dpi=170)
            fig.savefig(args.output_dir / 'differential_turn_nominal_demand.svg')
            plt.close(fig)
    (args.output_dir / 'nominal_dynamics.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
