#!/usr/bin/env python3
"""Fit effective one-pole actuator response from recorded commands/joint states.

This is a SITL consistency diagnostic, not hardware identification. The effective
delay includes command transport and timestamp alignment; the state equation is
evaluated over individual measurement intervals with held controller commands.
"""
import argparse
import json
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/phoenix_actuator_fit_matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.optimize import minimize_scalar

from analyze_tracking_oscillations import read_extra
from phoenix_tailsitter_control.actuator_feedback import parse_actuator_joint_state
from phoenix_tailsitter_control.config import PhoenixHoverConfig


def fit_response(t, state, cmd_time, cmd):
    dt = np.diff(t)
    valid = (dt > .002) & (dt < .04)
    t0, h = t[:-1][valid], dt[valid]
    y0, y1 = state[:-1][valid], state[1:][valid]
    nodes = np.array([.1127016654, .5, .8872983346])
    weights = np.array([5 / 18, 8 / 18, 5 / 18])
    def command_samples(delay):
        sample_times = t0[:, None] + h[:, None] * nodes - delay
        index = np.clip(np.searchsorted(cmd_time, sample_times, side='right') - 1,
                        0, len(cmd_time) - 1)
        return cmd[index]
    def prediction(tau, samples):
        kernel = weights * np.exp(-h[:, None] * (1 - nodes) / tau)
        u = np.sum(samples * kernel, axis=1) / kernel.sum(axis=1)
        decay = np.exp(-h / tau)
        return decay * y0 + (1 - decay) * u
    best = None
    scale = max(float(np.std(np.diff(state))), 1e-4)
    for delay in np.arange(0., .02501, .0005):
        samples = command_samples(delay)
        def loss(tau):
            residual = (prediction(tau, samples) - y1) / scale
            return float(np.mean(np.sqrt(1 + residual**2) - 1))
        candidate = minimize_scalar(loss, bounds=(.01, .12), method='bounded',
                                    options={'xatol': 1e-6})
        if best is None or candidate.fun < best[0]:
            best = (candidate.fun, candidate.x, delay)
    _, tau, delay = best
    residual = prediction(tau, command_samples(delay)) - y1
    return {'effective_time_constant_s': float(tau), 'effective_delay_s': float(delay),
            'one_step_rmse': float(np.sqrt(np.mean(residual**2))),
            'state_span': float(np.ptp(state)), 'valid_intervals': int(valid.sum()),
            'delay_search_resolution_s': .0005,
            'fit_at_bound': bool(tau <= .0101 or tau >= .1199 or delay >= .0249)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    out = args.output_dir
    out.mkdir(parents=True, exist_ok=True)
    extra = read_extra(args.case, out / 'tracking_extra.npz')
    epoch = float(extra['phase_start_epoch_s'])
    summary = json.loads((args.case / 'summary.json').read_text())
    duration = min(summary['tracking_duration_s'], summary['mission']['reference_duration_s'])
    cfg = PhoenixHoverConfig()
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.case / 'bag'), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    topic = '/world/stars_ts/model/phoenixdrone_alpha_0/joint_state'
    cls = get_message('sensor_msgs/msg/JointState')
    rows = []
    while reader.has_next():
        name, raw, stamp = reader.read_next()
        t = stamp * 1e-9 - epoch
        if name != topic or not .1 <= t <= duration:
            continue
        msg = deserialize_message(raw, cls)
        state = parse_actuator_joint_state(msg, cfg.rotor_velocity_slowdown)
        sim_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        rows.append([t, sim_t, *state.motor_speed_left_right, *state.flap_angle_left_right])
    joints = np.array(rows)
    if len(joints) < 100:
        raise ValueError('Insufficient raw joint-state measurements')
    sim_clock = np.ptp(joints[:, 1]) > 1 and np.all(np.diff(joints[:, 1]) >= 0)
    if sim_clock:
        slope, offset = np.polyfit(joints[:, 0], joints[:, 1], 1)
        keep = np.r_[True, np.diff(joints[:, 1]) > 1e-6]
        joints = joints[keep]
        time = joints[:, 1]
        cmd_time = extra['control'][:, 0] * slope + offset
    else:
        slope, offset = 1., 0.
        time, cmd_time = joints[:, 0], extra['control'][:, 0]
    controls = np.column_stack([extra['control'][:, 22:24] * cfg.motor_speed_max,
                               extra['control'][:, 24:26]])
    labels = ['motor_left', 'motor_right', 'flap_left', 'flap_right']
    metrics = {label: fit_response(time, joints[:, 2 + i], cmd_time, controls[:, i])
               for i, label in enumerate(labels)}
    result = {'case': str(args.case.resolve()), 'joint_header_clock_used': bool(sim_clock),
              'simulation_seconds_per_wall_second': float(slope), 'actuators': metrics,
              'interpretation': 'Effective SITL response, including timestamp alignment and transport. Single trajectory, no hardware calibration; bound-hitting fits are not conclusive.'}
    (out / 'actuator_fit.json').write_text(json.dumps(result, indent=2) + '\n')
    np.savez_compressed(out / 'joint_samples.npz', joints=joints)
    print(json.dumps(result, indent=2), flush=True)
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True, layout='constrained')
    wall_t = (time - offset) / slope
    zoom = (wall_t >= 11) & (wall_t < 17)
    control_zoom = (extra['control'][:, 0] >= 11) & (extra['control'][:, 0] < 17)
    axes[0].plot(extra['control'][control_zoom, 0], controls[control_zoom, 0], color='#E68A2E', label='Left motor command')
    axes[0].plot(wall_t[zoom], joints[zoom, 2], color='#007F86', label='Measured left motor')
    axes[0].set_ylabel('Motor speed (rad/s)')
    axes[1].plot(extra['control'][control_zoom, 0], np.rad2deg(controls[control_zoom, 2]), color='#E68A2E', label='Left flap command')
    axes[1].plot(wall_t[zoom], np.rad2deg(joints[zoom, 4]), color='#007F86', label='Measured left flap')
    axes[1].set(ylabel='Flap angle (deg)', xlabel='Tracking time (s)')
    for ax in axes:
        ax.grid(alpha=.2)
        ax.legend()
    m, f = metrics['motor_left'], metrics['flap_left']
    fig.suptitle(f"Recorded command/actual response | effective left motor tau {m['effective_time_constant_s']*1000:.1f} ms; flap tau {f['effective_time_constant_s']*1000:.1f} ms\nEffective delay includes transport and clock alignment", fontsize=12)
    fig.savefig(out / 'actuator_response.png', dpi=170)
    fig.savefig(out / 'actuator_response.svg')
    plt.close(fig)


if __name__ == '__main__':
    main()
