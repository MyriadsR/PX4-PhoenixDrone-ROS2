#!/usr/bin/env python3
"""Plot recorded tracking phases; TS attitude uses intrinsic Z-X-Y Euler angles."""
import argparse
import json
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/phoenix_tracking_matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

POS = '/phoenix_tailsitter/position_debug'
ATT = '/phoenix_tailsitter/attitude_tracking_debug'
ACTUAL, REFERENCE = '#007F86', '#E68A2E'


def read_tracking(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()
             if t.name in (POS, ATT, '/rosout')}
    positions, attitudes, phases = [], [], []
    mission = None
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic not in types:
            continue
        msg = deserialize_message(data, types[topic])
        if topic == POS:
            if len(msg.data) != 33:
                raise ValueError('Unexpected position_debug schema')
            positions.append([stamp * 1e-9, *msg.data])
        elif topic == ATT:
            if len(msg.data) != 8:
                raise ValueError('Unexpected attitude_tracking_debug schema')
            attitudes.append([stamp * 1e-9, *msg.data])
        elif msg.name == 'phoenix_lemniscate_mission_test':
            t = msg.stamp.sec + msg.stamp.nanosec * 1e-9
            if msg.msg.startswith('LEMNISCATE_PHASE '):
                phases.append((msg.msg.split()[-1], t))
            elif msg.msg.startswith('LEMNISCATE_MISSION_RESULT '):
                mission = json.loads(msg.msg.split(' ', 1)[1])
    starts = [t for name, t in phases if name == 'track']
    if not starts:
        raise ValueError('Bag contains no track phase; inspect mission.log')
    start = starts[0]
    ends = [t for name, t in phases if name in ('exit', 'abort_brake', 'land') and t > start]
    if not ends:
        raise ValueError('Bag contains no completed tracking phase')
    end = min(ends)
    def select(rows):
        if not rows:
            raise ValueError('Required debug stream is empty')
        array = np.asarray(rows)
        array = array[(array[:, 0] >= start) & (array[:, 0] < end)]
        if not len(array) or not np.all(np.isfinite(array)):
            raise ValueError('Empty/nonfinite tracking samples')
        if np.any(np.diff(array[:, 0]) < 0):
            raise ValueError('Nonmonotonic recording timestamps')
        array[:, 0] -= start
        return array
    return select(positions), select(attitudes), mission, end - start


def euler_pairs(attitude):
    # The flatness controller constructs Rz(yaw) Rx(roll) Ry(pitch).
    # ZXY avoids the hover pitch=90-degree singularity of conventional ZYX.
    def convert(q_wxyz):
        return Rotation.from_quat(q_wxyz[:, [1, 2, 3, 0]]).as_euler('ZXY')
    actual = np.unwrap(convert(attitude[:, 1:5]), axis=0)
    reference = np.unwrap(convert(attitude[:, 5:9]), axis=0)
    reference += 2 * np.pi * np.round((actual[0] - reference[0]) / (2 * np.pi))
    return np.rad2deg(actual[:, [1, 2, 0]]), np.rad2deg(reference[:, [1, 2, 0]])


def decorate(axis, lap_time, duration):
    axis.grid(alpha=0.18)
    axis.spines[['top', 'right']].set_visible(False)
    axis.set_xlim(0, duration)
    for t in np.arange(lap_time, duration - 0.05, lap_time):
        axis.axvline(t, color='#AAB4BE', linestyle=':', linewidth=0.9)


def plot_errors(axes, p, lap_time, duration):
    error = p[:, 1:4] - p[:, 4:7]
    for i, (axis, label, color) in enumerate(zip(axes, ['North / x', 'East / y', 'Down / z'], [ACTUAL, '#5570B4', '#9C5C9F'])):
        axis.plot(p[:, 0], error[:, i], color=color, linewidth=1.1)
        axis.axhline(0, color='#6A747F', linewidth=0.7)
        axis.set_ylabel(f'{label}\nerror (m)')
        axis.text(0.985, 0.91, f'RMSE {np.sqrt(np.mean(error[:, i] ** 2)):.3f} m',
                  transform=axis.transAxes, ha='right', va='top', fontsize=9,
                  bbox=dict(facecolor='white', edgecolor='none', alpha=0.8))
        decorate(axis, lap_time, duration)
    axes[0].set_title('Position tracking errors: actual minus reference', loc='left', pad=12)
    axes[-1].set_xlabel('Tracking time (s)')


def plot_speed(axis, p, lap_time, duration):
    axis.plot(p[:, 0], np.linalg.norm(p[:, 7:10], axis=1), color=ACTUAL, label='PX4 estimate', linewidth=1.2)
    axis.plot(p[:, 0], np.linalg.norm(p[:, 10:13], axis=1), color=REFERENCE, linestyle='--', label='Reference', linewidth=1.5)
    axis.set(xlabel='Tracking time (s)', ylabel='Speed magnitude (m/s)')
    axis.set_title('Velocity magnitude', loc='left', pad=12)
    axis.legend(loc='best')
    decorate(axis, lap_time, duration)


def plot_attitude(axes, att, actual, reference, lap_time, duration):
    for i, (axis, label) in enumerate(zip(axes, ['Roll', 'Pitch', 'Yaw (unwrapped)'])):
        axis.plot(att[:, 0], actual[:, i], color=ACTUAL, label='Actual', linewidth=1.1)
        axis.plot(att[:, 0], reference[:, i], color=REFERENCE, label='Controller target', linestyle='--', linewidth=1.1)
        axis.set_ylabel(f'{label}\n(deg)')
        decorate(axis, lap_time, duration)
    axes[0].set_title('TS attitude tracking: intrinsic Z-X-Y convention', loc='left', pad=12)
    axes[0].legend(loc='upper right', fontsize=8)
    axes[-1].set_xlabel('Tracking time (s)')


def plot_3d(axis, p, height, origin_down=None):
    # Keep x/y relative to the first tracking reference, with altitude above takeoff.
    origin_xy = p[0, 4:6]
    actual = p[:, 1:4].copy()
    reference = p[:, 4:7].copy()
    actual[:, :2] -= origin_xy
    reference[:, :2] -= origin_xy
    if origin_down is None:
        origin_down = p[0, 6] + height
    actual[:, 2] = origin_down - actual[:, 2]
    reference[:, 2] = origin_down - reference[:, 2]
    axis.plot(*actual.T, color=ACTUAL, label='PX4 estimate', linewidth=1.1)
    axis.plot(*reference.T, color=REFERENCE, label='Reference', linestyle='--', linewidth=1.5)
    axis.scatter(*actual[0], color=ACTUAL, s=24)
    axis.set(xlabel='Relative north (m)', ylabel='Relative east (m)', zlabel='Altitude above takeoff (m)')
    axis.set_title('3D position tracking', loc='left', pad=12)
    both = np.vstack((actual, reference))
    span = np.maximum(np.ptp(both, axis=0), [1, 1, 0.5])
    # Give the altitude discrepancy enough display space; explicitly mark exaggeration.
    axis.set_box_aspect([span[0], span[1], 0.6 * max(span[:2])])
    axis.view_init(elev=27, azim=-54)
    axis.legend(loc='upper right', fontsize=8)
    axis.text2D(0.01, 0.01, 'Vertical display scale exaggerated', transform=axis.transAxes, fontsize=8, color='#66727D')


def save(fig, out, name):
    fig.savefig(out / f'{name}.png', dpi=190, facecolor='white')
    fig.savefig(out / f'{name}.svg', facecolor='white')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--cached-samples', action='store_true',
                        help='Regenerate figures from this directory\'s verified summary/NPZ')
    args = parser.parse_args()
    out = args.output_dir
    out.mkdir(parents=True, exist_ok=True)
    if args.cached_samples:
        stored = json.loads((out / 'summary.json').read_text())
        if Path(stored['bag']).resolve() != args.bag.resolve():
            raise ValueError('Cached summary belongs to a different bag')
        samples = np.load(out / 'tracking_samples.npz')
        p, att = samples['position_debug'], samples['attitude_debug']
        mission, duration = stored['mission'], stored['tracking_duration_s']
    else:
        p, att, mission, duration = read_tracking(args.bag)
    if mission is None:
        raise ValueError('Missing mission result; no verified metadata')
    actual_angles, reference_angles = euler_pairs(att)
    lap_time = mission['lap_time_s']
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10,
                         'axes.titleweight': 'semibold', 'axes.labelcolor': '#364453',
                         'text.color': '#253445', 'axes.edgecolor': '#B9C2CB',
                         'svg.fonttype': 'none'})
    trajectory_name = mission.get('trajectory_name', 'lemniscate')
    coverage = min(1.0, duration / mission.get(
        'reference_duration_s', mission['lap_time_s'] * mission['laps']))
    status = 'ABORTED' if mission.get('aborted') else 'COMPLETED'
    title = (f"Phoenix tailsitter | {trajectory_name} | {mission['speed_m_s']:g} m/s | tracking phase\n"
             f"{status} | coverage {coverage:.1%} of original {mission['laps']} planned laps")
    origin_down = mission.get('takeoff_origin_ned_m', [None, None, None])[2]
    fig, axes = plt.subplots(3, 1, figsize=(11, 7.5), sharex=True, layout='constrained')
    fig.suptitle(title, fontsize=14)
    plot_errors(axes, p, lap_time, duration)
    save(fig, out, 'position_errors')
    fig, axis = plt.subplots(figsize=(11, 4), layout='constrained')
    fig.suptitle(title, fontsize=14)
    plot_speed(axis, p, lap_time, duration)
    save(fig, out, 'speed_magnitude')
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True, layout='constrained')
    fig.suptitle(title, fontsize=14)
    plot_attitude(axes, att, actual_angles, reference_angles, lap_time, duration)
    save(fig, out, 'attitude_tracking')
    fig = plt.figure(figsize=(10, 8), layout='constrained')
    fig.suptitle(title, fontsize=14)
    plot_3d(fig.add_subplot(projection='3d'), p, mission['takeoff_height_m'], origin_down)
    save(fig, out, 'trajectory_3d')
    fig = plt.figure(figsize=(16, 12.5), layout='constrained')
    grid = fig.add_gridspec(2, 2, height_ratios=[1.5, 1.2])
    left = grid[0, 0].subgridspec(3, 1)
    right = grid[0, 1].subgridspec(3, 1)
    plot_errors([fig.add_subplot(left[i]) for i in range(3)], p, lap_time, duration)
    plot_attitude([fig.add_subplot(right[i]) for i in range(3)], att, actual_angles, reference_angles, lap_time, duration)
    plot_speed(fig.add_subplot(grid[1, 0]), p, lap_time, duration)
    plot_3d(fig.add_subplot(grid[1, 1], projection='3d'), p, mission['takeoff_height_m'], origin_down)
    fig.suptitle(title + '\nPosition and velocity: PX4 estimates | Attitude: same-cycle actual/controller target', fontsize=15)
    save(fig, out, 'tracking_overview')
    error = p[:, 1:4] - p[:, 4:7]
    quaternion_actual = Rotation.from_quat(att[:, 1:5][:, [1, 2, 3, 0]])
    quaternion_reference = Rotation.from_quat(att[:, 5:9][:, [1, 2, 3, 0]])
    rotation_error_deg = np.rad2deg((quaternion_actual.inv() * quaternion_reference).magnitude())
    summary = {'bag': str(args.bag.resolve()), 'mission': mission,
               'tracking_duration_s': duration, 'position_samples': len(p), 'attitude_samples': len(att),
               'position_axis_rmse_m': np.sqrt(np.mean(error ** 2, axis=0)).tolist(),
               'position_rmse_m': float(np.sqrt(np.mean(np.sum(error ** 2, axis=1)))),
               'position_peak_m': float(np.max(np.linalg.norm(error, axis=1))),
               'position_mean_error_ned_m': np.mean(error, axis=0).tolist(),
               'velocity_magnitude_rmse_m_s': float(np.sqrt(np.mean((np.linalg.norm(p[:, 7:10], axis=1) - np.linalg.norm(p[:, 10:13], axis=1)) ** 2))),
               'velocity_vector_rmse_m_s': float(np.sqrt(np.mean(np.sum((p[:, 7:10] - p[:, 10:13]) ** 2, axis=1)))),
               'tracking_speed_peak_m_s': float(np.max(np.linalg.norm(p[:, 7:10], axis=1))),
               'attitude_geodesic_rmse_deg': float(np.sqrt(np.mean(rotation_error_deg ** 2))),
               'attitude_geodesic_peak_deg': float(np.max(rotation_error_deg)),
               'tracking_coverage_fraction': coverage,
               'position_error_convention': 'actual minus reference, NED',
               'attitude_convention': 'TS-to-NED intrinsic ZXY, yaw/roll/pitch; plot order roll/pitch/yaw',
               'source': 'Headless SITL run; PX4 estimates, not Gazebo ground truth'}
    (out / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    np.savez_compressed(out / 'tracking_samples.npz', position_debug=p, attitude_debug=att,
                        actual_euler_deg=actual_angles, reference_euler_deg=reference_angles)
    np.savetxt(out / 'position_tracking.csv', np.c_[p[:, :13], error], delimiter=',',
               header='time_s,n_m,e_m,d_m,n_ref_m,e_ref_m,d_ref_m,vn_m_s,ve_m_s,vd_m_s,vn_ref_m_s,ve_ref_m_s,vd_ref_m_s,error_n_m,error_e_m,error_d_m', comments='')
    np.savetxt(out / 'attitude_tracking.csv', np.c_[att[:, 0], actual_angles, reference_angles], delimiter=',',
               header='time_s,roll_deg,pitch_deg,yaw_unwrapped_deg,roll_ref_deg,pitch_ref_deg,yaw_ref_unwrapped_deg', comments='')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
