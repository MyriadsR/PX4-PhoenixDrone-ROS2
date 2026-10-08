#!/usr/bin/env python3
"""Diagnose existing SITL recordings without changing the controller or scores.

Run after sourcing ROS and the workspace. Spectra use a 400 Hz uniform grid,
zero-order hold for digital debug streams, and linear interpolation for Euler
angles. The continuous-reference comparison evaluates the original analytic
trajectory at recording time; it is a diagnostic, not a replacement score.
"""
import argparse
import json
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/phoenix_oscillation_matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.signal import welch
from scipy.spatial.transform import Rotation

from phoenix_tailsitter_control.paper_trajectory_mission import make_reference
from phoenix_tailsitter_control.config import PhoenixHoverConfig

CASES = ['segments', 'lemniscate', 'circle-coordinated',
         'transition-to-forward', 'transition-to-hover']
DEBUG = {'control': '/phoenix_tailsitter/control_debug',
         'alpha': '/phoenix_tailsitter/alpha_model_debug'}
SP = '/fmu/in/trajectory_setpoint'
LOCAL = '/fmu/out/vehicle_local_position'
ATT = '/fmu/out/vehicle_attitude'
FS = 400.0
TEAL, ORANGE, BLUE = '#007F86', '#E68A2E', '#5570B4'


def read_extra(folder, cache):
    if cache.exists():
        z = np.load(cache)
        return {key: z[key] for key in z.files}
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(folder / 'bag'), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    wanted = set(DEBUG.values()) | {SP, LOCAL, ATT, '/rosout'}
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()
             if t.name in wanted}
    rows = {key: [] for key in ['control', 'alpha', 'sp', 'local', 'att']}
    reverse = {v: k for k, v in DEBUG.items()}
    start, end = None, None
    while reader.has_next():
        topic, data, ns = reader.read_next()
        if topic not in types:
            continue
        message = deserialize_message(data, types[topic])
        t = ns * 1e-9
        if topic in reverse:
            rows[reverse[topic]].append([t, *message.data])
        elif topic == SP:
            rows['sp'].append([t, message.timestamp * 1e-6,
                *message.position, *message.velocity, *message.acceleration,
                *message.jerk, message.yaw, message.yawspeed])
        elif topic == LOCAL:
            rows['local'].append([t, message.timestamp * 1e-6,
                message.timestamp_sample * 1e-6,
                message.x, message.y, message.z, message.vx, message.vy, message.vz,
                message.ax, message.ay, message.az])
        elif topic == ATT:
            rows['att'].append([t, message.timestamp * 1e-6,
                message.timestamp_sample * 1e-6, *message.q])
        elif message.name == 'phoenix_lemniscate_mission_test':
            t_phase = message.stamp.sec + message.stamp.nanosec * 1e-9
            if message.msg == 'LEMNISCATE_PHASE track':
                start = t_phase
            elif start is not None and end is None and message.msg in (
                    'LEMNISCATE_PHASE exit', 'LEMNISCATE_PHASE abort_brake',
                    'LEMNISCATE_PHASE land'):
                end = t_phase
    if start is None or end is None:
        raise ValueError('Missing phase boundaries')
    result = {}
    for key, values in rows.items():
        array = np.asarray(values)
        # Keep one pre-track setpoint for the initial sample hold.
        mask = (array[:, 0] >= start - (0.10 if key == 'sp' else 0)) & (array[:, 0] < end)
        array = array[mask]
        array[:, 0] -= start
        if key == 'sp':
            array[:, 1] -= start
        result[key] = array
    result['phase_start_epoch_s'] = np.array(start)
    np.savez_compressed(cache, **result)
    return result


def hold(times, values, grid):
    indices = np.clip(np.searchsorted(times, grid, side='right') - 1, 0, len(times) - 1)
    return values[indices]


def interp(times, values, grid):
    return np.column_stack([np.interp(grid, times, column) for column in values.T])


def spectrum(values):
    if values.ndim == 1:
        values = values[:, None]
    f, power = welch(values, fs=FS, nperseg=min(len(values), int(8 * FS)),
                     detrend='linear', axis=0)
    return f, power.sum(axis=1)


def band_rms(values, low, high):
    f, power = spectrum(values)
    mask = (f >= low) & (f <= high)
    return float(np.sqrt(np.trapz(power[mask], f[mask])))


def peak_frequency(values, low, high):
    f, power = spectrum(values)
    mask = (f >= low) & (f <= high)
    return float(f[mask][np.argmax(power[mask])])


def timing(rows, column=0):
    delta = np.diff(rows[:, column])
    delta = delta[delta > 0]
    return {'mean_rate_hz': float(1 / np.mean(delta)),
            'median_interval_ms': float(np.median(delta) * 1000),
            'p99_interval_ms': float(np.quantile(delta, .99) * 1000),
            'max_interval_ms': float(delta.max() * 1000)}


def save(fig, output, name):
    fig.savefig(output / f'{name}.png', dpi=180, facecolor='white')
    fig.savefig(output / f'{name}.svg', facecolor='white')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--suite', type=Path,
        default=Path(__file__).parent / 'paper_suite_20261004_baseline')
    parser.add_argument('--output-dir', type=Path,
        default=Path(__file__).parent / 'oscillation_analysis_20261004')
    args = parser.parse_args()
    output = args.output_dir
    output.mkdir(parents=True, exist_ok=True)
    cfg = PhoenixHoverConfig()
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10,
                         'axes.spines.top': False, 'axes.spines.right': False,
                         'svg.fonttype': 'none'})
    results, data = {}, {}
    for name in CASES:
        folder = args.suite / name
        extra = read_extra(folder, output / f'{name}_extra.npz')
        z = np.load(folder / 'tracking_samples.npz')
        p, att = z['position_debug'], z['attitude_debug']
        summary = json.loads((folder / 'summary.json').read_text())
        mission = summary['mission']
        duration = summary['tracking_duration_s']
        reference = make_reference(name)
        translation = np.array(mission['takeoff_origin_ned_m']) + np.array(
            mission['reference_position_translation_ned_m'])
        analytic_samples = [reference.sample(float(t)) for t in p[:, 0]]
        analytic_position = np.array([s[0] + translation for s in analytic_samples])
        analytic_velocity = np.array([s[1] for s in analytic_samples])
        analytic_acceleration = np.array([s[2] for s in analytic_samples])
        error_held = p[:, 1:4] - p[:, 4:7]
        error_analytic = p[:, 1:4] - analytic_position
        # Prefer long quasi-steady windows; short transitions are nonstationary.
        window = (28.0, 33.0) if name == 'segments' else (
            (14.0, 55.95) if name == 'lemniscate' else (
                (3.0, min(duration - .02, 10.8)) if name == 'circle-coordinated'
                else (.10, duration - .02)))
        grid = np.arange(max(window[0], p[0, 0], extra['control'][0, 0], att[0, 0]),
                         min(window[1], p[-1, 0], extra['control'][-1, 0], att[-1, 0]), 1 / FS)
        control = hold(extra['control'][:, 0], extra['control'][:, 1:], grid)
        alpha = hold(extra['alpha'][:, 0], extra['alpha'][:, 1:], grid)
        position = hold(p[:, 0], p[:, 1:], grid)
        held_error = hold(p[:, 0], error_held, grid)
        continuous_error = hold(p[:, 0], error_analytic, grid)
        angles_actual = interp(att[:, 0], z['actual_euler_deg'], grid)
        angles_desired = interp(att[:, 0], z['reference_euler_deg'], grid)
        q_current = Rotation.from_quat(att[:, 1:5][:, [1, 2, 3, 0]])
        q_target = Rotation.from_quat(att[:, 5:9][:, [1, 2, 3, 0]])
        q_error = np.rad2deg((q_current.inv() * q_target).magnitude())
        q_angular_step = np.rad2deg((q_target[:-1].inv() * q_target[1:]).magnitude())
        in_window = (att[:, 0] >= window[0]) & (att[:, 0] <= window[1])
        active_steps = in_window[1:]
        sp = extra['sp']
        sp_track = sp[sp[:, 1] >= 0]
        sp_analytic = np.array([reference.sample(float(t))[0] + translation
                               for t in sp_track[:, 1]])
        reference_validation_error = np.linalg.norm(sp_analytic - sp_track[:, 2:5], axis=1)
        if len(p) != len(att) or np.max(np.abs(p[:, 0] - att[:, 0])) > .01:
            raise ValueError('Cannot pair same-cycle attitude and position debug')
        # One-step counterfactual: replay the position-feedback equation with
        # the same measured state/rotation/model force, changing only p/v/a
        # references to their continuous values. This is NOT a closed-loop run.
        rotation = q_current.as_matrix()
        def replay_acceleration(p_ref, v_ref, a_ref):
            e_p = np.einsum('nji,nj->ni', rotation, p_ref - p[:, 1:4])
            e_v = np.einsum('nji,nj->ni', rotation, v_ref - p[:, 7:10])
            body_feedback = cfg.tracking_position_gain * e_p + cfg.tracking_velocity_gain * e_v
            world_feedback = np.einsum('nij,nj->ni', rotation, body_feedback)
            norm_xy = np.linalg.norm(world_feedback[:, :2], axis=1)
            world_feedback[:, :2] *= np.minimum(1, cfg.tracking_horizontal_acceleration_limit /
                                                 np.maximum(norm_xy, 1e-12))[:, None]
            world_feedback[:, 2] = np.clip(world_feedback[:, 2],
                -cfg.tracking_vertical_acceleration_limit, cfg.tracking_vertical_acceleration_limit)
            return a_ref + world_feedback
        replay_held = replay_acceleration(p[:, 4:7], p[:, 10:13], p[:, 16:19])
        replay_continuous = replay_acceleration(analytic_position, analytic_velocity, analytic_acceleration)
        replay_force_continuous = p[:, 22:25] + cfg.mass * (replay_continuous - p[:, 13:16])
        force_held = hold(p[:, 0], p[:, 25:28], grid)
        force_continuous = hold(p[:, 0], replay_force_continuous, grid)
        analysis_mask = (p[:, 0] >= window[0]) & (p[:, 0] <= window[1])
        feedback = position[:, 18:21] - position[:, 15:18]
        entire_control = extra['control'][:, 1:]
        def fraction_at_limit(values, limits):
            return np.mean(np.abs(values) >= np.asarray(limits) * .999, axis=0).tolist()
        # Actual position/velocity are held PX4 state values, so repeated values
        # at timer rate must not be counted as independent sensor samples.
        p_changes = np.linalg.norm(np.diff(p[:, 1:4], axis=0), axis=1) > 1e-8
        reference_changes = np.linalg.norm(np.diff(p[:, 4:7], axis=0), axis=1) > 1e-8
        results[name] = {
            'window_s': list(window), 'stationary_window': name not in (
                'transition-to-forward', 'transition-to-hover'),
            'recorded_stream_timing': {key: timing(sp_track if key == 'sp' else value) for key, value in extra.items()
                                      if key != 'phase_start_epoch_s'},
            'local_px4_sample_timing': timing(extra['local'], 2),
            'debug_position_timing': timing(p),
            'debug_position_value_change_rate_hz': float(p_changes.sum() / duration),
            'debug_reference_position_change_rate_hz': float(reference_changes.sum() / duration),
            'analytic_reference_validation_max_m': float(reference_validation_error.max()),
            'analytic_reference_validation_median_m': float(np.median(reference_validation_error)),
            'position_feedback_replay_max_error_window_m_s2': float(np.max(
                np.linalg.norm(replay_held[analysis_mask] - p[analysis_mask, 19:22], axis=1))),
            'force_command_18_22hz_rms_recorded_n': band_rms(force_held, 18, 22),
            'force_command_18_22hz_rms_continuous_reference_one_step_replay_n': band_rms(force_continuous, 18, 22),
            'position_error_18_22hz_rms_held_m': band_rms(held_error, 18, 22),
            'position_error_18_22hz_rms_analytic_m': band_rms(continuous_error, 18, 22),
            'position_error_8_80hz_rms_held_m': band_rms(held_error, 8, 80),
            'position_error_8_80hz_rms_analytic_m': band_rms(continuous_error, 8, 80),
            'position_error_peak_0p2_5hz': peak_frequency(continuous_error, .2, 5),
            'pitch_actual_peak_0p2_5hz': peak_frequency(angles_actual[:, 1], .2, 5),
            'pitch_actual_18_22hz_rms_deg': band_rms(angles_actual[:, 1], 18, 22),
            'pitch_target_18_22hz_rms_deg': band_rms(angles_desired[:, 1], 18, 22),
            'euler_actual_18_22hz_rms_by_axis_deg': [band_rms(angles_actual[:, i], 18, 22) for i in range(3)],
            'euler_target_18_22hz_rms_by_axis_deg': [band_rms(angles_desired[:, i], 18, 22) for i in range(3)],
            'attitude_euler_actual_8_80hz_rms_deg': band_rms(angles_actual, 8, 80),
            'attitude_euler_target_8_80hz_rms_deg': band_rms(angles_desired, 8, 80),
            'rates_8_80hz_rms_rad_s': band_rms(control[:, 3:6], 8, 80),
            'angular_acceleration_8_80hz_rms_rad_s2': band_rms(control[:, 6:9], 8, 80),
            'motor_normalized_commands_18_22hz_rms': band_rms(control[:, 21:23], 18, 22),
            'flap_commands_18_22hz_rms_deg': band_rms(np.rad2deg(control[:, 23:25]), 18, 22),
            'angular_acceleration_limit_fraction_full': fraction_at_limit(
                entire_control[:, 9:12], cfg.angular_acceleration_limit),
            'angular_acceleration_limit_fraction_window': fraction_at_limit(
                control[:, 9:12], cfg.angular_acceleration_limit),
            'moment_limit_fraction_window': fraction_at_limit(control[:, 15:18], cfg.moment_limit),
            'horizontal_feedback_limit_fraction_window': float(np.mean(
                np.linalg.norm(feedback[:, :2], axis=1) >= cfg.tracking_horizontal_acceleration_limit * .999)),
            'actual_speed_minmax_window_m_s': [float(np.linalg.norm(position[:, 6:9], axis=1).min()),
                                             float(np.linalg.norm(position[:, 6:9], axis=1).max())],
            'position_error_north_minmax_window_m': [float(continuous_error[:, 0].min()), float(continuous_error[:, 0].max())],
            'actual_pitch_minmax_window_deg': [float(angles_actual[:, 1].min()), float(angles_actual[:, 1].max())],
            'quaternion_error_rms_window_deg': float(np.sqrt(np.mean(q_error[in_window]**2))),
            'quaternion_target_step_p99_window_deg': float(np.quantile(q_angular_step[active_steps], .99)),
        }
        data[name] = dict(grid=grid, p=p, held_error=held_error,
            continuous_error=continuous_error, control=control, alpha=alpha,
            actual=angles_actual, desired=angles_desired, position=position,
            analytic=analytic_position, error_held=error_held, error_analytic=error_analytic)
        print(name, json.dumps(results[name], ensure_ascii=False), flush=True)
    (output / 'metrics.json').write_text(json.dumps(results, indent=2) + '\n')

    fig, axes = plt.subplots(2, 2, figsize=(13, 8), layout='constrained')
    for column, name in enumerate(['lemniscate', 'circle-coordinated']):
        d = data[name]
        zoom_start = 30 if name == 'lemniscate' else 6
        zoom = (d['p'][:, 0] >= zoom_start) & (d['p'][:, 0] <= zoom_start + .6)
        axis = axes[0, column]
        axis.plot(d['p'][zoom, 0], d['error_held'][zoom, 0], color=ORANGE, label='Error vs held 20 Hz reference')
        axis.plot(d['p'][zoom, 0], d['error_analytic'][zoom, 0], color=TEAL, label='Error vs continuous analytic reference')
        axis.set(title=name, xlabel='Tracking time (s)', ylabel='North position error (m)')
        axis.legend(fontsize=8)
        axis = axes[1, column]
        for label, signal, color in [('Held reference', d['held_error'], ORANGE),
                                      ('Analytic reference', d['continuous_error'], TEAL)]:
            f, power = spectrum(signal)
            axis.semilogy(f, power, label=label, color=color)
        axis.axvline(20, color=BLUE, linestyle=':', linewidth=1)
        axis.set(xlim=(5, 65), xlabel='Frequency (Hz)', ylabel='Position-error PSD sum (m²/Hz)')
        axis.legend(fontsize=8)
    for axis in axes.flat:
        axis.grid(alpha=.2)
    fig.suptitle('20 Hz reference hold explains much of the dense position-error ripple\nSame measured states; only the diagnostic reference evaluation changes', fontsize=13)
    save(fig, output, 'reference_hold_diagnosis')

    fig, axes = plt.subplots(3, 2, figsize=(13, 10), layout='constrained')
    d = data['segments']
    axes[0, 0].plot(d['grid'], d['actual'][:, 1], color=TEAL, label='Actual TS pitch')
    axes[0, 0].plot(d['grid'], d['desired'][:, 1], color=ORANGE, label='Controller target')
    axes[0, 0].set(ylabel='Pitch (deg)', title='segments: final stationary hover (28–33 s)')
    axes[0, 0].legend(fontsize=8)
    axes[1, 0].plot(d['grid'], d['continuous_error'][:, 0], color=BLUE)
    axes[1, 0].set(ylabel='North error (m)')
    axes[2, 0].plot(d['grid'], np.linalg.norm(d['position'][:, 6:9], axis=1), color=TEAL, label='Actual speed')
    axes[2, 0].plot(d['grid'], np.linalg.norm(d['position'][:, 9:12], axis=1), color=ORANGE, label='Reference speed')
    axes[2, 0].set(ylabel='Speed (m/s)', xlabel='Tracking time (s)')
    axes[2, 0].legend(fontsize=8)
    for name, color in [('segments', TEAL), ('lemniscate', BLUE), ('circle-coordinated', ORANGE)]:
        d = data[name]
        f, power = spectrum(d['actual'][:, 1])
        axes[0, 1].semilogy(f, power, label=name, color=color)
        f, power = spectrum(d['desired'][:, 1])
        axes[1, 1].semilogy(f, power, label=name, color=color)
        f, power = spectrum(np.rad2deg(d['control'][:, 23:25]))
        axes[2, 1].semilogy(f, power, label=name, color=color)
    axes[0, 1].set(xlim=(.2, 35), ylabel='Actual pitch PSD (deg²/Hz)', title='Slow oscillation and fast command ripple coexist')
    axes[1, 1].set(xlim=(.2, 35), ylabel='Target pitch PSD (deg²/Hz)')
    axes[2, 1].set(xlim=(.2, 35), ylabel='Flap-command PSD sum (deg²/Hz)', xlabel='Frequency (Hz)')
    for axis in axes[:, 1]:
        axis.axvline(20, color='#888888', linestyle=':', linewidth=1)
        axis.legend(fontsize=8)
    for axis in axes.flat:
        axis.grid(alpha=.2)
    fig.suptitle('Persistent hover oscillation is present in states, not only in plotted reference angles', fontsize=13)
    save(fig, output, 'state_vs_command_diagnosis')

    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True, layout='constrained')
    d = data['circle-coordinated']
    mask = (d['grid'] >= 6) & (d['grid'] <= 6.6)
    t = d['grid'][mask]
    axes[0].plot(t, d['actual'][mask, 1], color=TEAL, label='Actual TS pitch')
    axes[0].plot(t, d['desired'][mask, 1], color=ORANGE, label='Controller target')
    axes[0].set(ylabel='Pitch (deg)')
    axes[1].plot(t, d['position'][mask, 12:15], linewidth=1)
    axes[1].set(ylabel='Filtered a (m/s²)')
    axes[2].plot(t, d['control'][mask, 21:23], linewidth=1)
    axes[2].set(ylabel='Motor commands\n(normalized speed)')
    axes[3].plot(t, np.rad2deg(d['control'][mask, 23:25]), linewidth=1)
    axes[3].set(ylabel='Flap commands (deg)', xlabel='Tracking time (s)')
    # Mark the actual setpoint hold changes visible in position_debug.
    p = d['p']
    changed = np.linalg.norm(np.diff(p[:, 4:7], axis=0), axis=1) > 1e-8
    for edge in p[1:, 0][changed]:
        if t[0] <= edge <= t[-1]:
            for axis in axes:
                axis.axvline(edge, color='#999999', linestyle=':', alpha=.5)
    axes[0].legend(fontsize=9)
    for axis in axes:
        axis.grid(alpha=.2)
    fig.suptitle('circle-coordinated: state/target/actuator-command zoom\nDotted lines: new held trajectory references (about every 50 ms)', fontsize=13)
    save(fig, output, 'command_ripple_zoom')

    report = output / 'README.md'
    lines = ['# 已完成轨迹振荡诊断', '',
             '仅分析现有录包，未修改控制器、轨迹、原始图表或验收统计。', '',
             '| 轨迹 | 分析窗口 s | 位置误差 18–22 Hz RMS：保持参考→连续参考 m | 俯仰 18–22 Hz RMS：实际/目标 ° | 角加速度指令限幅比例 x/y/z |',
             '|---|---|---|---|---|']
    for name, m in results.items():
        lines.append(f"| {name} | {m['window_s'][0]:.2f}–{m['window_s'][1]:.2f} | "
            f"{m['position_error_18_22hz_rms_held_m']:.4f} → {m['position_error_18_22hz_rms_analytic_m']:.4f} | "
            f"{m['pitch_actual_18_22hz_rms_deg']:.3f}/{m['pitch_target_18_22hz_rms_deg']:.3f} | "
            + '/'.join(f'{100 * x:.1f}%' for x in m['angular_acceleration_limit_fraction_window']) + ' |')
    lines += ['', '谱估计：400 Hz 等间距网格，数字量零阶保持，欧拉角线性插值，Welch 分段最长 8 s，线性去趋势。转入/转出仅 3.8 s，非平稳，频率不能解释为稳态固有模态。位置谱为三轴 PSD 之和；姿态使用 TS ZXY 欧拉角，同时保留四元数误差确认。', '',
              '连续参考由同一原始解析轨迹、同一世界平移、同一 track 相位时间计算；metrics.json 记录其在原始参考消息时间戳上的位置校验误差。该对比用于识别保持纹波，不应替换原任务误差。', '',
              '本轮未录 timing_debug、vehicle_angular_velocity 或原始执行器关节反馈；录包接收间隔只能说明消息频率/抖动，不能直接量化控制回调实时周期、IMU 噪声或执行器实际延迟。电机/舵面的图是控制指令，不是实测运动。', '',
              '文件：reference_hold_diagnosis.png、state_vs_command_diagnosis.png、command_ripple_zoom.png（均另有 SVG）；原始派生缓存 *_extra.npz，完整指标 metrics.json。', '',
              '复现：source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash && /usr/bin/python3 ros2_ws/analysis/analyze_tracking_oscillations.py']
    lines += ['', '## 分析结论', '',
        '1. 高速轨迹的密集位置误差锯齿主要来自 20 Hz 参考零阶保持。参考每 50 ms 跳一次，而飞机连续运动：6 m/s 对应一步约 0.30 m，8.1 m/s 对应约 0.405 m。控制 debug 约 463–493 Hz，但位置、速度、姿态估计实际约 100 Hz；debug 更密并不等于新增状态测量。用原解析参考重新计算后，八字和协调圆周的 18–22 Hz 位置误差 RMS 分别从 0.0677/0.0908 m 降至 0.0028/0.0040 m，下降约 96%。解析参考与原参考消息的校验最大误差为毫米量级。剩余的细齿还包含约 100 Hz 状态保持产生的阶梯，不能把所有纹波都称为机体振荡。', '',
        '2. 参考保持也实际影响控制指令。位置环使用保持的 p/v/a，yaw 也直接保持；因此误差跳变会进入加速度、INDI 力指令和姿态目标。对相同实测姿态、位置、速度、加速度和模型力，仅换成连续参考做单步重算，八字/圆周的 18–22 Hz 力指令 RMS 为 0.266→0.0219 N / 0.523→0.0538 N。原位置环重算与记录吻合至数值精度。该离线重算确认即时指令纹波的主要来源，但不是重新运行闭环后的改善效果。目标俯仰的 20 Hz 带 RMS 为 0.302/0.440°，实际俯仰仅 0.0034/0.0018°。图中的 controller target 是反馈修正后的目标，不是纯轨迹前馈姿态。', '',
        '3. segments 最后 28–33 s 的悬停有真实持续振荡：参考位置/速度恒定，北向误差约 −0.64～+0.52 m，实际 TS 俯仰约 60～134°，速度峰值约 1.74 m/s。位置/俯仰谱峰约 0.8 Hz（5 s 窗口分辨率 0.2 Hz），俯仰轴角加速度指令约 81% 时间被 ±15 rad/s² 限幅，水平反馈约 48% 时间触及 3 m/s² 边界。四元数误差 RMS 约 29.2°，排除了仅由欧拉角显示导致的解释。它具有极限环特征；主因候选为高增益、限幅和执行链路延迟的组合，尚未用参数对照实验单独确认。', '',
        '4. 一处明确的增益调度问题是把“任意有限加速度字段”当作机动标记，因此悬停的 [0,0,0] 也启用机动增益。俯仰姿态增益从 6 升到 54.88，水平反馈上限从 0.45 升到 3 m/s²；静止参考与起飞准备时的 NaN 加速度语义不一致。电机/舵面模型具有 40/30 ms 时间常数，加上滤波和状态更新延迟；当前 alpha 路径的线加速度实际使用 15 Hz 截止，配置中的 linear_indi_lpf_cutoff_hz=5 不被该路径使用。角加速度又由滤后角速度每个控制 tick 差分；状态保持和时序抖动会影响这条 INDI 通道。不能仅靠降低某个未使用的配置项解决问题。', '',
        '5. 转入/转出在 0.4 s 和 3.4 s 处切换 ±2.7 m/s² 切向加速度，存在参考加速度阶跃，其 jerk 没有表示这些瞬间阶跃；会激发入口/出口瞬态。转入前飞正式跟踪中俯仰角加速度限幅比例约 36%，应与高频纹波分别处理。八字/圆周随曲率重复的低频起伏也不能一概解释为不稳定振荡。', '',
        '## 建议处理顺序', '',
        '先按消息时间戳把 p/v/a/jerk/yaw 一致地推进到控制时刻，或提高参考更新率；不能只对图表做平滑。然后用显式任务阶段/运动状态区分悬停与机动，恢复适合悬停的增益与反馈边界并做独立对照。随后识别执行器延迟、核对 INDI 滤波相位和真实采样 dt，平滑转场加速度边界。每次只改一项，并同时比较跟踪 RMS、限幅比例、状态/指令频谱；下一轮增加 timing_debug、原始角速度、原始关节反馈和 Gazebo ground truth 录包。', '',
        '控制源码：alpha_controller_node.py 的 171–179、222–249、434–457、495–503 行；position_control.py 的 111–143 行；config.py 的 59–84、97–115 行；reference_trajectories.py 的 CircularTransitionTrajectory.sample。']
    report.write_text('\n'.join(lines) + '\n')


if __name__ == '__main__':
    main()
