#!/usr/bin/env python3
"""Compare SITL variants against the same continuous analytic trajectory.

Usage: --run label=/absolute/suite/directory (repeat); each directory may contain
one or more trajectory case folders. The baseline is the selected original suite.
"""
import argparse
import json
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/phoenix_improvement_matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from analyze_tracking_oscillations import read_extra, hold, interp, band_rms, FS
from phoenix_tailsitter_control.paper_trajectory_mission import make_reference
from phoenix_tailsitter_control.config import PhoenixHoverConfig

NAMES = ['segments', 'lemniscate', 'circle-coordinated',
         'transition-to-forward', 'transition-to-hover', 'knife-edge-transition',
         'circular-knife-edge', 'differential-turn']
TEAL, ORANGE = '#007F86', '#E68A2E'


def read_diagnostics(folder, epoch):
    topics = {
        '/phoenix_tailsitter/timing_debug': 'timing',
        '/phoenix_tailsitter/reference_debug': 'reference',
        '/phoenix_tailsitter/actuator_feedback_debug': 'feedback',
    }
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(folder / 'bag'), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()
             if t.name in topics}
    rows = {key: [] for key in topics.values()}
    if not types:
        return {}
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        if topic in types and stamp * 1e-9 >= epoch:
            msg = deserialize_message(raw, types[topic])
            rows[topics[topic]].append([stamp * 1e-9 - epoch, *msg.data])
    return {key: np.array(value) for key, value in rows.items() if value}


def load_case(folder, cache, label):
    cfg = PhoenixHoverConfig()
    summary = json.loads((folder / 'summary.json').read_text())
    mission = summary['mission']
    name = mission['trajectory_name']
    z = np.load(folder / 'tracking_samples.npz')
    p, att = z['position_debug'], z['attitude_debug']
    extra = read_extra(folder, cache / f'{label}_{name}_extra.npz')
    diagnostics = read_diagnostics(folder, float(extra['phase_start_epoch_s']))
    duration = min(summary['tracking_duration_s'], mission['reference_duration_s'])
    original = make_reference(name)
    # Older aborted missions reused origin for landing and reported that final
    # value. Reconstruct the constant scored translation from recorded references.
    sp = extra['sp']
    sp = sp[(sp[:, 1] >= 0) & (sp[:, 1] <= mission['reference_duration_s'])]
    reference_at_publish = np.array([original.sample(float(t))[0] for t in sp[:, 1]])
    translation = np.median(sp[:, 2:5] - reference_at_publish, axis=0)
    translation_spread = np.linalg.norm(sp[:, 2:5] - reference_at_publish - translation, axis=1)
    if np.quantile(translation_spread, .95) > .03:
        raise ValueError(f'Recorded reference does not match the original trajectory: {folder}')
    samples = [original.sample(float(t)) for t in p[:, 0]]
    continuous_p = np.array([s[0] + translation for s in samples])
    continuous_v = np.array([s[1] for s in samples])
    e_p = p[:, 1:4] - continuous_p
    e_v = p[:, 7:10] - continuous_v
    full = p[:, 0] <= duration
    windows = {'segments': (28., 33.), 'lemniscate': (14., 55.95),
               'circle-coordinated': (3., 10.8)}
    start, end = windows.get(name, (.1, duration - .02))
    if end > duration or start >= duration:
        start, end = .1, duration - .02
    grid = np.arange(max(start, p[0, 0], att[0, 0], extra['control'][0, 0]),
                     min(end, p[-1, 0], att[-1, 0], extra['control'][-1, 0]), 1 / FS)
    if len(grid) < 100:
        raise ValueError(f'Insufficient comparison window: {folder}')
    control = hold(extra['control'][:, 0], extra['control'][:, 1:], grid)
    p_grid = hold(p[:, 0], p[:, 1:], grid)
    error_grid = hold(p[:, 0], e_p, grid)
    actual = interp(att[:, 0], z['actual_euler_deg'], grid)
    target = interp(att[:, 0], z['reference_euler_deg'], grid)
    qa = Rotation.from_quat(att[:, 1:5][:, [1, 2, 3, 0]])
    qt = Rotation.from_quat(att[:, 5:9][:, [1, 2, 3, 0]])
    q_error = np.rad2deg((qa.inv() * qt).magnitude())
    att_window = (att[:, 0] >= start) & (att[:, 0] < end)
    metrics = {
        'folder': str(folder.resolve()), 'status': 'aborted' if mission['aborted'] else 'completed',
        'coverage': summary['tracking_coverage_fraction'], 'window_s': [start, end],
        'reconstructed_reference_translation_ned_m': translation.tolist(),
        'recorded_reference_translation_p95_residual_m': float(np.quantile(translation_spread, .95)),
        'position_rmse_continuous_m': float(np.sqrt(np.mean(np.sum(e_p[full]**2, axis=1)))),
        'position_peak_continuous_m': float(np.linalg.norm(e_p[full], axis=1).max()),
        'velocity_vector_rmse_continuous_m_s': float(np.sqrt(np.mean(np.sum(e_v[full]**2, axis=1)))),
        'original_debug_position_rmse_m': summary['position_rmse_m'],
        'position_20hz_rms_m': band_rms(error_grid, 18, 22),
        'force_command_20hz_rms_n': band_rms(p_grid[:, 24:27], 18, 22),
        'target_pitch_20hz_rms_deg': band_rms(target[:, 1], 18, 22),
        'actual_pitch_20hz_rms_deg': band_rms(actual[:, 1], 18, 22),
        'motor_commands_20hz_rms_normalized': band_rms(control[:, 21:23], 18, 22),
        'flap_commands_20hz_rms_deg': band_rms(np.rad2deg(control[:, 23:25]), 18, 22),
        'angular_acceleration_saturation_fraction': np.mean(
            np.abs(control[:, 9:12]) >= cfg.angular_acceleration_limit * .999, axis=0).tolist(),
        'window_position_rmse_continuous_m': float(np.sqrt(np.mean(np.sum(error_grid**2, axis=1)))),
        'window_attitude_error_rms_deg': float(np.sqrt(np.mean(q_error[att_window]**2))),
        'window_speed_peak_m_s': float(np.linalg.norm(p_grid[:, 6:9], axis=1).max()),
        'window_speed_rms_m_s': float(np.sqrt(np.mean(np.sum(p_grid[:, 6:9]**2, axis=1)))),
        'window_pitch_minmax_deg': [float(actual[:, 1].min()), float(actual[:, 1].max())],
    }
    if name == 'segments':
        tail = grid >= 30.0
        tail_error = error_grid[tail]
        metrics['hover_last_3s_position_rmse_m'] = float(np.sqrt(np.mean(np.sum(tail_error**2, axis=1))))
        metrics['hover_last_3s_speed_rms_m_s'] = float(np.sqrt(np.mean(np.sum(p_grid[tail, 6:9]**2, axis=1))))
        metrics['hover_last_3s_pitch_minmax_deg'] = [float(actual[tail, 1].min()), float(actual[tail, 1].max())]
        last_second = grid >= 32.0
        metrics['hover_last_1s_speed_rms_m_s'] = float(np.sqrt(np.mean(np.sum(p_grid[last_second, 6:9]**2, axis=1))))
    for key, rows in diagnostics.items():
        window = rows[(rows[:, 0] >= start) & (rows[:, 0] < end)]
        if not len(window):
            continue
        if key == 'timing':
            metrics['control_dt_mean_ms'] = float(np.mean(window[:, 1]) * 1000)
            metrics['control_dt_max_ms'] = float(window[:, 3].max() * 1000)
            metrics['filter_dt_mean_ms'] = float(window[:, 4].mean() * 1000)
            metrics['state_arrival_age_mean_ms_rates_local_actuator'] = (
                window[:, 5:8].mean(axis=0) * 1000).tolist()
        elif key == 'reference':
            metrics['prediction_age_mean_ms'] = float(window[:, 1].mean() * 1000)
            metrics['reference_same_clock_fraction'] = float(window[:, 2].mean())
            metrics['applied_gain_blend_minmax'] = [float(window[:, 5].min()), float(window[:, 5].max())]
        elif key == 'feedback':
            metrics['actuator_measured_feedback_fraction'] = float(window[:, 15].mean())
            metrics['actuator_prediction_motor_rmse_rad_s'] = np.sqrt(np.mean(window[:, 5:7]**2, axis=0)).tolist()
            metrics['actuator_prediction_flap_rmse_deg'] = np.rad2deg(np.sqrt(np.mean(window[:, 11:13]**2, axis=0))).tolist()
    return metrics, dict(p=p, error=e_p, grid=grid, actual=actual, target=target,
                         speed=np.linalg.norm(p_grid[:, 6:9], axis=1), control=control,
                         diagnostics=diagnostics)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, default=Path(__file__).parent / 'paper_suite_20261004_baseline')
    parser.add_argument('--run', action='append', required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    runs = {'baseline': args.baseline}
    for spec in args.run:
        label, path = spec.split('=', 1)
        if label == 'baseline' or '/' in label:
            raise ValueError('Invalid comparison label')
        runs[label] = Path(path)
    names = [name for name in NAMES if any((root / name / 'case_result.json').exists()
             for label, root in runs.items() if label != 'baseline')]
    metrics, data = {}, {}
    for label, root in runs.items():
        metrics[label], data[label] = {}, {}
        for name in names:
            folder = root / name
            if not (folder / 'summary.json').exists():
                if (folder / 'case_result.json').exists():
                    case = json.loads((folder / 'case_result.json').read_text())
                    metrics[label][name] = {'folder': str(folder.resolve()),
                        'status': case['status'], 'coverage': 0.,
                        'position_rmse_continuous_m': None,
                        'target_pitch_20hz_rms_deg': None,
                        'motor_commands_20hz_rms_normalized': None,
                        'angular_acceleration_saturation_fraction': [None]*3,
                        'note': 'No scored tracking samples; inspect mission.log'}
                continue
            m, d = load_case(folder, args.output_dir, label)
            metrics[label][name], data[label][name] = m, d
            print(label, name, json.dumps(m), flush=True)
    (args.output_dir / 'metrics.json').write_text(json.dumps(metrics, indent=2) + '\n')
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10, 'svg.fonttype': 'none',
                         'axes.spines.top': False, 'axes.spines.right': False})
    labels = list(runs)
    colors = ['#E68A2E', '#5570B4', '#9C5C9F', '#8C6D31', '#007F86', '#C94C4C', '#607D8B', '#555555']
    for name in names:
        fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True, layout='constrained')
        for label, color in zip(labels, colors):
            if name not in data[label]:
                continue
            d = data[label][name]
            if name == 'segments':
                axes[0].plot(d['grid'], d['actual'][:, 1], label=label, color=color)
                axes[1].plot(d['grid'], np.interp(d['grid'], d['p'][:, 0], d['error'][:, 0]), label=label, color=color)
                axes[2].plot(d['grid'], d['speed'], label=label, color=color)
            else:
                start = 30. if name == 'lemniscate' else 6. if name == 'circle-coordinated' else .6
                zoom = (d['grid'] >= start) & (d['grid'] <= start + .6)
                axes[0].plot(d['grid'][zoom], d['target'][zoom, 1], label=label, color=color)
                axes[1].plot(d['grid'][zoom], d['control'][zoom, 21], label=label, color=color)
                axes[2].plot(d['grid'][zoom], np.rad2deg(d['control'][zoom, 23]), label=label, color=color)
        ylabels = ['Actual TS pitch (deg)', 'North error vs analytic reference (m)', 'Actual speed (m/s)'] if name == 'segments' else ['Target TS pitch (deg)', 'Left motor command (normalized)', 'Left flap command (deg)']
        for axis, ylabel in zip(axes, ylabels):
            axis.set_ylabel(ylabel)
            axis.grid(alpha=.2)
            axis.legend(fontsize=9)
        axes[-1].set_xlabel('Tracking time (s)')
        fig.suptitle(f'{name}: independent SITL runs, same original trajectory', fontsize=14)
        fig.savefig(args.output_dir / f'{name}_comparison.png', dpi=175)
        fig.savefig(args.output_dir / f'{name}_comparison.svg')
        plt.close(fig)
    rows = ['# 控制改进对照', '',
            '所有位置、速度统计统一对原连续解析轨迹计算；正式轨迹的源码、速度、圈数和时长保持相同，只有固定位置平移。正式跟踪开始之后计分，准备与退出排除；新版地面起飞分段在解锁后直接开始原轨迹，去掉旧版额外的地面等待。独立 SITL 单次运行，尚不等同于多随机种子的鲁棒性验收。', '',
            '| 轨迹 | 版本 | 完成/覆盖 | 连续参考位置 RMS m | 目标俯仰 18–22 Hz RMS ° | 电机指令 18–22 Hz RMS | 分析窗口俯仰限幅比例 |',
            '|---|---|---|---|---|---|---|']
    for name in names:
        for label in labels:
            if name in metrics[label]:
                m = metrics[label][name]
                def number(value, spec):
                    return '—' if value is None else format(value, spec)
                rows.append(f"| {name} | {label} | {m['status']}/{m['coverage']:.1%} | "
                    f"{number(m['position_rmse_continuous_m'], '.3f')} | {number(m['target_pitch_20hz_rms_deg'], '.4f')} | "
                    f"{number(m['motor_commands_20hz_rms_normalized'], '.5f')} | {number(m['angular_acceleration_saturation_fraction'][1], '.1%')} |")
    rows += ['', '短时、非平稳转场的频谱仅用于纹波比较；中止轨迹的数值仅覆盖实际飞过的片段，不能直接按 RMS 排名。新录包包含真实 control tick 时序、原始角速度和关节反馈；metrics.json 中列出滤波 dt、消息龄期、执行器预测误差与实际反馈占比。', '',
             '控制参数可独立关闭以复现旧控制路径：trajectory_prediction_enabled、motion_gain_scheduling_enabled、use_measured_control_dt、world_force_filter_enabled、transport_feedforward_rates_enabled。预测使用同钟发布时刻，异钟或无时间戳则用接收时刻，推进 p/v/a/yaw 并保留原控制 mask，预测上限 0.10 s。移动参考和静止参考使用同一个平滑增益权重，位置反馈上限也随之过渡。完整模型力先旋转到 NED，再按测量加速度相同的截止和实测 dt 滤波；名义参考角速度前馈通过 R_current.T @ R_reference 转到当前机体系。', '',
             '原始控制曲线未作额外绘图平滑。18–22 Hz 指标由相同采样与频带算法提取；消息到达抖动、状态采样保持和测量噪声仍可能产生剩余纹波。状态与控制均来自 PX4/ROS 录包，未用 Gazebo 真值替代。']
    (args.output_dir / 'README.md').write_text('\n'.join(rows) + '\n')


if __name__ == '__main__':
    main()
