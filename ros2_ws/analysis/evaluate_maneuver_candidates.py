#!/usr/bin/env python3
"""Score candidate SITL cases against the current baseline with common windows."""
import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from compare_tracking_improvements import load_case, hold

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_BASELINE = ROOT / 'ros2_ws/analysis/tracking_improvement_20261004/rate_transport'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, default=DEFAULT_BASELINE)
    parser.add_argument('--run', action='append', required=True, help='label=directory')
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    runs = {'baseline': args.baseline}
    for spec in args.run:
        label, path = spec.split('=', 1)
        runs[label] = Path(path)
    names = [name for name in ('knife-edge-transition', 'differential-turn', 'segments',
                              'lemniscate', 'circle-coordinated', 'transition-to-forward',
                              'transition-to-hover')
             if any((root / name / 'summary.json').exists() for label, root in runs.items()
                    if label != 'baseline')]
    metrics, data = {}, {}
    for label, root in runs.items():
        metrics[label], data[label] = {}, {}
        for name in names:
            folder = root / name
            if not (folder / 'summary.json').exists():
                continue
            cache_label = label + '_' + hashlib.sha256(
                str(folder.resolve()).encode()).hexdigest()[:12]
            m, d = load_case(folder, args.out, cache_label)
            case = json.loads((folder / 'case_result.json').read_text())
            scales = {'maneuver_acceleration_limit_scale': 1.0,
                      'maneuver_z_limit_scale': 1.0}
            for argument in case.get('launch_arguments', []):
                key, value = argument.split(':=', 1)
                if key in scales:
                    scales[key] = float(value)
            blend = hold(d['diagnostics']['reference'][:, 0],
                         d['diagnostics']['reference'][:, 5:6], d['grid'])[:, 0]
            axis_scale = np.array([1.0, 1.0, scales['maneuver_z_limit_scale']])
            applied_scale = 1.0 + blend[:, None] * (
                scales['maneuver_acceleration_limit_scale'] * axis_scale - 1.0)
            limit = np.array([20., 15., 24.])[None, :] * applied_scale
            m['angular_acceleration_saturation_fraction'] = np.mean(
                np.abs(d['control'][:, 9:12]) >= limit * .999, axis=0).tolist()
            m['track_completed'] = m['coverage'] >= .999
            m['mission_passed'] = bool(case.get('mission', {}).get('passed', False))
            m['exit_code'] = case.get('exit_code')
            m['abort_reason'] = case.get('abort_reason')
            m['launch_arguments'] = case.get('launch_arguments', [])
            baseline_folder = args.baseline / name
            base_summary = json.loads((baseline_folder / 'summary.json').read_text())
            current_summary = json.loads((folder / 'summary.json').read_text())
            end = min(base_summary['tracking_duration_s'], current_summary['tracking_duration_s'],
                      current_summary['mission']['reference_duration_s'])
            mask = (d['p'][:, 0] >= 0) & (d['p'][:, 0] <= end)
            m['baseline_common_window_end_s'] = end
            m['baseline_common_window_position_rmse_m'] = float(np.sqrt(np.mean(np.sum(d['error'][mask] ** 2, axis=1))))
            metrics[label][name], data[label][name] = m, d
            print(label, name, 'coverage', round(m['coverage'], 4), 'p_rms',
                  round(m['position_rmse_continuous_m'], 3), 'common_rms',
                  round(m['baseline_common_window_position_rmse_m'], 3), flush=True)
    (args.out / 'metrics.json').write_text(json.dumps(metrics, indent=2) + '\n')
    plt.rcParams.update({'font.family': 'Noto Sans CJK JP', 'axes.unicode_minus': False,
                         'svg.fonttype': 'none'})
    colors = ['#666666', *plt.get_cmap('tab10').colors]
    for name in names:
        fig, axes = plt.subplots(3, 1, figsize=(12, 9), layout='constrained')
        for i, (label, _) in enumerate(runs.items()):
            color = colors[i % len(colors)]
            if name not in data[label]:
                continue
            d, m = data[label][name], metrics[label][name]
            stop = m['coverage'] * json.loads((runs[label] / name / 'summary.json').read_text())['mission']['reference_duration_s']
            mask = d['p'][:, 0] <= stop
            axes[0].plot(d['p'][mask, 0], np.linalg.norm(d['error'][mask], axis=1),
                         color=color, lw=1.3, label=f'{label}（覆盖 {100*m["coverage"]:.1f}%）')
            axes[1].plot(d['grid'], d['speed'], color=color, lw=1.1, label=label)
            axes[2].plot(d['grid'], np.linalg.norm(d['control'][:, :3], axis=1) * 180 / np.pi,
                         color=color, lw=1.1, label=label)
        for ax, ylabel in zip(axes, ['位置误差范数 / m', '速度幅值 / m/s', '姿态指令误差 / °']):
            ax.set_ylabel(ylabel)
            ax.grid(alpha=.2)
            ax.legend(fontsize=8)
            ax.set_xlabel('正式跟踪时间 / s')
        fig.suptitle(f'{name}：独立 SITL 对照，原始轨迹保持一致')
        fig.savefig(args.out / f'{name}_comparison.png', dpi=175)
        fig.savefig(args.out / f'{name}_comparison.svg')
        plt.close(fig)


if __name__ == '__main__':
    main()
