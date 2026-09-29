#!/usr/bin/env python3
"""Plot recorded PX4 estimator tracking data; run in the ROS 2 environment."""

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read_track(uri):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=uri, storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    types = {item.name: get_message(item.type)
             for item in reader.get_all_topics_and_types()
             if item.name in ('/phoenix_tailsitter/position_debug', '/rosout')}
    rows, phases = [], {}
    result = None
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic not in ('/phoenix_tailsitter/position_debug', '/rosout'):
            continue
        message = deserialize_message(data, types[topic])
        if topic.endswith('/position_debug'):
            rows.append([timestamp * 1e-9, *message.data])
        elif message.name == 'phoenix_lemniscate_mission_test':
            if message.msg.startswith('LEMNISCATE_PHASE '):
                phases[message.msg.split()[-1]] = (
                    message.stamp.sec + message.stamp.nanosec * 1e-9)
            elif message.msg.startswith('LEMNISCATE_MISSION_RESULT '):
                result = json.loads(message.msg.split(' ', 1)[1])
    if result is None or 'track' not in phases:
        raise ValueError(f'Missing completed mission or track phase: {uri}')
    start = phases['track']
    end = min(value for key, value in phases.items()
              if key in ('exit', 'abort_brake', 'land') and value > start)
    rows = np.asarray(rows)
    rows = rows[(rows[:, 0] >= start) & (rows[:, 0] < end)]
    if len(rows) == 0:
        raise ValueError(f'No tracking samples: {uri}')
    values = rows[:, 1:]
    error = values[:, :3] - values[:, 3:6]
    return {
        'bag': uri, 'time': rows[:, 0] - start, 'position': values[:, :3],
        'reference': values[:, 3:6], 'error': np.linalg.norm(error, axis=1),
        'height': result['takeoff_height_m'] - error[:, 2],
        'mission': result,
        'debug_rms_m': float(np.sqrt(np.mean(np.sum(error**2, axis=1)))),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', required=True)
    parser.add_argument('--improved-single', required=True)
    parser.add_argument('--improved-multilap', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    baseline = read_track(args.baseline)
    improved = read_track(args.improved_single)
    multilap = read_track(args.improved_multilap)
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), layout='constrained')
    colors = ['#ba4343', '#007f78']
    for item, label, color in zip(
            [baseline, improved], ['Baseline (1 lap)', 'Improved (1 lap)'], colors):
        origin = item['reference'][0]
        relative = item['position'] - origin
        axes[0, 0].plot(relative[:, 1], relative[:, 0], color=color, label=label)
        axes[0, 1].plot(item['time'], item['error'], color=color, label=label)
    relative_reference = improved['reference'] - improved['reference'][0]
    axes[0, 0].plot(relative_reference[:, 1], relative_reference[:, 0],
                    '--', color='#404040', label='Reference')
    axes[0, 0].set(xlabel='Relative east (m)', ylabel='Relative north (m)',
                   title='5 m/s lemniscate: single-lap comparison')
    axes[0, 0].set_aspect('equal', adjustable='datalim')
    axes[0, 1].set(xlabel='Tracking time (s)', ylabel='Position error norm (m)',
                   title='Single-lap error (high-rate debug samples)')
    axes[1, 0].plot(multilap['time'], multilap['error'], color=colors[1])
    axes[1, 0].set(xlabel='Tracking time (s)', ylabel='Position error norm (m)',
                   title='Improved configuration: three consecutive laps')
    axes[1, 1].plot(multilap['time'], multilap['height'], color='#6e5494',
                    label='PX4 estimated height relative to takeoff origin')
    axes[1, 1].axhline(multilap['mission']['takeoff_height_m'],
                       color='#404040', linestyle='--', label='Target height')
    axes[1, 1].set(xlabel='Tracking time (s)', ylabel='Height (m)',
                   title='Residual altitude bias (not Gazebo ground truth)')
    for axis in axes[1]:
        for lap in range(1, multilap['mission']['laps']):
            axis.axvline(lap * multilap['mission']['lap_time_s'],
                         color='#999999', linestyle=':', linewidth=1)
    for axis in axes.flat:
        axis.grid(alpha=0.2)
    for axis in (axes[0, 0], axes[0, 1], axes[1, 1]):
        axis.legend(fontsize=8)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=160)
    plt.close(fig)
    summaries = [{key: item[key] for key in ('bag', 'mission', 'debug_rms_m')}
                 for item in (baseline, improved, multilap)]
    args.output.with_suffix('.json').write_text(
        json.dumps(summaries, indent=2) + '\n', encoding='utf-8')


if __name__ == '__main__':
    main()
