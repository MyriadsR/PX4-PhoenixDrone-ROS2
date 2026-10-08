#!/usr/bin/env python3
"""Run original Tailsitter-control profiles sequentially in fresh headless SITL."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import hashlib
import shutil
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/phoenix_tailsitter_control'))
from phoenix_tailsitter_control.maneuver_profiles import (
    CONTROLLER_PROFILES, controller_arguments, mission_arguments as profile_mission_arguments,
)
NAMES = ['segments', 'lemniscate', 'knife-edge-transition', 'circle-coordinated', 'circular-knife-edge', 'transition-to-forward', 'transition-to-hover', 'differential-turn']
TOPICS = ['/phoenix_tailsitter/position_debug', '/phoenix_tailsitter/control_debug', '/phoenix_tailsitter/attitude_tracking_debug', '/phoenix_tailsitter/alpha_model_debug', '/phoenix_tailsitter/timing_debug', '/phoenix_tailsitter/actuator_feedback_debug', '/phoenix_tailsitter/reference_debug', '/fmu/out/vehicle_attitude', '/fmu/out/vehicle_angular_velocity', '/fmu/out/vehicle_local_position', '/fmu/in/trajectory_setpoint', '/world/stars_ts/model/phoenixdrone_alpha_0/joint_state', '/rosout']


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()


def run_case(name, destination, launch_arguments=None, mission_arguments=None,
             controller_source=None):
    destination.mkdir(parents=True, exist_ok=False)
    processes, handles = [], []
    env = os.environ.copy()
    source = str((ROOT / 'ros2_ws/src/phoenix_tailsitter_control')
                 if controller_source is None else Path(controller_source).resolve())
    env['PYTHONPATH'] = source + ':' + env.get('PYTHONPATH', '')
    env['ROS_LOG_DIR'] = str(destination / 'ros_logs')
    env['MPLCONFIGDIR'] = '/tmp/phoenix_paper_mpl'
    def start(args, logfile):
        handle = (destination / logfile).open('w')
        handles.append(handle)
        process = subprocess.Popen(args, cwd=ROOT, env=env, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process
    launch_arguments = [] if launch_arguments is None else list(launch_arguments)
    mission_arguments = [] if mission_arguments is None else list(mission_arguments)
    source_files = list(Path(source).rglob('*.py'))
    for path in source_files:
        saved = destination / 'controller_source' / path.relative_to(source)
        saved.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, saved)
    (destination / 'controller_source' / 'COLCON_IGNORE').touch()
    base_launch_arguments = ['headless:=true', 'output_enabled:=true',
                             'setpoint_source:=trajectory', 'use_measured_control_dt:=true']
    result = {'trajectory_name': name, 'output_dir': str(destination), 'status': 'failed',
              'base_launch_arguments': base_launch_arguments,
              'launch_arguments': launch_arguments,
              'mission_arguments': mission_arguments,
              'controller_source_sha256': {str(p.relative_to(source)): hashlib.sha256(p.read_bytes()).hexdigest()
                  for p in source_files if '__pycache__' not in p.parts}}
    try:
        print(f'[{name}] Starting fresh PX4/Gazebo SITL.', flush=True)
        launch_target = (['phoenix_tailsitter_control', 'tailsitter_alpha_sitl.launch.py']
                         if controller_source is None else
                         [str(Path(source) / 'launch/tailsitter_alpha_sitl.launch.py')])
        sim = start(['ros2', 'launch', *launch_target, *base_launch_arguments, *launch_arguments], 'simulation.log')
        recorder = start(['ros2', 'bag', 'record', '-s', 'mcap', '-o', str(destination / 'bag'), *TOPICS], 'record.log')
        deadline = time.monotonic() + 90
        ready = False
        while time.monotonic() < deadline:
            if sim.poll() is not None:
                raise RuntimeError('SITL exited during initialization')
            if 'Ready for takeoff!' in (destination / 'simulation.log').read_text(errors='replace'):
                ready = True
                break
            time.sleep(1)
        if not ready:
            raise RuntimeError('SITL did not become ready within 90 s')
        time.sleep(5)
        extra_mission = [item for argument in mission_arguments for item in ('-p', argument)]
        mission = start(['/usr/bin/python3', '-m', 'phoenix_tailsitter_control.paper_trajectory_mission', '--ros-args', '-p', 'confirmation:=PHOENIX_LEMNISCATE_MISSION_TEST', '-p', f'trajectory_name:={name}', '-p', 'takeoff_height_m:=10.0', '-p', 'climb_speed_m_s:=0.4', '-p', 'descent_speed_m_s:=0.4', '-p', 'staging_horizontal_speed_m_s:=0.5', '-p', 'takeoff_timeout_s:=100.0', '-p', 'land_timeout_s:=100.0', '-p', 'reference_rate_hz:=20.0', *extra_mission], 'mission.log')
        deadline = time.monotonic() + 300
        last_phase = ''
        while mission.poll() is None and time.monotonic() < deadline:
            time.sleep(2)
            phase_lines = [line for line in (destination / 'mission.log').read_text(errors='replace').splitlines() if 'LEMNISCATE_PHASE' in line or 'LEMNISCATE_ABORT' in line]
            if phase_lines and phase_lines[-1] != last_phase:
                last_phase = phase_lines[-1]
                print(f'[{name}] {last_phase}', flush=True)
        if mission.poll() is None:
            result['status'] = 'mission_timeout'
            raise RuntimeError('Mission exceeded 300 s; ending this isolated simulation')
        result['exit_code'] = mission.returncode
        for line in (destination / 'mission.log').read_text(errors='replace').splitlines():
            if 'LEMNISCATE_MISSION_RESULT ' in line:
                result['mission'] = json.loads(line.split('LEMNISCATE_MISSION_RESULT ', 1)[1])
            elif 'LEMNISCATE_ABORT ' in line:
                result['abort_reason'] = line.split('LEMNISCATE_ABORT ', 1)[1]
        if 'mission' in result:
            result['status'] = ('aborted' if result['mission']['aborted'] else
                                'completed' if result['mission']['passed'] else 'tracking_failed')
        else:
            result['status'] = 'no_mission_result'
    except Exception as error:
        result['error'] = str(error)
        print(f'[{name}] {error}', flush=True)
    finally:
        for process in reversed(processes):
            stop(process)
        for handle in handles:
            handle.close()
    if result.get('mission', {}).get('track_sample_count', 0) > 0:
        with (destination / 'plot.log').open('w') as handle:
            plotted = subprocess.run(['/usr/bin/python3', str(ROOT / 'ros2_ws/analysis/plot_tracking_details.py'), '--bag', str(destination / 'bag'), '--output-dir', str(destination)], cwd=ROOT, env=env, stdout=handle, stderr=subprocess.STDOUT)
        result['plot_exit_code'] = plotted.returncode
        if plotted.returncode == 0:
            result['tracking'] = json.loads((destination / 'summary.json').read_text())
    (destination / 'case_result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(f'[{name}] RESULT {json.dumps({k:v for k,v in result.items() if k not in ("tracking", "mission", "controller_source_sha256")})}', flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--trajectories', nargs='+', choices=NAMES, default=NAMES)
    parser.add_argument('--launch-argument', action='append', default=[],
                        help='Extra ROS launch name:=value; repeat for A/B variants')
    parser.add_argument('--mission-argument', action='append', default=[])
    parser.add_argument('--baseline-controller', action='store_true',
                        help='Keep the previous controller and preparation on every profile')
    parser.add_argument('--controller-source', type=Path,
                        help='Isolated Python/launch package overlay for candidate tests')
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    results = []
    for name in args.trajectories:
        tuned = not args.baseline_controller and name in CONTROLLER_PROFILES and name != 'standard'
        launch_args = controller_arguments(name) if tuned else []
        mission_args = profile_mission_arguments(name) if tuned else []
        results.append(run_case(name, args.output_dir.resolve() / name,
                                launch_args + args.launch_argument,
                                mission_args + args.mission_argument,
                                controller_source=args.controller_source))
        # A suite may be extended by adding new cases after an A/B experiment.
        # Retain every existing completed case instead of replacing its index.
        all_results = [json.loads(path.read_text()) for trajectory in NAMES
                       if (path := args.output_dir / trajectory / 'case_result.json').exists()]
        (args.output_dir / 'suite_results.json').write_text(json.dumps(all_results, indent=2) + '\n')
        time.sleep(3)
    print('Suite finished. All processes started for these tests stopped.', flush=True)


if __name__ == '__main__':
    main()
