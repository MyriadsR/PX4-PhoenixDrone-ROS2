#!/usr/bin/env python3
"""Fresh, sequential PX4/Gazebo runs; checkpoint after every mission."""
import hashlib
import json
import os
from pathlib import Path
import sys
import time

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from run_paper_trajectory_suite import run_case
from phoenix_tailsitter_control.maneuver_profiles import controller_arguments, mission_arguments, CONTROLLER_PROFILES

NAMES = ['segments', 'lemniscate', 'knife-edge-transition', 'circle-coordinated',
         'transition-to-forward', 'transition-to-hover', 'differential-turn']
PERIODIC = ['lemniscate', 'knife-edge-transition', 'circle-coordinated']
OVERLAY = HERE / 'mission_overlay'
RESET_ENV = {f'PX4_PARAM_CAL_MAG{sensor}_{axis}OFF':'0'
             for sensor in range(2) for axis in 'XYZ'}


def main():
    # Each rcS imports persisted params before applying PX4_PARAM_* overrides.
    # Reset only calibration offsets; retain the magnetometer check and normal
    # online estimation/autocalibration behavior during each flight.
    os.environ.update(RESET_ENV)
    manifest = {
        'experiment': 'Fresh PX4 SITL + Gazebo, uniform multilap and repetition tests',
        'date': '2026-10-08', 'trajectories': NAMES,
        'excluded': ['circular-knife-edge'], 'closed_laps': 8,
        'fresh_repetitions': 3, 'periodic_trajectories': PERIODIC,
        'nonperiodic_policy': 'Original complete maneuver once per fresh flight; no artificial cyclic join',
        'scoring': 'Formal tracking only; continuous analytic reference for all seven',
        'state_source': 'PX4 estimated state, not Gazebo ground truth',
        'reference_variant': {'differential-turn': 'centered-yaw; retained production opt-in'},
        'controller_profiles': CONTROLLER_PROFILES,
        'production_hash_manifest': str(HERE / 'production_before.json'),
        'test_overlay': str(OVERLAY),
        'per_run_initial_magnetometer_offset_environment': RESET_ENV,
        'preliminary_data': str(HERE/'pilot_shared_calibration'),
        'initialization_policy': 'Same zero magnetic calibration offsets every fresh run; no health check disabled',
        'overlay_sha256': {str(p.relative_to(OVERLAY)): hashlib.sha256(p.read_bytes()).hexdigest()
                           for p in OVERLAY.rglob('*.py')},
    }
    (HERE / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    for repeat in range(1, 4):
        for name in NAMES:
            destination = HERE / f'repeat_{repeat:02d}' / name
            if (destination / 'case_result.json').exists():
                print(f'Checkpoint retained: repeat {repeat}, {name}', flush=True)
                continue
            tuned = name in CONTROLLER_PROFILES
            print(f'REPEAT {repeat}/3 CASE {name}', flush=True)
            result = run_case(name, destination,
                     controller_arguments(name) if tuned else [],
                     (mission_arguments(name) if tuned else []) + ['test_closed_laps:=8'],
                     controller_source=OVERLAY)
            result['initial_magnetometer_offset_environment'] = RESET_ENV
            (destination/'case_result.json').write_text(json.dumps(result,indent=2)+'\n')
            results = [json.loads(p.read_text()) for p in sorted(HERE.glob('repeat_*/*/case_result.json'))]
            (HERE / 'suite_results.json').write_text(json.dumps(results, indent=2) + '\n')
            time.sleep(3)
    print('ALL 21 FRESH MISSIONS FINISHED; OWNED SIMULATIONS STOPPED', flush=True)


if __name__ == '__main__':
    main()
