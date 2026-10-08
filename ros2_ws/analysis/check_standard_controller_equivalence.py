#!/usr/bin/env python3
"""Compare untuned controller outputs with the user's saved starting version."""
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
from phoenix_tailsitter_control.attitude_control import AttitudeINDIController
from phoenix_tailsitter_control.config import PhoenixHoverConfig
from phoenix_tailsitter_control.position_control import apply_takeoff_force_floor, PositionController
from phoenix_tailsitter_control.paper_trajectory_mission import PaperTrajectoryAdapter, NAMES

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'ros2_ws/analysis/maneuver_improvement_20261004'
BACKUP = OUT / 'baseline_source/phoenix_tailsitter_control'


def original(name):
    spec = importlib.util.spec_from_file_location('phoenix_tailsitter_control.baseline_' + name,
                                                  BACKUP / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def main():
    cfg = PhoenixHoverConfig()
    old_attitude = original('attitude_control').AttitudeINDIController(cfg)
    new_attitude = AttitudeINDIController(cfg)
    old_floor = original('position_control').apply_takeoff_force_floor
    old_position = original('position_control').PositionController(cfg)
    new_position = PositionController(cfg)
    old_adapter = original('paper_trajectory_mission').PaperTrajectoryAdapter
    rng = np.random.default_rng(20261004)
    counts = {'angular_commands': 0, 'moment_commands': 0, 'takeoff_force_commands': 0,
              'position_commands': 0, 'standard_reference_samples': 0, 'standard_entry_samples': 0}
    for _ in range(500):
        q, target = rng.normal(size=(2, 4))
        q /= np.linalg.norm(q)
        target /= np.linalg.norm(target)
        rates, feedforward = rng.normal(size=(2, 3)) * 20
        kwargs = dict(tracking_blend=float(rng.random()), trajectory_mode=bool(rng.integers(2)))
        np.testing.assert_array_equal(old_attitude.angular_acceleration_command(q, target, rates, feedforward, **kwargs),
                                      new_attitude.angular_acceleration_command(q, target, rates, feedforward, **kwargs))
        counts['angular_commands'] += 1
        inputs = rng.normal(size=(3, 3)) * 50
        np.testing.assert_array_equal(old_attitude.moment_command(*inputs), new_attitude.moment_command(*inputs))
        counts['moment_commands'] += 1
        reference = SimpleNamespace(position=rng.normal(size=3) * 5,
                                    velocity=rng.normal(size=3), acceleration=rng.normal(size=3),
                                    position_mask=np.ones(3, dtype=bool), velocity_mask=np.ones(3, dtype=bool))
        force, nominal, estimated, position = rng.normal(size=(4, 3)) * 10
        np.testing.assert_array_equal(old_floor(force, nominal, estimated, position, reference, cfg),
                                      apply_takeoff_force_floor(force, nominal, estimated, position, reference, cfg))
        counts['takeoff_force_commands'] += 1
        reference.acceleration_mask = np.ones(3, dtype=bool)
        pos, vel, accel = rng.normal(size=(3, 3)) * 10
        from phoenix_tailsitter_control.math_utils import quaternion_to_matrix
        rotation = quaternion_to_matrix(q)
        np.testing.assert_array_equal(
            old_position.acceleration_command(reference, pos, vel, accel, rotation, **kwargs),
            new_position.acceleration_command(reference, pos, vel, accel, rotation, **kwargs))
        counts['position_commands'] += 1
    for name in NAMES:
        old, new = old_adapter(name), PaperTrajectoryAdapter(name)
        for t in np.linspace(0, old.total_duration, 101):
            a, b = old.sample(t), new.sample(t)
            for field in ('position', 'velocity', 'acceleration', 'jerk', 'yaw', 'yawspeed'):
                np.testing.assert_array_equal(getattr(a, field), getattr(b, field))
            counts['standard_reference_samples'] += 1
        for t in np.linspace(0, 4, 41):
            a, b = old.entry_sample(t, 4), new.entry_sample(t, 4)
            for field in ('position', 'velocity', 'acceleration', 'jerk', 'yaw', 'yawspeed'):
                np.testing.assert_array_equal(getattr(a, field), getattr(b, field))
            counts['standard_entry_samples'] += 1
    result = {'passed': True, 'equality': 'exact NumPy element equality',
              'baseline': str(BACKUP), 'cases': counts,
              'scope': 'Default, untuned functions and reference adapters; runtime ROS/Gazebo timing is checked separately.'}
    (OUT / 'standard_equivalence.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
