#!/usr/bin/env python3
"""Read-only comparison of the original numerical model and recorded SITL.

Experiments wrap controller methods in this process; neither project source nor
the numerical project's existing logs are changed. Actuator limits/dynamics and
initialization are retained in every run. No live simulator is started.
"""
import argparse
import contextlib
import hashlib
import json
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace

os.environ.setdefault("MPLBACKEND", "Agg")
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
ORIGINAL = Path("/home/zr/Tailsitter-control")
PROFILES = ("knife-edge-transition", "circular-knife-edge", "differential-turn")
ALPHA_LIMIT = np.array([20.0, 15.0, 24.0])
MOMENT_LIMIT = np.array([0.25, 0.08, 0.35])


def run_experiment(sim, profile, mode, out, sitl_duration):
    folder = out / mode / profile
    folder.mkdir(parents=True, exist_ok=True)
    pd_original = sim.PDAttitudeController.compute_angular_acceleration_command
    indi_original = sim.INDIAngularController.compute_moment_command
    allocator_original = sim.ControlAllocator.allocate_controls
    position_original = sim.PositionController.compute_acceleration_command
    linear_original = sim.INDILinearController.compute_force_command
    records = {name: [] for name in ("alpha_raw", "moment_raw", "moment_applied",
                                     "motor_command", "flap_command")}
    extra_filter = None
    latest_reference = None
    latest_position = None
    with_alpha_limit = mode in ("alpha_limits", "both_limits", "both_limits_force_floor")
    with_moment_limit = mode in ("moment_limits", "both_limits", "both_limits_force_floor")
    with_force_floor = mode in ("force_floor", "both_limits_force_floor")
    sys.path.insert(0, str(ROOT / "ros2_ws/src/phoenix_tailsitter_control"))
    from phoenix_tailsitter_control.position_control import apply_takeoff_force_floor
    from phoenix_tailsitter_control.config import PhoenixHoverConfig
    floor_config = PhoenixHoverConfig()

    def pd(self, *args, **kwargs):
        value = pd_original(self, *args, **kwargs)
        records["alpha_raw"].append(value.copy())
        if with_alpha_limit:
            return np.clip(value, -ALPHA_LIMIT, ALPHA_LIMIT)
        return value

    def indi(self, alpha, alpha_measured, model_moment):
        nonlocal extra_filter
        if mode == "extra_moment_lpf":
            if extra_filter is None:
                extra_filter = sim.SecondOrderButterworthFilter(
                    15.0, 500.0, channels=3, filter_type="lowpass",
                    initial_value=model_moment)
            model_moment = extra_filter.update(model_moment)
        value = indi_original(self, alpha, alpha_measured, model_moment)
        records["moment_raw"].append(value.copy())
        if with_moment_limit:
            value = np.clip(value, -MOMENT_LIMIT, MOMENT_LIMIT)
        records["moment_applied"].append(value.copy())
        return value

    def allocate(self, *args, **kwargs):
        motor, flap = allocator_original(self, *args, **kwargs)
        records["motor_command"].append(motor.copy())
        records["flap_command"].append(flap.copy())
        return motor, flap

    def position(self, xref, vref, aref, pos, *args, **kwargs):
        nonlocal latest_reference, latest_position
        latest_reference = SimpleNamespace(position=xref, velocity=vref, acceleration=aref,
                                           position_mask=np.ones(3, dtype=bool),
                                           velocity_mask=np.ones(3, dtype=bool))
        latest_position = pos
        return position_original(self, xref, vref, aref, pos, *args, **kwargs)

    def linear(self, ac, measured_acceleration, force_alpha_lpf, rotation, r_alpha_to_body):
        value = linear_original(self, ac, measured_acceleration, force_alpha_lpf, rotation, r_alpha_to_body)
        if with_force_floor:
            value = apply_takeoff_force_floor(value, .7 * (ac - np.array([0., 0., 9.81])),
                                             rotation @ r_alpha_to_body @ force_alpha_lpf,
                                             latest_position, latest_reference, floor_config)
        return value

    sim.PDAttitudeController.compute_angular_acceleration_command = pd
    sim.INDIAngularController.compute_moment_command = indi
    sim.ControlAllocator.allocate_controls = allocate
    sim.PositionController.compute_acceleration_command = position
    sim.INDILinearController.compute_force_command = linear
    started = time.monotonic()
    try:
        with (folder / "simulation.log").open("w") as log, contextlib.redirect_stdout(log):
            result = sim.main(profile, log_path=str(folder / "flight_log.npz"), show_plot=False)
    finally:
        sim.PDAttitudeController.compute_angular_acceleration_command = pd_original
        sim.INDIAngularController.compute_moment_command = indi_original
        sim.ControlAllocator.allocate_controls = allocator_original
        sim.PositionController.compute_acceleration_command = position_original
        sim.INDILinearController.compute_force_command = linear_original
    records = {name: np.array(value) for name, value in records.items()}
    np.savez_compressed(folder / "commands.npz", **records)
    with np.load(folder / "flight_log.npz") as data:
        err = np.linalg.norm(data["position"] - data["position_ref"], axis=1)
        common = data["time"] <= sitl_duration
        result.update({
            "common_window_end_s": sitl_duration,
            "common_window_position_rmse_m": float(np.sqrt(np.mean(err[common] ** 2))),
            "first_position_error_above_5m_s": float(data["time"][err > 5][0]) if np.any(err > 5) else None,
            "first_position_error_above_10m_s": float(data["time"][err > 10][0]) if np.any(err > 10) else None,
            "alpha_raw_max_abs_rad_s2": np.max(np.abs(records["alpha_raw"]), axis=0).tolist(),
            "alpha_actual_max_abs_rad_s2": np.max(np.abs(data["angular_acceleration"]), axis=0).tolist(),
            "alpha_raw_above_sitl_limit_fraction": np.mean(np.abs(records["alpha_raw"]) > ALPHA_LIMIT, axis=0).tolist(),
            "moment_raw_max_abs_nm": np.max(np.abs(records["moment_raw"]), axis=0).tolist(),
            "moment_raw_above_sitl_limit_fraction": np.mean(np.abs(records["moment_raw"]) > MOMENT_LIMIT, axis=0).tolist(),
            "motor_command_at_bound_fraction": np.mean((records["motor_command"] <= .001) | (records["motor_command"] >= 2499.999), axis=0).tolist(),
            "flap_command_at_bound_fraction": np.mean(np.abs(records["flap_command"]) >= .999, axis=0).tolist(),
            "initial_speed_m_s": float(np.linalg.norm(data["velocity"][0])),
            "initial_body_rate_rad_s": data["angular_velocity"][0].tolist(),
            "initial_flap_deg": data["flap_deg"][0].tolist(),
            "wall_time_s": time.monotonic() - started,
        })
    (folder / "metrics.json").write_text(json.dumps(result, indent=2) + "\n")
    print(f"{profile:24} {mode:18} full RMS={result['rms_position_error']:.3f} m; "
          f"common RMS={result['common_window_position_rmse_m']:.3f} m; "
          f"elapsed={result['wall_time_s']:.1f}s", flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=ROOT / "ros2_ws/analysis/numerical_sitl_comparison_20261004")
    modes = ["baseline", "alpha_limits", "moment_limits", "both_limits", "extra_moment_lpf",
             "force_floor", "both_limits_force_floor"]
    parser.add_argument("--modes", nargs="+", choices=modes, default=modes)
    parser.add_argument("--profiles", nargs="+", default=list(PROFILES))
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    snapshot_files = ["main_sim.py", "config.py", "controller/att_ctrl.py",
                      "controller/pos_ctrl.py", "controller/actuator_ctrl.py",
                      "environment/aerodynamics.py", "environment/dynamics.py",
                      "trajectory/test.py", "utils/filters.py"]
    manifest = {name: hashlib.sha256((ORIGINAL / name).read_bytes()).hexdigest()
                for name in snapshot_files}
    (args.out / "source_manifest.json").write_text(json.dumps({
        "original_project": str(ORIGINAL), "sha256": manifest,
        "alpha_limit_rad_s2": ALPHA_LIMIT.tolist(), "moment_limit_nm": MOMENT_LIMIT.tolist(),
        "note": "Offline wrappers only; original sources/logs and production controller unchanged."}, indent=2) + "\n")
    sys.path.insert(0, str(ORIGINAL))
    import main_sim as sim
    metrics_path = args.out / "metrics.json"
    metrics = json.loads(metrics_path.read_text()) if metrics_path.exists() else {}
    for mode in args.modes:
        metrics.setdefault(mode, {})
        for profile in args.profiles:
            latest = json.loads((ROOT / "ros2_ws/analysis/tracking_improvement_20261004/rate_transport" / profile / "summary.json").read_text())
            metrics[mode][profile] = run_experiment(sim, profile, mode, args.out,
                                                   latest["tracking_duration_s"])
            metrics_path.write_text(json.dumps(metrics, indent=2) + "\n")

if __name__ == "__main__":
    main()
