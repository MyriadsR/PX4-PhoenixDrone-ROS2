"""Reference timing/mask contracts and stationary-versus-moving gain behavior."""
from types import SimpleNamespace
import math

import numpy as np
import pytest

from phoenix_tailsitter_control.config import PhoenixHoverConfig
from phoenix_tailsitter_control.attitude_control import AttitudeINDIController, transport_reference_rates
from phoenix_tailsitter_control.math_utils import quaternion_to_matrix
from phoenix_tailsitter_control.filters import TimeAwareButterworth
from phoenix_tailsitter_control.paper_trajectory_mission import PaperTrajectoryMission
from phoenix_tailsitter_control.reference_trajectories import CircularTrajectory
from phoenix_tailsitter_control.position_control import (
    LinearAccelerationINDIController, PositionController, advance_trajectory_reference, maneuver_gain_target,
    resolve_trajectory_reference, trajectory_reference_age, update_gain_blend,
)


def reference(p=(0., 0., 0.), v=(0., 0., 0.), a=(0., 0., 0.),
              j=(0., 0., 0.), yaw=0., rate=0.):
    msg = SimpleNamespace(position=p, velocity=v, acceleration=a,
                          jerk=j, yaw=yaw, yawspeed=rate)
    return resolve_trajectory_reference(msg, np.array([1., 2., 3.]),
                                        np.array([.1, .2, .3]), .7)


def test_prediction_tracks_constant_jerk_motion_between_messages():
    initial = reference(p=[1., 2., 3.], v=[4., 5., 6.], a=[7., 8., 9.],
                        j=[10., 11., 12.], yaw=3.13, rate=.8)
    dt = .047
    predicted = advance_trajectory_reference(initial, dt)
    np.testing.assert_allclose(predicted.position,
        initial.position + dt * initial.velocity + dt**2 / 2 * initial.acceleration
        + dt**3 / 6 * initial.jerk)
    np.testing.assert_allclose(predicted.velocity,
        initial.velocity + dt * initial.acceleration + dt**2 / 2 * initial.jerk)
    np.testing.assert_allclose(predicted.acceleration, initial.acceleration + dt * initial.jerk)
    assert predicted.yaw == pytest.approx(3.13 + .8 * dt)
    # The input remains a sample at the publisher time, not cumulatively aged.
    np.testing.assert_allclose(initial.position, [1, 2, 3])


def test_prediction_removes_constant_speed_circle_hold_error():
    curve = CircularTrajectory()
    t, age = 1.2, .048
    p, v, a, j, yaw, rate = curve.sample(t)
    predicted = advance_trajectory_reference(reference(p, v, a, j, yaw, rate), age)
    exact = curve.sample(t + age)
    assert np.linalg.norm(predicted.position - exact[0]) < 3e-5
    assert np.linalg.norm(predicted.velocity - exact[1]) < .002
    assert abs(math.remainder(predicted.yaw - exact[4], 2 * math.pi)) < 1e-12
    assert np.linalg.norm(p - exact[0]) > .38


def test_prediction_preserves_uncontrolled_axes_and_position_only_holds():
    initial = reference(p=[1., math.nan, -2.], v=[math.nan]*3,
                        a=[math.nan, .5, math.nan], j=[1., 2., 3.])
    predicted = advance_trajectory_reference(initial, .05)
    np.testing.assert_array_equal(predicted.position_mask, [True, False, True])
    np.testing.assert_array_equal(predicted.velocity_mask, [True, False, True])
    np.testing.assert_allclose(predicted.position, initial.position)
    np.testing.assert_allclose(predicted.velocity, initial.velocity)
    np.testing.assert_allclose(predicted.acceleration, [0., .6, 0.])


def test_prediction_does_not_repeatedly_advance_unspecified_yaw_fallback():
    initial = reference(yaw=math.nan, rate=.8)
    predicted = advance_trajectory_reference(initial, .05, advance_yaw=False)
    assert predicted.yaw == .7


def test_reference_age_uses_sender_time_includes_transport_and_bounds_horizon():
    now = 1_700_000_000_000_000_000
    stamp = (now - 45_000_000) // 1000
    arrival = now - 40_000_000
    age, common_clock = trajectory_reference_age(stamp, now, arrival)
    assert common_clock and age == pytest.approx(.045)
    assert trajectory_reference_age(stamp, now + 200_000_000, arrival)[0] == .1
    assert trajectory_reference_age(now // 1000 + 1000, now, arrival)[0] == 0.


@pytest.mark.parametrize('stamp', [0, 123456789])
def test_reference_age_falls_back_for_missing_or_boot_relative_stamp(stamp):
    now = 1_700_000_000_000_000_000
    age, common_clock = trajectory_reference_age(stamp, now, now - 25_000_000)
    assert not common_clock and age == pytest.approx(.025)


def test_stationary_finite_acceleration_uses_hover_gains_even_at_large_error():
    cfg = PhoenixHoverConfig()
    hold = reference(p=[3., 0., -10.])
    assert maneuver_gain_target(hold, cfg) == 0.
    controller = PositionController(cfg)
    command = controller.acceleration_command(hold, np.zeros(3), np.ones(3),
                                              np.zeros(3), np.eye(3), tracking_blend=0.)
    assert np.linalg.norm(command[:2]) == pytest.approx(cfg.horizontal_acceleration_limit)
    assert abs(command[2]) <= cfg.vertical_acceleration_limit


@pytest.mark.parametrize('kwargs', [dict(v=[1., 0., 0.]), dict(a=[0., 1., 0.]), dict(rate=1.)])
def test_moving_explicit_reference_activates_maneuver_gains(kwargs):
    assert maneuver_gain_target(reference(**kwargs), PhoenixHoverConfig()) == 1.


def test_nan_acceleration_staging_retains_low_speed_gains():
    assert maneuver_gain_target(reference(v=[.4, 0., 0.], a=[math.nan]*3), PhoenixHoverConfig()) == 0.


def test_uncontrolled_measured_velocity_does_not_activate_maneuver_schedule():
    command = reference(p=[math.nan]*3, v=[math.nan]*3, a=[0., 0., 0.])
    assert np.linalg.norm(command.velocity) > 0.
    assert maneuver_gain_target(command, PhoenixHoverConfig()) == 0.


def test_gain_blend_has_bounded_slew_and_reaches_hover_without_chatter():
    blend = 1.
    for _ in range(500):
        old = blend
        blend = update_gain_blend(blend, 0., .002, 1.)
        assert 0 <= old - blend <= .002 + 1e-12
    assert blend == pytest.approx(0., abs=1e-12)
    assert update_gain_blend(.4, .4005, .002, 1.) == .4005


def test_trajectory_hover_brakes_without_changing_staging_or_moving_endpoint():
    cfg = PhoenixHoverConfig()
    controller = PositionController(cfg)
    hold = reference(p=[3., 0., 0.])
    command = controller.acceleration_command(hold, np.zeros(3), np.zeros(3),
        np.zeros(3), np.eye(3), tracking_blend=0., trajectory_mode=True)
    assert np.linalg.norm(command[:2]) == pytest.approx(cfg.trajectory_hover_horizontal_acceleration_limit)
    attitude = AttitudeINDIController(cfg)
    q, target = np.array([1., 0., 0., 0.]), np.array([math.cos(.05), 0., math.sin(.05), 0.])
    staging = attitude.angular_acceleration_command(q, target, np.zeros(3))
    hover = attitude.angular_acceleration_command(q, target, np.zeros(3), trajectory_mode=True)
    assert hover[1] > staging[1]
    np.testing.assert_allclose(
        attitude.angular_acceleration_command(q, target, np.zeros(3), tracking_blend=1., trajectory_mode=True),
        attitude.angular_acceleration_command(q, target, np.zeros(3), tracking_blend=1.))


def test_ground_start_profile_begins_its_own_takeoff_on_arm(monkeypatch):
    from phoenix_tailsitter_control.lemniscate_mission_test import LemniscateMissionTest
    phases = []
    monkeypatch.setattr(LemniscateMissionTest, '_set_phase', lambda self, name, now: phases.append(name))
    node = object.__new__(PaperTrajectoryMission)
    node.trajectory = SimpleNamespace(ground_start=True)
    node.track_started_ns = None
    node.origin = np.array([1., 2., 3.])
    node.takeoff_origin_ned = None
    PaperTrajectoryMission._set_phase(node, 'takeoff', 123)
    assert node.track_started_ns == 123
    assert phases == ['track']
    node.origin[:2] = 30.
    np.testing.assert_allclose(node.takeoff_origin_ned, [1., 2., 3.])


def test_reference_rate_transport_preserves_world_motion_when_body_is_tilted():
    q_current = np.array([math.cos(.4), 0., math.sin(.4), 0.])
    q_reference = np.array([math.cos(.2), math.sin(.2), 0., 0.])
    world_rate = np.array([0., 0., 2.3])
    nominal_body_rate = quaternion_to_matrix(q_reference).T @ world_rate
    transported = transport_reference_rates(q_current, q_reference, nominal_body_rate)
    np.testing.assert_allclose(quaternion_to_matrix(q_current) @ transported, world_rate, atol=1e-12)
    assert np.linalg.norm(transported - nominal_body_rate) > 1.
    np.testing.assert_allclose(transport_reference_rates(q_reference, q_reference, nominal_body_rate), nominal_body_rate, atol=1e-12)


def test_world_force_and_acceleration_filters_cancel_rotating_thrust_in_indi():
    cfg = PhoenixHoverConfig()
    gravity = np.array([0., 0., cfg.gravity])
    initial_force = -cfg.mass * gravity
    force_filter = TimeAwareButterworth(15., 500., 3, initial_force)
    acceleration_filter = TimeAwareButterworth(15., 500., 3, np.zeros(3))
    transient_filter = TimeAwareButterworth(15., 500., 3, np.zeros(3))
    indi = LinearAccelerationINDIController(cfg)
    time, legacy_phase_errors = 0., []
    command = np.array([.2, -.1, 0.])
    for dt in [.002, .0035, .0015] * 300:
        time += dt
        angle = .4 * math.sin(2 * math.pi * .8 * time)
        raw_force = cfg.hover_total_thrust * np.array([math.sin(angle), 0., -math.cos(angle)])
        measured_acceleration = raw_force / cfg.mass + gravity
        total_lpf = force_filter.update(raw_force, dt)
        a_lpf = acceleration_filter.update(measured_acceleration, dt)
        flap_lpf = transient_filter.update([.1 * math.sin(5 * time), 0., 0.], dt)
        # Steady/transient decomposition must preserve the total filtered force.
        result = indi.force_command(command, a_lpf - flap_lpf / cfg.mass,
                                    total_lpf - flap_lpf)
        np.testing.assert_allclose(result, cfg.mass * (command - gravity), atol=1e-11)
        legacy_phase_errors.append(np.linalg.norm(raw_force - total_lpf))
    assert max(legacy_phase_errors) > .08


@pytest.mark.parametrize('dt', [-.1, math.nan, math.inf])
def test_prediction_rejects_invalid_time(dt):
    with pytest.raises(ValueError):
        advance_trajectory_reference(reference(), dt)
