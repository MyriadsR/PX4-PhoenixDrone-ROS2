from types import SimpleNamespace

import numpy as np
import pytest

from phoenix_tailsitter_control.alpha_controller_node import AlphaTailsitterController
from phoenix_tailsitter_control.config import PhoenixHoverConfig
from phoenix_tailsitter_control.filters import (
    ButterworthHighPass,
    ButterworthLowPass,
    TimeAwareButterworth,
)


@pytest.mark.parametrize('high_pass', [False, True])
def test_time_aware_filter_matches_fixed_filter_at_nominal_rate(high_pass):
    initial = np.array([0.2, -0.1])
    reference_type = ButterworthHighPass if high_pass else ButterworthLowPass
    reference = reference_type(15.0, 500.0, 2, initial)
    actual = TimeAwareButterworth(15.0, 500.0, 2, initial, high_pass=high_pass)
    for sample in np.random.default_rng(42).normal(size=(1000, 2)):
        np.testing.assert_allclose(
            actual.update(sample, 0.002), reference.update(sample), atol=1e-12)


@pytest.mark.parametrize('high_pass', [False, True])
def test_time_aware_filter_preserves_constant_input_during_jitter(high_pass):
    initial = np.array([0.2, -0.1])
    filt = TimeAwareButterworth(15.0, 500.0, 2, initial, high_pass=high_pass)
    for dt in [0.001, 0.003, 0.01, 0.002] * 100:
        np.testing.assert_allclose(
            filt.update(initial, dt), 0.0 if high_pass else initial, atol=1e-12)


def test_time_aware_filter_tracks_same_sine_under_variable_periods():
    filt = TimeAwareButterworth(15.0, 500.0, 1, [0.0])
    t = 0.0
    errors = []
    w = 2.0 * np.pi * 2.0
    response = 1.0 / (1.0 - (w / filt.frequency)**2
                      + 1j * np.sqrt(2.0) * w / filt.frequency)
    for dt in [0.001, 0.003, 0.0025, 0.0015] * 500:
        t += dt
        actual = filt.update([np.sin(w * t)], dt)[0]
        if t > 1.0:
            expected = abs(response) * np.sin(w * t + np.angle(response))
            errors.append(actual - expected)
    assert np.sqrt(np.mean(np.square(errors))) < 0.002


@pytest.mark.parametrize('dt', [0.0, -1.0, float('nan'), float('inf')])
def test_time_aware_filter_rejects_invalid_dt(dt):
    filt = TimeAwareButterworth(15.0, 500.0, 1, [0.0])
    with pytest.raises(ValueError):
        filt.update([0.0], dt)


def test_control_timing_uses_elapsed_time_and_resets_after_a_gap():
    resets = []
    node = SimpleNamespace(
        cfg=PhoenixHoverConfig(), previous_control_ns=None,
        use_measured_control_dt=True, timing_intervals=[],
        timing_last_publish_ns=0, rates_arrival_ns=0,
        local_position_arrival_ns=0, actuator_feedback_arrival_ns=0,
        timing_debug_publisher=SimpleNamespace(publish=lambda _: None),
        _reset_dynamic_state=lambda: resets.append(True),
    )
    tick = AlphaTailsitterController._update_control_timing
    assert tick(node, 1_000_000_000) == pytest.approx(0.002)
    assert tick(node, 1_003_000_000) == pytest.approx(0.003)
    node.use_measured_control_dt = False
    assert tick(node, 1_006_000_000) == pytest.approx(0.002)
    tick(node, 2_000_000_000)
    assert resets == [True]
