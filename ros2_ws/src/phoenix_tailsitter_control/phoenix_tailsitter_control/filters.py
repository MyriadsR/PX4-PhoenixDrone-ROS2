"""Dependency-free second-order Butterworth filters for the INDI loop."""

import math

import numpy as np


class TimeAwareButterworth:
    """Two-pole Butterworth with physical states and a variable sample period.

    Trapezoidal integration equals the fixed-rate bilinear filter at its
    nominal period. Physical states remain valid when dt changes.
    """

    def __init__(self, cutoff_hz, sample_rate_hz, channels, initial_value,
                 *, high_pass=False):
        if (not math.isfinite(cutoff_hz) or not math.isfinite(sample_rate_hz)
                or not 0.0 < cutoff_hz < 0.5 * sample_rate_hz):
            raise ValueError('cutoff must be finite, positive and below Nyquist')
        self.channels = int(channels)
        if self.channels <= 0 or self.channels != channels:
            raise ValueError('channels must be a positive integer')
        self.nominal_dt = 1.0 / sample_rate_hz
        self.frequency = 2.0 * sample_rate_hz * math.tan(
            math.pi * cutoff_hz / sample_rate_hz)
        self.high_pass = bool(high_pass)
        self.position = self._sample(initial_value).copy()
        self.velocity = np.zeros(self.channels)
        self.previous_input = self.position.copy()

    def _sample(self, value):
        sample = np.asarray(value, dtype=float)
        if sample.shape != (self.channels,) or not np.all(np.isfinite(sample)):
            raise ValueError(f'filter sample must be {self.channels} finite values')
        return sample

    def update(self, value, dt=None):
        sample = self._sample(value)
        dt = self.nominal_dt if dt is None else float(dt)
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError('filter dt must be finite and positive')
        h = 0.5 * self.frequency * dt
        damping = math.sqrt(2.0)
        rhs_position = self.position + h * self.velocity
        rhs_velocity = (
            -h * self.position + (1.0 - damping * h) * self.velocity
            + h * (self.previous_input + sample))
        self.velocity = (rhs_velocity - h * rhs_position) / (
            1.0 + damping * h + h * h)
        self.position = rhs_position + h * self.velocity
        self.previous_input = sample.copy()
        if self.high_pass:
            return sample - self.position - damping * self.velocity
        return self.position.copy()


class ButterworthLowPass:
    """Fixed-rate, two-pole low-pass using a transposed direct-form II biquad."""

    def __init__(self, cutoff_hz, sample_rate_hz, channels, initial_value):
        if cutoff_hz <= 0.0 or sample_rate_hz <= 0.0:
            raise ValueError('filter frequencies must be positive')
        if cutoff_hz >= 0.5 * sample_rate_hz:
            raise ValueError('cutoff must be below Nyquist')
        self.channels = int(channels)
        if self.channels <= 0 or self.channels != channels:
            raise ValueError('channels must be a positive integer')

        # Bilinear-transform coefficients for a second-order Butterworth LPF.
        k = math.tan(math.pi * cutoff_hz / sample_rate_hz)
        norm = 1.0 / (1.0 + math.sqrt(2.0) * k + k * k)
        self.b0 = k * k * norm
        self.b1 = 2.0 * self.b0
        self.b2 = self.b0
        self.a1 = 2.0 * (k * k - 1.0) * norm
        self.a2 = (1.0 - math.sqrt(2.0) * k + k * k) * norm

        initial = self._sample(initial_value)
        # Steady-state initial conditions give y[0] == initial without a ramp.
        self.state_1 = initial * (1.0 - self.b0)
        self.state_2 = initial * (self.b2 - self.a2)

    def _sample(self, value):
        sample = np.asarray(value, dtype=float)
        if sample.shape != (self.channels,) or not np.all(np.isfinite(sample)):
            raise ValueError(f'filter sample must be {self.channels} finite values')
        return sample

    def update(self, value):
        sample = self._sample(value)
        output = self.b0 * sample + self.state_1
        state_1 = self.b1 * sample - self.a1 * output + self.state_2
        self.state_2 = self.b2 * sample - self.a2 * output
        self.state_1 = state_1
        return output


class ButterworthHighPass:
    """Fixed-rate two-pole Butterworth high-pass with zero steady output."""

    def __init__(self, cutoff_hz, sample_rate_hz, channels, initial_value):
        if cutoff_hz <= 0.0 or sample_rate_hz <= 0.0:
            raise ValueError('filter frequencies must be positive')
        if cutoff_hz >= 0.5 * sample_rate_hz:
            raise ValueError('cutoff must be below Nyquist')
        self.channels = int(channels)
        if self.channels <= 0 or self.channels != channels:
            raise ValueError('channels must be a positive integer')
        k = math.tan(math.pi * cutoff_hz / sample_rate_hz)
        norm = 1.0 / (1.0 + math.sqrt(2.0) * k + k * k)
        self.b0 = norm
        self.b1 = -2.0 * norm
        self.b2 = norm
        self.a1 = 2.0 * (k * k - 1.0) * norm
        self.a2 = (1.0 - math.sqrt(2.0) * k + k * k) * norm
        initial = self._sample(initial_value)
        # For a constant input x, y=0 requires these transposed DF-II states.
        self.state_1 = -self.b0 * initial
        self.state_2 = self.b2 * initial

    def _sample(self, value):
        sample = np.asarray(value, dtype=float)
        if sample.shape != (self.channels,) or not np.all(np.isfinite(sample)):
            raise ValueError(f'filter sample must be {self.channels} finite values')
        return sample

    def update(self, value):
        sample = self._sample(value)
        output = self.b0 * sample + self.state_1
        state_1 = self.b1 * sample - self.a1 * output + self.state_2
        self.state_2 = self.b2 * sample - self.a2 * output
        self.state_1 = state_1
        return output
