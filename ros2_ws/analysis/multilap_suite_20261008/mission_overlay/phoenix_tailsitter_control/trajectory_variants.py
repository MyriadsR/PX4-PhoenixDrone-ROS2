"""Explicit reference variants; the archived original references remain intact."""
import numpy as np
from .reference_trajectories import DifferentialThrustTurnTrajectory, _quintic_smoothstep

class CenteredDifferentialTurnTrajectory(DifferentialThrustTurnTrajectory):
    """Keep translation and yaw speed; center yaw on the velocity reversal."""
    def sample(self, t):
        position, velocity, acceleration, jerk, _, _ = super().sample(t)
        start = self.entry_duration + .5 * (self.reversal_duration - self.yaw_duration)
        u = float(np.clip((t-start)/self.yaw_duration, 0., 1.))
        progress, derivative, _ = _quintic_smoothstep(u)
        return (position, velocity, acceleration, jerk,
                float(.5*np.pi + np.pi*progress),
                float(np.pi*derivative/self.yaw_duration))
