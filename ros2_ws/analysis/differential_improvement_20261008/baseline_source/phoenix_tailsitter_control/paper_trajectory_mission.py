"""SITL missions for the eight unmodified Tailsitter-control reference profiles.

reference_trajectories.py is a byte-for-byte snapshot of
/home/zr/Tailsitter-control/trajectory/test.py, commit 4cc11be,
SHA256 b2a2b64d5316a579ef5ae263c5b8d75af7568359f4c7a554b0cec97202831786.
Only constant world-position translation is applied to the tracking profiles.
Time-scaled preparation/exit joins are outside the scored tracking interval.
"""
import math
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from .lemniscate_mission_test import LemniscateMissionTest, LemniscateSample
from .reference_trajectories import (
    TrajectoryGenerator, LemniscateTrajectory, KnifeEdgeTransitionTrajectory,
    CircularTrajectory, CircularTransitionTrajectory, DifferentialThrustTurnTrajectory,
)

NAMES = ('segments', 'lemniscate', 'knife-edge-transition', 'circle-coordinated',
         'circular-knife-edge', 'transition-to-forward', 'transition-to-hover',
         'differential-turn')


def make_reference(name):
    if name == 'segments':
        return TrajectoryGenerator()
    if name == 'lemniscate':
        return LemniscateTrajectory()
    if name == 'knife-edge-transition':
        return KnifeEdgeTransitionTrajectory()
    if name in ('circle-coordinated', 'circular-knife-edge'):
        return CircularTrajectory(mode='coordinated' if name == 'circle-coordinated' else 'knife_edge')
    if name in ('transition-to-forward', 'transition-to-hover'):
        return CircularTransitionTrajectory(transition='to_forward' if name == 'transition-to-forward' else 'to_hover')
    if name == 'differential-turn':
        return DifferentialThrustTurnTrajectory()
    raise ValueError(f'Unknown reference trajectory: {name}')


class PaperTrajectoryAdapter:
    track_phase_offset_s = 0.0

    def __init__(self, name, entry_duration=4.0, *, knife_entry_straight=False):
        self.name = name
        self.knife_entry_straight = bool(knife_entry_straight)
        self.reference = make_reference(name)
        self.ground_start = self.reference.initial_state_mode == 'ground'
        self.total_duration = self.reference.total_duration
        self.laps = getattr(self.reference, 'laps', 1)
        self.lap_time = getattr(self.reference, 'lap_time', self.total_duration)
        self.speed = getattr(self.reference, 'speed', getattr(self.reference, 'target_speed', 3.125))
        self.scale = getattr(self.reference, 'scale', getattr(self.reference, 'radius', 1.0))
        self.takeoff_height = 10.0
        self.entry_duration_s = entry_duration
        initial = self.reference.sample(0.0)
        entry_initial = self._extended_sample(-0.5 * entry_duration)
        # Accelerate along the reference geometry into its exact original t=0.
        # Spatial translation preserves the complete scored profile.
        self.offset = np.zeros(3)
        if not self.ground_start:
            self.offset[:2] = -entry_initial[0][:2]
        self.staging_position = np.array([0.0, 0.0, 0.0 if self.ground_start else -10.0])
        self.initial_yaw = float(entry_initial[4])

    def sample(self, t):
        p, v, a, j, yaw, yawspeed = self.reference.sample(t)
        return LemniscateSample(p + self.offset, v, a, j, yaw, yawspeed)

    def staging_sample(self):
        return LemniscateSample(self.staging_position.copy(), np.zeros(3), np.zeros(3), np.zeros(3), self.initial_yaw, 0.0)

    def _extended_sample(self, t):
        if t >= 0:
            return self.reference.sample(t)
        if (self.name in ('lemniscate', 'circle-coordinated', 'circular-knife-edge')
                or (self.name == 'knife-edge-transition' and not self.knife_entry_straight)):
            periods = int(math.ceil(-t / self.lap_time))
            p, v, a, j, yaw, rate = self.reference.sample(t + periods * self.lap_time)
            if self.name == 'knife-edge-transition':
                yaw -= periods * math.pi
            return p, v, a, j, yaw, rate
        if self.name == 'transition-to-hover':
            # Before the original slowdown, extend its initial constant-speed
            # circle. The original scored slowdown remains unchanged.
            circle = CircularTrajectory(speed=self.speed, radius=self.reference.radius)
            return circle.sample(t % circle.lap_time)
        p, v, a, j, yaw, rate = self.reference.sample(0)
        return p + t * v, v, a, j, yaw, rate

    def entry_sample(self, t, duration):
        if duration <= 0:
            raise ValueError('entry duration must be positive')
        u = float(np.clip(t / duration, 0, 1))
        integral = 2.5 * u**4 - 3 * u**5 + u**6
        rate = 10 * u**3 - 15 * u**4 + 6 * u**5
        acceleration = (30 * u**2 - 60 * u**3 + 30 * u**4) / duration
        jerk = (60 * u - 180 * u**2 + 120 * u**3) / duration**2
        p, v, a, j, yaw, yawspeed = self._extended_sample(duration * (integral - 0.5))
        if self.ground_start:
            p = np.zeros(3)
        return LemniscateSample(p + self.offset, v * rate,
            a * rate**2 + v * acceleration,
            j * rate**3 + 3 * a * rate * acceleration + v * jerk,
            yaw, yawspeed * rate)

    def exit_sample(self, start_time, t, duration):
        # Decelerate along the same reference geometry/yaw law. A straight
        # polynomial join can introduce a large attitude change at the end
        # of a high-curvature trajectory even when its derivatives match.
        if duration <= 0:
            raise ValueError('exit duration must be positive')
        u = float(np.clip(t / duration, 0, 1))
        integral = 2.5 * u**4 - 3 * u**5 + u**6
        rate = 1 - (10 * u**3 - 15 * u**4 + 6 * u**5)
        acceleration = -(30 * u**2 - 60 * u**3 + 30 * u**4) / duration
        jerk = -(60 * u - 180 * u**2 + 120 * u**3) / duration**2
        sample = self.sample(start_time + duration * (u - integral))
        return LemniscateSample(
            sample.position,
            sample.velocity * rate,
            sample.acceleration * rate**2 + sample.velocity * acceleration,
            sample.jerk * rate**3 + 3 * sample.acceleration * rate * acceleration + sample.velocity * jerk,
            sample.yaw, sample.yawspeed * rate)

    def horizontal_radius_bound(self):
        samples = [self.sample(t).position for t in np.linspace(0, self.total_duration, 1501)]
        if not self.ground_start:
            samples.extend(self.entry_sample(t, self.entry_duration_s).position
                           for t in np.linspace(0, self.entry_duration_s, 101))
        return max(float(np.linalg.norm(p[:2])) for p in samples)


class PaperTrajectoryMission(LemniscateMissionTest):
    def _maximum_reference_speed(self):
        return 8.1

    def _make_trajectory(self):
        self.declare_parameter('trajectory_name', 'segments')
        self.declare_parameter('knife_entry_straight', False)
        self.trajectory_name = str(self.get_parameter('trajectory_name').value)
        trajectory = PaperTrajectoryAdapter(
            self.trajectory_name, self.entry_duration_s,
            knife_entry_straight=bool(self.get_parameter('knife_entry_straight').value))
        self.speed = trajectory.speed
        self.lap_time = trajectory.lap_time
        self.laps = trajectory.laps
        if not np.isclose(self.takeoff_height, trajectory.takeoff_height):
            raise ValueError('Original paper profiles require takeoff_height_m=10')
        return trajectory

    def __init__(self):
        super().__init__()
        self.takeoff_origin_ned = None
        self.start_sample = self.trajectory.staging_sample()
        self.get_logger().info(f'PAPER_TRAJECTORY {self.trajectory_name}: original duration {self.trajectory.total_duration:.6f} s, speed {self.speed:.3f} m/s')

    def _set_phase(self, name, now_ns):
        if name == 'takeoff':
            self.takeoff_origin_ned = self.origin.copy()
        if name in ('takeoff', 'entry') and self.trajectory.ground_start:
            # This profile already contains the ground-to-flight segment.
            # Begin on armed/offboard acquisition instead of dwelling armed
            # at zero motion/zero INDI force before its own takeoff starts.
            self.track_started_ns = now_ns
            name = 'track'
        super()._set_phase(name, now_ns)

    def _publish_hold_or_slew(self, position, velocity=None, *, tracking_feedback=False):
        if self.phase in ('prestream', 'takeoff'):
            from px4_msgs.msg import TrajectorySetpoint
            message = TrajectorySetpoint()
            message.timestamp = self.get_clock().now().nanoseconds // 1000
            message.position = np.asarray(position, float).tolist()
            message.velocity = (np.zeros(3) if velocity is None else np.asarray(velocity, float)).tolist()
            message.acceleration = [math.nan] * 3
            message.jerk = [math.nan] * 3
            message.yaw = self.trajectory.initial_yaw
            message.yawspeed = 0.0
            self.setpoint_publisher.publish(message)
        else:
            super()._publish_hold_or_slew(position, velocity, tracking_feedback=tracking_feedback)

    def _tracking_summary(self):
        result = super()._tracking_summary()
        result.update(trajectory_name=self.trajectory_name,
                      knife_entry_straight=self.trajectory.knife_entry_straight,
                      takeoff_origin_ned_m=(self.origin if self.takeoff_origin_ned is None
                                           else self.takeoff_origin_ned).tolist(),
                      reference_duration_s=self.trajectory.total_duration,
                      reference_position_translation_ned_m=self.trajectory.offset.tolist(),
                      reference_source_sha256='b2a2b64d5316a579ef5ae263c5b8d75af7568359f4c7a554b0cec97202831786')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = PaperTrajectoryMission()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return node.exit_code


if __name__ == '__main__':
    raise SystemExit(main())
