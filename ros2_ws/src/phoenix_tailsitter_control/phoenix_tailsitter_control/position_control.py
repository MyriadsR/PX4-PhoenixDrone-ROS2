"""Low-speed position and linear-acceleration INDI control in NED.

World vectors stay in PX4 NED.  Body-dependent gains and force estimates use
the Tailsitter-control body basis through the supplied ``R_ts_to_ned`` matrix.
"""

from dataclasses import dataclass, replace
import math

import numpy as np

from .math_utils import matrix_to_quaternion, normalize_quaternion, quaternion_to_matrix


@dataclass(frozen=True)
class TrajectoryReference:
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    jerk: np.ndarray
    yaw: float
    yawspeed: float
    position_mask: np.ndarray
    velocity_mask: np.ndarray
    acceleration_mask: np.ndarray


def _trajectory_vector(values, name):
    vector = np.asarray(values, dtype=float)
    if vector.shape != (3,) or np.any(np.isinf(vector)):
        raise ValueError(f'{name} must contain three finite values or NaN')
    return vector


def resolve_trajectory_reference(message, position, velocity, fallback_yaw):
    """Resolve PX4 NaN semantics without leaking NaNs into the controller."""
    current_position = _trajectory_vector(position, 'position state')
    current_velocity = _trajectory_vector(velocity, 'velocity state')
    if not np.all(np.isfinite(current_position)) or not np.all(np.isfinite(current_velocity)):
        raise ValueError('position and velocity state must be finite')

    raw_position = _trajectory_vector(message.position, 'position reference')
    raw_velocity = _trajectory_vector(message.velocity, 'velocity reference')
    raw_acceleration = _trajectory_vector(message.acceleration, 'acceleration reference')
    raw_jerk = _trajectory_vector(message.jerk, 'jerk reference')
    position_mask = np.isfinite(raw_position)
    explicit_velocity = np.isfinite(raw_velocity)
    acceleration_mask = np.isfinite(raw_acceleration)
    if not np.any(position_mask | explicit_velocity | acceleration_mask):
        raise ValueError('trajectory must control position, velocity, or acceleration')

    position_reference = np.where(position_mask, raw_position, current_position)
    # A finite position with unspecified velocity means zero velocity at that
    # position.  An entirely uncontrolled axis follows the current velocity.
    velocity_reference = np.where(
        explicit_velocity,
        raw_velocity,
        np.where(position_mask, 0.0, current_velocity),
    )
    velocity_mask = explicit_velocity | position_mask
    acceleration_reference = np.where(acceleration_mask, raw_acceleration, 0.0)
    jerk_reference = np.where(np.isfinite(raw_jerk), raw_jerk, 0.0)

    yaw = float(message.yaw)
    yawspeed = float(message.yawspeed)
    if math.isinf(yaw) or math.isinf(yawspeed):
        raise ValueError('yaw reference must be finite or NaN')
    if not math.isfinite(yaw):
        yaw = float(fallback_yaw)
    if not math.isfinite(yawspeed):
        yawspeed = 0.0
    return TrajectoryReference(
        position_reference,
        velocity_reference,
        acceleration_reference,
        jerk_reference,
        yaw,
        yawspeed,
        position_mask,
        velocity_mask,
        acceleration_mask,
    )


def trajectory_reference_age(timestamp_us, now_ns, arrival_ns,
                             maximum_age_s=0.10, clock_tolerance_s=0.50):
    """Use a common-clock publisher stamp, otherwise the local arrival clock.

    PX4-style stamps may be boot-relative whereas ROS stamps are epoch-relative.
    Never extrapolate across that clock-domain difference, into negative time,
    or past the bounded prediction horizon. Freshness gating remains separate.
    """
    if not math.isfinite(maximum_age_s) or maximum_age_s <= 0.0:
        raise ValueError('prediction horizon must be finite and positive')
    timestamp_ns = int(timestamp_us) * 1000
    common_clock = (timestamp_ns > 0 and
                    abs(timestamp_ns - arrival_ns) * 1e-9 <= clock_tolerance_s)
    origin_ns = timestamp_ns if common_clock else arrival_ns
    age = float(np.clip((now_ns - origin_ns) * 1e-9, 0.0, maximum_age_s))
    return age, common_clock


def advance_trajectory_reference(reference, age_s, *, advance_yaw=True):
    """Evaluate the message's p/v/a/jerk polynomial at the control time.

    Preserve the resolved control masks: acceleration-only commands must not
    silently acquire position or velocity feedback, and position-only holds
    still resolve unspecified derivatives to zero.
    """
    if not math.isfinite(age_s) or age_s < 0.0:
        raise ValueError('reference age must be finite and nonnegative')
    dt = float(age_s)
    a = reference.acceleration
    j = np.where(reference.acceleration_mask, reference.jerk, 0.0)
    p = reference.position + reference.velocity * dt + a * dt**2 / 2 + j * dt**3 / 6
    v = reference.velocity + a * dt + j * dt**2 / 2
    return replace(reference,
        position=np.where(reference.position_mask, p, reference.position),
        velocity=np.where(reference.velocity_mask, v, reference.velocity),
        acceleration=np.where(reference.acceleration_mask, a + j * dt, a),
        yaw=reference.yaw + (reference.yawspeed * dt if advance_yaw else 0.0))


def maneuver_gain_target(reference, config):
    """A stationary finite-acceleration reference is hover, not a maneuver.

    NaN-acceleration staging references retain their existing low-speed gains.
    Explicit trajectories blend continuously with requested motion, independent
    of tracking error or measured speed, so an oscillating hover stays in hover.
    """
    if not np.any(reference.acceleration_mask):
        return 0.0
    activity = max(
        float(np.linalg.norm(np.where(reference.velocity_mask, reference.velocity, 0.0))) / config.maneuver_speed_threshold,
        float(np.linalg.norm(np.where(reference.acceleration_mask, reference.acceleration, 0.0))) / config.maneuver_acceleration_threshold,
        abs(reference.yawspeed) / config.maneuver_yaw_rate_threshold)
    u = float(np.clip(activity, 0.0, 1.0))
    return u * u * (3.0 - 2.0 * u)


def update_gain_blend(current, target, dt, transition_s):
    if (not all(math.isfinite(x) for x in (current, target, dt, transition_s))
            or not 0 <= current <= 1 or not 0 <= target <= 1
            or dt <= 0 or transition_s <= 0):
        raise ValueError('gain blend requires bounded gains and positive timing')
    step = dt / transition_s
    return float(current + np.clip(target - current, -step, step))


class PositionController:
    """Body-gain position feedback ported from Tailsitter-control Eq. 41."""

    def __init__(self, config):
        self.cfg = config

    @staticmethod
    def _limit_norm(vector, limit):
        vector = np.asarray(vector, dtype=float)
        magnitude = float(np.linalg.norm(vector))
        return vector if magnitude <= limit else vector * (limit / magnitude)

    def acceleration_command(self, reference, position, velocity,
                             acceleration_lpf_ned, r_ts_to_ned,
                             tracking_blend=None, trajectory_mode=False):
        position_error = np.where(
            reference.position_mask, reference.position - position, 0.0)
        velocity_error = np.where(
            reference.velocity_mask, reference.velocity - velocity, 0.0)
        acceleration_error = reference.acceleration - acceleration_lpf_ned

        position_error = np.clip(
            position_error, -self.cfg.position_error_limit,
            self.cfg.position_error_limit)
        velocity_error = np.clip(
            velocity_error, -self.cfg.velocity_error_limit,
            self.cfg.velocity_error_limit)
        if tracking_blend is None:
            tracking_blend = float(np.any(reference.acceleration_mask))
        if not math.isfinite(tracking_blend) or not 0 <= tracking_blend <= 1:
            raise ValueError('tracking blend must be between zero and one')
        position_gain = self.cfg.position_gain + tracking_blend * (
            self.cfg.tracking_position_gain - self.cfg.position_gain)
        velocity_gain = self.cfg.velocity_gain + tracking_blend * (
            self.cfg.tracking_velocity_gain - self.cfg.velocity_gain)
        r_ned_to_ts = np.asarray(r_ts_to_ned, dtype=float).T
        feedback_ts = (
            position_gain * (r_ned_to_ts @ position_error)
            + velocity_gain * (r_ned_to_ts @ velocity_error)
            + self.cfg.linear_acceleration_gain
            * (r_ned_to_ts @ acceleration_error)
        )
        feedback_ned = r_ts_to_ned @ feedback_ts
        low_horizontal = (self.cfg.trajectory_hover_horizontal_acceleration_limit
                          if trajectory_mode else self.cfg.horizontal_acceleration_limit)
        low_vertical = (self.cfg.trajectory_hover_vertical_acceleration_limit
                        if trajectory_mode else self.cfg.vertical_acceleration_limit)
        horizontal_limit = low_horizontal + tracking_blend * (
            self.cfg.tracking_horizontal_acceleration_limit - low_horizontal)
        vertical_limit = low_vertical + tracking_blend * (
            self.cfg.tracking_vertical_acceleration_limit - low_vertical)
        horizontal_feedback = self._limit_norm(
            feedback_ned[:2], horizontal_limit)
        limited_feedback = np.array([
            horizontal_feedback[0],
            horizontal_feedback[1],
            np.clip(
                feedback_ned[2],
                -vertical_limit,
                vertical_limit,
            ),
        ])
        return reference.acceleration + limited_feedback


class LinearAccelerationINDIController:
    """Incremental force controller corresponding to Eq. 46."""

    def __init__(self, config):
        self.cfg = config

    def estimate_force_components_ts(self, motor_speed_left_right,
                                     flap_angle_left_right):
        omega = np.asarray(motor_speed_left_right, dtype=float)
        flap = np.asarray(flap_angle_left_right, dtype=float)
        if omega.shape != (2,) or flap.shape != (2,):
            raise ValueError('force estimate requires two motors and two flaps')
        omega_squared = omega ** 2
        thrust = self.cfg.motor_thrust_coefficient * np.sum(omega_squared)
        flap_lift = self.cfg.flap_lift_coefficient * np.sum(omega_squared * flap)
        flap_drag = self.cfg.flap_drag_coefficient * np.sum(omega_squared * flap ** 2)
        # PhoenixAero uses upward_gz=+x and forward_gz=+z.  The combined
        # Gazebo FLU -> PX4 FRD -> TS mapping is [z_gz,-y_gz,x_gz], hence
        # flap lift is +z_ts and quadratic drag is -x_ts.
        propulsion = np.array([thrust, 0.0, 0.0])
        aerodynamic = np.array([-flap_drag, 0.0, flap_lift])
        return propulsion, aerodynamic

    def estimate_force_ts(self, motor_speed_left_right, flap_angle_left_right):
        propulsion, aerodynamic = self.estimate_force_components_ts(
            motor_speed_left_right, flap_angle_left_right)
        return propulsion + aerodynamic

    def force_command(self, acceleration_command_ned, acceleration_lpf_ned,
                      estimated_force_lpf_ned):
        values = (
            np.asarray(acceleration_command_ned, dtype=float),
            np.asarray(acceleration_lpf_ned, dtype=float),
            np.asarray(estimated_force_lpf_ned, dtype=float),
        )
        if any(value.shape != (3,) or not np.all(np.isfinite(value)) for value in values):
            raise ValueError('linear INDI inputs must be finite 3-vectors')
        gravity_ned = np.array([0.0, 0.0, self.cfg.gravity])
        nominal_force = self.cfg.mass * (values[0] - gravity_ned)
        indi_force = values[2] + self.cfg.mass * (values[0] - values[1])
        blend = self.cfg.linear_indi_blend
        return nominal_force + blend * (indi_force - nominal_force)


def apply_takeoff_force_floor(force_ned, nominal_force_ned, estimated_force_ned,
                              position_ned, reference, config, *, ground_only=False):
    """Keep ground-start INDI takeoff from losing gravity compensation."""
    force = np.asarray(force_ned, dtype=float).copy()
    nominal = np.asarray(nominal_force_ned, dtype=float)
    estimated = np.asarray(estimated_force_ned, dtype=float)
    position = np.asarray(position_ned, dtype=float)
    values = (force, nominal, estimated, position)
    if any(value.shape != (3,) or not np.all(np.isfinite(value)) for value in values):
        raise ValueError('takeoff force floor inputs must be finite 3-vectors')

    # PX4 local NED is referenced to the takeoff surface in this SITL path.
    # An airborne loss of lift must not cancel the maneuver's horizontal force.
    if ground_only and -position[2] > 2.0:
        return force

    climbing_reference = (
        bool(reference.position_mask[2] and reference.position[2] < position[2] - 0.05)
        or bool(reference.velocity_mask[2] and reference.velocity[2] < -0.05)
        or bool(reference.acceleration[2] < -0.2)
    )
    upward_force_estimate = max(0.0, -float(estimated[2]))
    if (climbing_reference
            and upward_force_estimate
            < config.takeoff_force_floor_trigger_scale * config.hover_total_thrust):
        # NED z force is negative upward, so the smaller value is the stronger
        # upward command.  Suppress horizontal force until the motors have built
        # enough estimated lift for the regular INDI loop to take over.
        upward_floor = min(
            max(0.0, -float(nominal[2])),
            config.takeoff_force_floor_max_scale * config.hover_total_thrust,
        )
        force[:2] = 0.0
        force[2] = min(force[2], -upward_floor)
    return force


class HoverForceSlewLimiter:
    """Rate-limit horizontal force so it cannot outrun the attitude loop."""

    def __init__(self, horizontal_rate_n_s):
        rate = float(horizontal_rate_n_s)
        if not math.isfinite(rate) or rate <= 0.0:
            raise ValueError('horizontal force slew rate must be positive')
        self.horizontal_rate_n_s = rate
        self.value = None

    def reset(self):
        self.value = None

    def update(self, force_ned, dt):
        force = np.asarray(force_ned, dtype=float)
        dt = float(dt)
        if force.shape != (3,) or not np.all(np.isfinite(force)):
            raise ValueError('force must be a finite NED 3-vector')
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError('force slew limiter dt must be positive')
        if self.value is None:
            # Start at vertical hover force.  Initial horizontal position error
            # is introduced at the same bounded rate as later reversals.
            self.value = np.array([0.0, 0.0, force[2]])
        delta = force[:2] - self.value[:2]
        magnitude = float(np.linalg.norm(delta))
        maximum_delta = self.horizontal_rate_n_s * dt
        if magnitude > maximum_delta:
            delta *= maximum_delta / magnitude
        self.value[:2] += delta
        # Vertical thrust does not require an attitude change and keeps its
        # existing position-loop limit rather than inheriting horizontal lag.
        self.value[2] = force[2]
        return self.value.copy()


def tailsitter_heading_from_attitude(q_ts):
    """Return the horizontal heading of the TS z axis in NED."""
    body_z_ned = quaternion_to_matrix(normalize_quaternion(q_ts))[:, 2]
    if np.linalg.norm(body_z_ned[:2]) < 1e-6:
        return 0.0
    return math.atan2(body_z_ned[1], body_z_ned[0])


def limit_hover_force(force_ned, config):
    """Apply low-speed thrust and tilt limits to a non-gravity force."""
    force = np.asarray(force_ned, dtype=float)
    if force.shape != (3,) or not np.all(np.isfinite(force)):
        raise ValueError('force must be a finite NED 3-vector')
    upward = np.clip(
        -force[2], config.minimum_position_thrust, config.maximum_position_thrust)
    horizontal = PositionController._limit_norm(
        force[:2], upward * math.tan(config.position_tilt_limit_rad))
    limited = np.array([horizontal[0], horizontal[1], -upward])
    magnitude = float(np.linalg.norm(limited))
    if magnitude > config.maximum_position_thrust:
        limited *= config.maximum_position_thrust / magnitude
    return limited


def force_to_tailsitter_attitude(force_ned, yaw):
    """Align TS +x with force while TS +z follows the NED yaw heading."""
    force = np.asarray(force_ned, dtype=float)
    magnitude = float(np.linalg.norm(force))
    if force.shape != (3,) or not np.all(np.isfinite(force)) or magnitude < 1e-6:
        raise ValueError('force direction is undefined')
    body_x = force / magnitude
    heading = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    body_y = np.cross(heading, body_x)
    if np.linalg.norm(body_y) < 1e-6:
        heading = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
        body_y = np.cross(heading, body_x)
    body_y /= np.linalg.norm(body_y)
    body_z = np.cross(body_x, body_y)
    body_z /= np.linalg.norm(body_z)
    rotation = np.column_stack((body_x, body_y, body_z))
    return matrix_to_quaternion(rotation), magnitude


def degraded_vertical_force(z, vz, z_reference, vz_reference,
                            acceleration_feedforward, config):
    """Build a bounded vertical force when horizontal navigation is invalid.

    PX4 NED uses positive-down z.  Keeping this small fallback independent of
    x/y prevents a transient loss of horizontal EKF validity from turning a
    recoverable navigation degradation into a zero-thrust free fall.
    """
    values = np.asarray([
        z, vz, z_reference, vz_reference, acceleration_feedforward,
    ], dtype=float)
    if not np.all(np.isfinite(values)):
        raise ValueError('degraded vertical-control inputs must be finite')
    position_error = float(np.clip(
        z_reference - z,
        -config.position_error_limit,
        config.position_error_limit,
    ))
    velocity_error = float(np.clip(
        vz_reference - vz,
        -config.velocity_error_limit,
        config.velocity_error_limit,
    ))
    acceleration = (
        acceleration_feedforward
        + config.position_gain[0] * position_error
        + config.velocity_gain[0] * velocity_error
    )
    acceleration = float(np.clip(
        acceleration,
        -config.vertical_acceleration_limit,
        config.vertical_acceleration_limit,
    ))
    upward_force = float(np.clip(
        config.mass * (config.gravity - acceleration),
        config.minimum_position_thrust,
        config.maximum_position_thrust,
    ))
    return np.array([0.0, 0.0, -upward_force])
