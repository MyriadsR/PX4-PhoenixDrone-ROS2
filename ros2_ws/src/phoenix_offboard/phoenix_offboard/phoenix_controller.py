"""Bug-compatible port of PX4-PhoenixDrone ts_att_control and ts_rate_control."""

import math
from typing import List, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from px4_msgs.msg import (
    ActuatorMotors,
    ActuatorServos,
    OffboardControlMode,
    VehicleAngularVelocity,
    VehicleAttitude,
    VehicleAttitudeSetpoint,
    VehicleControlMode,
)
from rclpy.node import Node

from .qos import PX4_INPUT_QOS, PX4_OUTPUT_QOS


Vector = List[float]
Matrix = List[List[float]]


def dot(a: Vector, b: Vector) -> float:
    return sum(x * y for x, y in zip(a, b))


def cross(a: Vector, b: Vector) -> Vector:
    return [a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]


def norm(v: Vector) -> float:
    return math.sqrt(dot(v, v))


def mat_mul(a: Matrix, b: Matrix) -> Matrix:
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def mat_vec(a: Matrix, v: Vector) -> Vector:
    return [dot(row, v) for row in a]


def transpose(a: Matrix) -> Matrix:
    return [[a[j][i] for j in range(3)] for i in range(3)]


def quat_to_dcm(q: Vector) -> Matrix:
    w, x, y, z = q
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ]


def dcm_to_quat(m: Matrix) -> Vector:
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        return [0.25 * s, (m[2][1] - m[1][2]) / s,
                (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s]
    idx = max(range(3), key=lambda i: m[i][i])
    if idx == 0:
        s = math.sqrt(1 + m[0][0] - m[1][1] - m[2][2]) * 2
        return [(m[2][1] - m[1][2]) / s, 0.25 * s,
                (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s]
    if idx == 1:
        s = math.sqrt(1 + m[1][1] - m[0][0] - m[2][2]) * 2
        return [(m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s,
                0.25 * s, (m[1][2] + m[2][1]) / s]
    s = math.sqrt(1 + m[2][2] - m[0][0] - m[1][1]) * 2
    return [(m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s,
            (m[1][2] + m[2][1]) / s, 0.25 * s]


def constrain(value: float, lower: float, upper: float) -> float:
    return min(upper, max(lower, value))


class PhoenixController(Node):
    # Constants from commit 70af6cbc.
    ATT_P = [8.0, 3.0, 3.0]
    RATE_MAX = [40.0, 100.0, 0.4]
    RATE_TC = [0.04, 0.06, 0.5]
    INERTIA = [0.0144, 0.00638929, 0.0176]
    KT = 7.864e-6
    KL = 3.48e-6
    KP = 3.44e-7
    KM = 0.023
    ARM = 0.3
    MAX_OMEGA = 800.0

    def __init__(self) -> None:
        super().__init__('phoenix_controller')
        self._attitude: Optional[VehicleAttitude] = None
        self._rates: Optional[VehicleAngularVelocity] = None
        self._setpoint: Optional[VehicleAttitudeSetpoint] = None
        self._input_setpoint: Optional[VehicleAttitudeSetpoint] = None
        self._input_setpoint_received_ns = 0
        self._armed = False

        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude',
                                 self._on_attitude, PX4_OUTPUT_QOS)
        self.create_subscription(VehicleAngularVelocity, '/fmu/out/vehicle_angular_velocity',
                                 self._on_rates, PX4_OUTPUT_QOS)
        self.create_subscription(VehicleAttitudeSetpoint, '/fmu/out/vehicle_attitude_setpoint',
                                 self._on_setpoint, PX4_OUTPUT_QOS)
        self.create_subscription(VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint',
                                 self._on_input_setpoint, PX4_INPUT_QOS)
        self.create_subscription(VehicleControlMode, '/fmu/out/vehicle_control_mode',
                                 self._on_control_mode, PX4_OUTPUT_QOS)
        self._mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_INPUT_QOS)
        self._motors_pub = self.create_publisher(
            ActuatorMotors, '/fmu/in/actuator_motors', PX4_INPUT_QOS)
        self._servos_pub = self.create_publisher(
            ActuatorServos, '/fmu/in/actuator_servos', PX4_INPUT_QOS)

    def _on_attitude(self, msg: VehicleAttitude) -> None:
        self._attitude = msg

    def _on_setpoint(self, msg: VehicleAttitudeSetpoint) -> None:
        self._setpoint = msg

    def _on_input_setpoint(self, msg: VehicleAttitudeSetpoint) -> None:
        self._input_setpoint = msg
        self._input_setpoint_received_ns = self.get_clock().now().nanoseconds

    def _on_control_mode(self, msg: VehicleControlMode) -> None:
        self._armed = msg.flag_armed

    def _on_rates(self, msg: VehicleAngularVelocity) -> None:
        self._rates = msg
        self._run_control()

    def _attitude_error(self, q: Vector, q_sp: Vector) -> Vector:
        r = quat_to_dcm(q)
        r_sp = quat_to_dcm(q_sp)
        r_z = [r[0][2], r[1][2], r[2][2]]
        r_sp_z = [r_sp[0][2], r_sp[1][2], r_sp[2][2]]
        e_r = mat_vec(transpose(r), cross(r_z, r_sp_z))
        sin_z = norm(e_r)
        cos_z = dot(r_z, r_sp_z)
        yaw_w = r_sp[2][2] * r_sp[2][2]
        r_rp = r
        if sin_z > 0.0:
            angle = math.atan2(sin_z, cos_z)
            axis = [v / sin_z for v in e_r]
            e_r = [v * angle for v in axis]
            k = [[0.0, -axis[2], axis[1]],
                 [axis[2], 0.0, -axis[0]],
                 [-axis[1], axis[0], 0.0]]
            k2 = mat_mul(k, k)
            rod = [[(1.0 if i == j else 0.0) + k[i][j] * sin_z
                    + k2[i][j] * (1.0 - cos_z) for j in range(3)] for i in range(3)]
            r_rp = mat_mul(r, rod)
        r_sp_x = [r_sp[0][0], r_sp[1][0], r_sp[2][0]]
        r_rp_x = [r_rp[0][0], r_rp[1][0], r_rp[2][0]]
        e_r[2] = math.atan2(dot(cross(r_rp_x, r_sp_x), r_sp_z),
                            dot(r_rp_x, r_sp_x)) * yaw_w
        if cos_z < 0.0:
            q_err = dcm_to_quat(mat_mul(transpose(r), r_sp))
            direct = [2.0 * v if q_err[0] >= 0 else -2.0 * v for v in q_err[1:]]
            direct_w = cos_z * cos_z * yaw_w
            e_r = [e_r[i] * (1.0 - direct_w) + direct[i] * direct_w for i in range(3)]
        return e_r

    def _mix(self, force_per_motor: float, moment: Vector) -> tuple[Vector, Vector]:
        mx, my, mz = moment
        l = self.ARM
        omega_l2 = (mx + 2.0 * force_per_motor * l) / (2.0 * self.KT * l)
        omega_r2 = -(mx - 2.0 * force_per_motor * l) / (2.0 * self.KT * l)
        den_r = self.KL * self.KP * l * (mx - 2.0 * force_per_motor * l)
        den_l = 2.0 * force_per_motor * self.KL * self.KP * l * l + self.KL * self.KP * mx * l
        delta_r = 0.0 if den_r == 0.0 else (
            self.KL * self.KT * my * l * l - self.KP * self.KT * mz * l
            + self.KM * self.KP * self.KT * mx) / den_r
        delta_l = 0.0 if den_l == 0.0 else -(
            self.KL * self.KT * my * l * l + self.KP * self.KT * mz * l
            - self.KM * self.KP * self.KT * mx) / den_l
        motors = [constrain(math.sqrt(max(0.0, omega_l2)) / self.MAX_OMEGA, -1.0, 1.0),
                  constrain(math.sqrt(max(0.0, omega_r2)) / self.MAX_OMEGA, -1.0, 1.0)]
        deg_r = constrain(math.degrees(delta_r), -60.0, 60.0)
        deg_l = constrain(math.degrees(delta_l), -60.0, 60.0)
        # Bridge ranges reproduce old -0.017444 rad/degree and joint clipping.
        servos = [constrain(deg_r / 30.0, -1.0, 1.0),
                  constrain(deg_l / 60.0, -1.0, 1.0)]
        return motors, servos

    def _run_control(self) -> None:
        if self._attitude is None or self._rates is None:
            return
        now_ns = self.get_clock().now().nanoseconds
        setpoint = (self._input_setpoint
                    if self._input_setpoint is not None
                    and now_ns - self._input_setpoint_received_ns < 500_000_000
                    else self._setpoint)
        if setpoint is None:
            return
        timestamp = now_ns // 1000
        mode = OffboardControlMode()
        mode.timestamp = timestamp
        mode.direct_actuator = True
        self._mode_pub.publish(mode)

        rates = list(self._rates.xyz)
        e_r = self._attitude_error(list(self._attitude.q), list(setpoint.q_d))
        rates_sp = [constrain(self.ATT_P[i] * e_r[i], -self.RATE_MAX[i], self.RATE_MAX[i])
                    for i in range(3)]
        rates_err = [rates_sp[i] - rates[i] for i in range(3)]
        j_rates = [self.INERTIA[i] * rates[i] for i in range(3)]
        gyroscopic = cross(rates, j_rates)
        moment = [self.INERTIA[i] * rates_err[i] / self.RATE_TC[i] + gyroscopic[i]
                  for i in range(3)]
        moment = [constrain(moment[i], -self.RATE_MAX[i], self.RATE_MAX[i]) for i in range(3)]
        force_per_motor = max(0.0, -float(setpoint.thrust_body[2]))
        motors, servos = self._mix(force_per_motor, moment) if self._armed else ([0.0, 0.0], [0.0, 0.0])

        motor_msg = ActuatorMotors()
        motor_msg.timestamp = timestamp
        motor_msg.timestamp_sample = self._rates.timestamp_sample
        motor_msg.control = motors + [math.nan] * (ActuatorMotors.NUM_CONTROLS - 2)
        self._motors_pub.publish(motor_msg)

        servo_msg = ActuatorServos()
        servo_msg.timestamp = timestamp
        servo_msg.timestamp_sample = self._rates.timestamp_sample
        servo_msg.control = servos + [math.nan] * (ActuatorServos.NUM_CONTROLS - 2)
        self._servos_pub.publish(servo_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PhoenixController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
