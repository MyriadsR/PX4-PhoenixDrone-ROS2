"""Bug-compatible ROS 2 port of the commit-70af6cbc mc_pos_control law."""

import math
from typing import Optional

import rclpy
from px4_msgs.msg import TrajectorySetpoint, VehicleAttitudeSetpoint, VehicleLocalPosition
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .phoenix_controller import cross, dcm_to_quat, norm
from .qos import PX4_INPUT_QOS, PX4_OUTPUT_QOS


class PhoenixPositionController(Node):
    # Original TS_POS_* defaults and hard-coded mass from commit 70af6cbc.
    POS_TC = [1.0, 1.0, 0.2]
    POS_DR = [2.0 * 0.6, 2.0 * 0.6, 2.0 * 1.5]
    MASS = 0.63
    GRAVITY = 9.8066
    THR_MIN = 0.2
    THR_MAX = 10.0
    TILT_MAX = math.radians(30.0)
    SIGMA = 1e-6

    def __init__(self) -> None:
        super().__init__('phoenix_position_controller')
        self._trajectory: Optional[TrajectorySetpoint] = None
        self._trajectory_received_ns = 0
        self.create_subscription(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint',
            self._on_trajectory, PX4_INPUT_QOS)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._on_local_position, PX4_OUTPUT_QOS)
        self._attitude_pub = self.create_publisher(
            VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', PX4_INPUT_QOS)

    def _on_trajectory(self, msg: TrajectorySetpoint) -> None:
        self._trajectory = msg
        self._trajectory_received_ns = self.get_clock().now().nanoseconds

    @staticmethod
    def _finite(values) -> bool:
        return all(math.isfinite(float(value)) for value in values)

    def _limit_force(self, force):
        # Same offboard branch and ordering as the old mc_pos_control implementation.
        if -force[2] < self.THR_MIN:
            force[2] = -self.THR_MIN

        force_xy = math.hypot(force[0], force[1])
        if force_xy > 0.01:
            force_xy_max = -force[2] * math.tan(self.TILT_MAX)
            if force_xy > force_xy_max:
                scale = force_xy_max / force_xy
                force[0] *= scale
                force[1] *= scale

        force_abs = norm(force)
        if force_abs > self.THR_MAX:
            if force[2] < 0.0:
                if -force[2] > self.THR_MAX:
                    force = [0.0, 0.0, -self.THR_MAX]
                else:
                    force_xy_max = math.sqrt(
                        self.THR_MAX * self.THR_MAX - force[2] * force[2])
                    force_xy = math.hypot(force[0], force[1])
                    scale = force_xy_max / force_xy
                    force[0] *= scale
                    force[1] *= scale
            else:
                scale = self.THR_MAX / force_abs
                force = [value * scale for value in force]
        return force

    def _force_to_attitude(self, force):
        force_abs = norm(force)
        body_z = ([-value / force_abs for value in force]
                  if force_abs > self.SIGMA else [0.0, 0.0, 1.0])

        # Preserve the original yaw-validity condition exactly. For every finite yaw it
        # overwrote y_C with [-1, 0, 0], so position commands use this fixed heading.
        y_c = [-1.0, 0.0, 0.0]
        if abs(body_z[2]) > self.SIGMA:
            body_x = cross(y_c, body_z)
            if body_z[2] < 0.0:
                body_x = [0.0, 1.0, 0.0]
            body_x_abs = norm(body_x)
            body_x = [value / body_x_abs for value in body_x]
        else:
            body_z = [0.0, 0.0, 1.0]
            body_x = [0.0, 1.0, 0.0]
        body_y = cross(body_z, body_x)
        rotation = [[body_x[i], body_y[i], body_z[i]] for i in range(3)]
        return dcm_to_quat(rotation), force_abs / 2.0

    def _on_local_position(self, local: VehicleLocalPosition) -> None:
        trajectory = self._trajectory
        if trajectory is None:
            return
        now = self.get_clock().now()
        if now.nanoseconds - self._trajectory_received_ns > 500_000_000:
            return
        if not (local.xy_valid and local.z_valid and
                local.v_xy_valid and local.v_z_valid):
            return

        position = [float(local.x), float(local.y), float(local.z)]
        velocity = [float(local.vx), float(local.vy), float(local.vz)]
        if not self._finite(position + velocity):
            return

        if self._finite(trajectory.acceleration):
            acceleration = [float(value) for value in trajectory.acceleration]
        else:
            position_sp = ([float(value) for value in trajectory.position]
                           if self._finite(trajectory.position) else position)
            velocity_sp = ([float(value) for value in trajectory.velocity]
                           if self._finite(trajectory.velocity) else [0.0, 0.0, 0.0])
            acceleration = [
                (position_sp[i] - position[i]) / (self.POS_TC[i] ** 2)
                + (velocity_sp[i] - velocity[i]) / self.POS_TC[i] * self.POS_DR[i]
                for i in range(3)
            ]

        force = [self.MASS * acceleration[0],
                 self.MASS * acceleration[1],
                 self.MASS * acceleration[2] - self.MASS * self.GRAVITY]
        force = self._limit_force(force)
        quaternion, force_per_motor = self._force_to_attitude(force)

        setpoint = VehicleAttitudeSetpoint()
        setpoint.timestamp = now.nanoseconds // 1000
        setpoint.q_d = quaternion
        setpoint.thrust_body = [0.0, 0.0, -force_per_motor]
        setpoint.yaw_sp_move_rate = 0.0
        self._attitude_pub.publish(setpoint)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PhoenixPositionController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
