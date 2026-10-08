"""主仿真使用的分段轨迹和 Bernoulli 双纽线测试轨迹。"""

import numpy as np


def quintic_segment(distance, tau, duration):
    """计算静止端点五次多项式的位置、速度、加速度和 jerk。"""
    s = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
    s_dot = (30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4) / duration
    s_ddot = (60.0 * tau - 180.0 * tau**2 + 120.0 * tau**3) / duration**2
    s_dddot = (60.0 - 360.0 * tau + 360.0 * tau**2) / duration**3
    return distance * s, distance * s_dot, distance * s_ddot, distance * s_dddot


def _bernoulli_lemniscate_derivatives(parameter):
    """返回单位 Bernoulli 双纽线对参数 u 的 0～3 阶导数。

    曲线采用

        x = cos(u) / (1 + sin(u)^2)
        y = sin(u) cos(u) / (1 + sin(u)^2)

    最后一个数组维度依次为 ``[x, y]``。
    """
    u = np.asarray(parameter, dtype=float)
    sin_u = np.sin(u)
    cos_u = np.cos(u)

    denominator = 1.0 + sin_u**2
    denominator_1 = 2.0 * sin_u * cos_u
    denominator_2 = 2.0 * (cos_u**2 - sin_u**2)
    denominator_3 = -8.0 * sin_u * cos_u

    inverse = 1.0 / denominator
    inverse_1 = -denominator_1 / denominator**2
    inverse_2 = (
        2.0 * denominator_1**2 / denominator**3
        - denominator_2 / denominator**2
    )
    inverse_3 = (
        -6.0 * denominator_1**3 / denominator**4
        + 6.0 * denominator_1 * denominator_2 / denominator**3
        - denominator_3 / denominator**2
    )

    x_numerator = (cos_u, -sin_u, -cos_u, sin_u)
    y_numerator = (
        sin_u * cos_u,
        cos_u**2 - sin_u**2,
        -4.0 * sin_u * cos_u,
        -4.0 * (cos_u**2 - sin_u**2),
    )

    def quotient_derivatives(numerator):
        n_0, n_1, n_2, n_3 = numerator
        value = n_0 * inverse
        first = n_1 * inverse + n_0 * inverse_1
        second = n_2 * inverse + 2.0 * n_1 * inverse_1 + n_0 * inverse_2
        third = (
            n_3 * inverse
            + 3.0 * n_2 * inverse_1
            + 3.0 * n_1 * inverse_2
            + n_0 * inverse_3
        )
        return value, first, second, third

    x_derivatives = quotient_derivatives(x_numerator)
    y_derivatives = quotient_derivatives(y_numerator)
    return tuple(
        np.stack((x_derivatives[order], y_derivatives[order]), axis=-1)
        for order in range(4)
    )


def _quintic_smoothstep(tau):
    """返回五次 smoothstep 及其对无量纲参数 tau 的一、二阶导数。"""
    tau = np.clip(float(tau), 0.0, 1.0)
    value = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
    first = 30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4
    second = 60.0 * tau - 180.0 * tau**2 + 120.0 * tau**3
    return value, first, second


def _quintic_smoothstep_integral(tau):
    """五次 smoothstep 从 0 到 tau 的积分。"""
    tau = np.clip(float(tau), 0.0, 1.0)
    return 2.5 * tau**4 - 3.0 * tau**5 + tau**6


def _septic_smoothstep(tau):
    """返回七次 smoothstep 及其对无量纲参数 tau 的一、二阶导数。"""
    tau = np.clip(float(tau), 0.0, 1.0)
    value = 35.0 * tau**4 - 84.0 * tau**5 + 70.0 * tau**6 - 20.0 * tau**7
    first = (
        140.0 * tau**3
        - 420.0 * tau**4
        + 420.0 * tau**5
        - 140.0 * tau**6
    )
    second = (
        420.0 * tau**2
        - 1680.0 * tau**3
        + 2100.0 * tau**4
        - 840.0 * tau**5
    )
    return value, first, second


def _septic_smoothstep_integral(tau):
    """七次 smoothstep 从 0 到 tau 的积分。"""
    tau = np.clip(float(tau), 0.0, 1.0)
    return (
        7.0 * tau**5
        - 14.0 * tau**6
        + 10.0 * tau**7
        - 2.5 * tau**8
    )


class TrajectoryGenerator:
    """生成起飞、前移和右移组成的 NED 分段测试轨迹。"""

    def __init__(
        self,
        takeoff_duration=6.0,
        hover_1_duration=5.0,
        forward_duration=6.0,
        hover_2_duration=5.0,
        right_duration=6.0,
        hover_3_duration=5.0,
        takeoff_height=10.0,
        forward_distance=10.0,
        right_distance=10.0,
        yaw_ref=0.0,
    ):
        self.initial_state_mode = "ground"
        moving_durations = (takeoff_duration, forward_duration, right_duration)
        hover_durations = (hover_1_duration, hover_2_duration, hover_3_duration)
        if any(duration <= 0.0 for duration in moving_durations):
            raise ValueError("运动轨迹段时长必须大于 0")
        if any(duration < 0.0 for duration in hover_durations):
            raise ValueError("悬停轨迹段时长不能为负")

        self.takeoff_duration = float(takeoff_duration)
        self.hover_1_duration = float(hover_1_duration)
        self.forward_duration = float(forward_duration)
        self.hover_2_duration = float(hover_2_duration)
        self.right_duration = float(right_duration)
        self.hover_3_duration = float(hover_3_duration)
        self.takeoff_height = float(takeoff_height)
        self.forward_distance = float(forward_distance)
        self.right_distance = float(right_distance)
        self.yaw_ref = float(yaw_ref)

        self.t_takeoff_end = self.takeoff_duration
        self.t_hover_1_end = self.t_takeoff_end + self.hover_1_duration
        self.t_forward_end = self.t_hover_1_end + self.forward_duration
        self.t_hover_2_end = self.t_forward_end + self.hover_2_duration
        self.t_right_end = self.t_hover_2_end + self.right_duration
        self.total_duration = self.t_right_end + self.hover_3_duration

    def sample(self, t):
        """返回 x_ref、v_ref、a_ref、j_ref、psi_ref 和 psi_dot_ref。"""
        t = float(t)
        if t < 0.0:
            raise ValueError("轨迹采样时间不能为负")

        altitude_ned = -self.takeoff_height

        if t <= self.t_takeoff_end:
            tau = t / self.takeoff_duration
            z_pos, z_vel, z_acc, z_jerk = quintic_segment(
                altitude_ned, tau, self.takeoff_duration
            )
            x_ref = np.array([0.0, 0.0, z_pos])
            v_ref = np.array([0.0, 0.0, z_vel])
            a_ref = np.array([0.0, 0.0, z_acc])
            j_ref = np.array([0.0, 0.0, z_jerk])
        elif t <= self.t_hover_1_end:
            x_ref = np.array([0.0, 0.0, altitude_ned])
            v_ref = np.zeros(3)
            a_ref = np.zeros(3)
            j_ref = np.zeros(3)
        elif t <= self.t_forward_end:
            tau = (t - self.t_hover_1_end) / self.forward_duration
            x_pos, x_vel, x_acc, x_jerk = quintic_segment(
                self.forward_distance, tau, self.forward_duration
            )
            x_ref = np.array([x_pos, 0.0, altitude_ned])
            v_ref = np.array([x_vel, 0.0, 0.0])
            a_ref = np.array([x_acc, 0.0, 0.0])
            j_ref = np.array([x_jerk, 0.0, 0.0])
        elif t <= self.t_hover_2_end:
            x_ref = np.array([self.forward_distance, 0.0, altitude_ned])
            v_ref = np.zeros(3)
            a_ref = np.zeros(3)
            j_ref = np.zeros(3)
        elif t <= self.t_right_end:
            tau = (t - self.t_hover_2_end) / self.right_duration
            y_pos, y_vel, y_acc, y_jerk = quintic_segment(
                self.right_distance, tau, self.right_duration
            )
            x_ref = np.array([self.forward_distance, y_pos, altitude_ned])
            v_ref = np.array([0.0, y_vel, 0.0])
            a_ref = np.array([0.0, y_acc, 0.0])
            j_ref = np.array([0.0, y_jerk, 0.0])
        else:
            x_ref = np.array(
                [self.forward_distance, self.right_distance, altitude_ned]
            )
            v_ref = np.zeros(3)
            a_ref = np.zeros(3)
            j_ref = np.zeros(3)

        psi_ref = self.yaw_ref
        psi_dot_ref = 0.0
        return x_ref, v_ref, a_ref, j_ref, psi_ref, psi_dot_ref


class LemniscateTrajectory:
    """论文飞行测试使用的恒速 Bernoulli 双纽线轨迹。

    默认参数对应论文 Sec. VI-B：空速 6 m/s、每圈 7 s、连续 8 圈，
    并从曲线的正 Y 极值处开始。轨迹位于 NED 世界系的水平面内，默认
    高度为 ``z=-10 m``。取 ``psi=atan2(v_y, v_x)``，使偏航中间系的
    Y 轴（以及平飞时的机体横轴）与速度垂直，从而构造论文所说的
    coordinated flight 参考。

    ``speed * lap_time`` 决定一圈周长，曲线尺度据此自动计算。参数 u
    经过数值弧长反解，因此是沿曲线恒速飞行，而不是简单令 u 匀速。
    """

    def __init__(
        self,
        speed=6.0,
        lap_time=7.0,
        laps=8,
        center=(0.0, 0.0, -10.0),
        arc_samples=20001,
    ):
        if speed <= 0.0:
            raise ValueError("双纽线飞行速度必须大于 0")
        if lap_time <= 0.0:
            raise ValueError("双纽线单圈时长必须大于 0")
        if int(laps) != laps or laps <= 0:
            raise ValueError("双纽线圈数必须为正整数")
        if int(arc_samples) != arc_samples or arc_samples < 1001:
            raise ValueError("弧长查找表采样数必须是且至少为 1001 的整数")

        center_array = np.asarray(center, dtype=float)
        if center_array.shape != (3,):
            raise ValueError("center 必须是包含三个 NED 坐标的向量")

        self.speed = float(speed)
        self.lap_time = float(lap_time)
        self.laps = int(laps)
        self.center = center_array.copy()
        self.total_duration = self.laps * self.lap_time
        self.path_length = self.speed * self.lap_time
        self.initial_state_mode = "reference"

        # 先计算单位曲线的弧长，再将其缩放到 speed * lap_time。
        self._u_table = np.linspace(0.0, 2.0 * np.pi, int(arc_samples))
        _, tangent_unit, _, _ = _bernoulli_lemniscate_derivatives(
            self._u_table
        )
        parameter_speed = np.linalg.norm(tangent_unit, axis=1)
        delta_u = np.diff(self._u_table)
        delta_arc_unit = 0.5 * (
            parameter_speed[:-1] + parameter_speed[1:]
        ) * delta_u
        arc_unit = np.concatenate(([0.0], np.cumsum(delta_arc_unit)))
        self.scale = self.path_length / arc_unit[-1]
        self._arc_table = self.scale * arc_unit

        # 对当前参数式，正 Y 极值满足 tan(u)=1/sqrt(2)。使用解析相位，
        # 再从弧长表插值得到对应弧长，避免初始相位受表格分辨率影响。
        self.start_parameter = float(np.arctan(1.0 / np.sqrt(2.0)))
        self._start_arc = float(
            np.interp(self.start_parameter, self._u_table, self._arc_table)
        )

    def sample(self, t):
        """返回 x_ref、v_ref、a_ref、j_ref、psi_ref 和 psi_dot_ref。"""
        t = float(t)
        if t < 0.0:
            raise ValueError("轨迹采样时间不能为负")

        # 轨迹本身按圈周期延拓；total_duration 只定义主仿真的停止时刻。
        distance = np.mod(self._start_arc + self.speed * t, self.path_length)
        parameter = float(np.interp(distance, self._arc_table, self._u_table))

        position_u, first_u, second_u, third_u = (
            self.scale * derivative
            for derivative in _bernoulli_lemniscate_derivatives(parameter)
        )

        # 由 ds/dt=speed 对 u(t) 作链式求导。这样返回的速度、加速度和
        # jerk 与位置属于同一条恒速轨迹，可直接用于微分平坦前馈。
        tangent_norm = np.linalg.norm(first_u)
        tangent_curvature = float(np.dot(first_u, second_u))
        u_dot = self.speed / tangent_norm
        u_ddot = -(self.speed**2) * tangent_curvature / tangent_norm**4
        u_dddot = -(self.speed**3) * (
            (np.dot(second_u, second_u) + np.dot(first_u, third_u))
            / tangent_norm**5
            - 4.0 * tangent_curvature**2 / tangent_norm**7
        )

        velocity_xy = first_u * u_dot
        acceleration_xy = second_u * u_dot**2 + first_u * u_ddot
        jerk_xy = (
            third_u * u_dot**3
            + 3.0 * second_u * u_dot * u_ddot
            + first_u * u_dddot
        )

        x_ref = self.center + np.array([position_u[0], position_u[1], 0.0])
        v_ref = np.array([velocity_xy[0], velocity_xy[1], 0.0])
        a_ref = np.array([acceleration_xy[0], acceleration_xy[1], 0.0])
        j_ref = np.array([jerk_xy[0], jerk_xy[1], 0.0])

        # ψ 中间系的 X 轴沿速度方向，其 Y 轴垂直速度，满足论文 coordinated
        # flight 的 yaw 约束。ψ 属于 S1，跨 ±pi 时数值回绕但方向和 ψ_dot 连续。
        psi_ref = float(np.arctan2(v_ref[1], v_ref[0]))
        psi_dot_ref = float(
            (v_ref[0] * a_ref[1] - v_ref[1] * a_ref[0])
            / (v_ref[0] ** 2 + v_ref[1] ** 2)
        )
        return x_ref, v_ref, a_ref, j_ref, psi_ref, psi_dot_ref


class KnifeEdgeTransitionTrajectory:
    """论文 Sec. VI-C 的恒速椭圆跑道及 knife-edge/coordinated yaw 变换。

    论文给出 6 m/s、6.25 s/圈、8 圈和约 1.6g 转弯，但没有公布完整
    解析曲线。本实现依据 Fig. 7 使用 3 m 转弯半径和由总周长反算出的
    9.325 m 直线段。转弯曲率使用平滑进入/退出，避免理想体育场曲线在
    直线与圆弧连接处产生无限 jerk。
    """

    def __init__(
        self,
        speed=6.0,
        lap_time=6.25,
        laps=8,
        turn_radius=3.0,
        curvature_blend=0.6,
        center=(0.0, 0.0, -10.0),
        arc_samples=40001,
    ):
        if speed <= 0.0 or lap_time <= 0.0:
            raise ValueError("速度和单圈时间必须大于 0")
        if int(laps) != laps or laps <= 0:
            raise ValueError("圈数必须为正整数")
        if turn_radius <= 0.0:
            raise ValueError("转弯半径必须大于 0")
        if int(arc_samples) != arc_samples or arc_samples < 4001:
            raise ValueError("arc_samples 必须是且至少为 4001 的整数")

        center_array = np.asarray(center, dtype=float)
        if center_array.shape != (3,):
            raise ValueError("center 必须是包含三个 NED 坐标的向量")

        self.speed = float(speed)
        self.lap_time = float(lap_time)
        self.laps = int(laps)
        self.turn_radius = float(turn_radius)
        self.path_length = self.speed * self.lap_time
        self.turn_length = np.pi * self.turn_radius
        self.straight_length = 0.5 * (
            self.path_length - 2.0 * self.turn_length
        )
        if self.straight_length <= 0.0:
            raise ValueError("总周长不足以容纳两个指定半径的半圆")
        if not 0.0 < curvature_blend < 0.5 * self.turn_length:
            raise ValueError("curvature_blend 必须位于 (0, turn_length/2) 内")

        self.curvature_blend = float(curvature_blend)
        self._turn_curvature = np.pi / (
            self.turn_length - self.curvature_blend
        )
        self._half_length = self.straight_length + self.turn_length
        self.center = center_array.copy()
        self.total_duration = self.laps * self.lap_time
        self.initial_state_mode = "reference"

        # s 本身就是弧长。只需数值积分单位切向量获得位置；速度及更高阶
        # 导数仍由解析曲率关系计算，不对插值位置作噪声敏感的数值微分。
        self._arc_table = np.linspace(0.0, self.path_length, int(arc_samples))
        heading_table = np.array(
            [self._geometry_at_arc(s)[0] for s in self._arc_table]
        )
        tangent_table = np.column_stack(
            (np.cos(heading_table), np.sin(heading_table))
        )
        delta_s = np.diff(self._arc_table)
        delta_position = 0.5 * (
            tangent_table[:-1] + tangent_table[1:]
        ) * delta_s[:, None]
        position_table = np.vstack(
            (np.zeros(2), np.cumsum(delta_position, axis=0))
        )
        closure_error = np.linalg.norm(position_table[-1] - position_table[0])
        if closure_error > 1e-7:
            raise RuntimeError(f"平滑椭圆跑道未闭合: {closure_error:.3e} m")

        bounds_center = 0.5 * (
            np.min(position_table[:-1], axis=0)
            + np.max(position_table[:-1], axis=0)
        )
        self._position_table = position_table - bounds_center

    def _turn_profile(self, local_arc):
        """返回单个平滑半圆的转角、曲率和曲率弧长导数。"""
        local_arc = np.clip(float(local_arc), 0.0, self.turn_length)
        blend = self.curvature_blend
        curvature = self._turn_curvature

        if local_arc < blend:
            tau = local_arc / blend
            smooth, smooth_1, _ = _quintic_smoothstep(tau)
            angle = curvature * blend * _quintic_smoothstep_integral(tau)
            return angle, curvature * smooth, curvature * smooth_1 / blend

        if local_arc <= self.turn_length - blend:
            angle = curvature * (0.5 * blend + local_arc - blend)
            return angle, curvature, 0.0

        remaining = self.turn_length - local_arc
        tau = remaining / blend
        smooth, smooth_1, _ = _quintic_smoothstep(tau)
        angle = np.pi - curvature * blend * _quintic_smoothstep_integral(tau)
        return angle, curvature * smooth, -curvature * smooth_1 / blend

    def _geometry_at_arc(self, arc):
        """返回弧长位置处的切线航向、曲率和曲率导数。"""
        arc = float(np.mod(arc, self.path_length))
        if arc < self.straight_length:
            return 0.5 * np.pi, 0.0, 0.0
        if arc < self._half_length:
            angle, curvature, curvature_s = self._turn_profile(
                arc - self.straight_length
            )
            return 0.5 * np.pi + angle, curvature, curvature_s
        if arc < self._half_length + self.straight_length:
            return 1.5 * np.pi, 0.0, 0.0

        angle, curvature, curvature_s = self._turn_profile(
            arc - self._half_length - self.straight_length
        )
        return 1.5 * np.pi + angle, curvature, curvature_s

    def sample(self, t):
        """返回平滑椭圆跑道的位置导数和交替 knife-edge yaw 参考。"""
        t = float(t)
        if t < 0.0:
            raise ValueError("轨迹采样时间不能为负")

        distance_total = self.speed * t
        arc = float(np.mod(distance_total, self.path_length))
        x_local = np.interp(arc, self._arc_table, self._position_table[:, 0])
        y_local = np.interp(arc, self._arc_table, self._position_table[:, 1])
        heading, curvature, curvature_s = self._geometry_at_arc(arc)
        tangent = np.array([np.cos(heading), np.sin(heading)])
        normal = np.array([-np.sin(heading), np.cos(heading)])

        velocity_xy = self.speed * tangent
        acceleration_xy = self.speed**2 * curvature * normal
        jerk_xy = self.speed**3 * (
            curvature_s * normal - curvature**2 * tangent
        )

        x_ref = self.center + np.array([x_local, y_local, 0.0])
        v_ref = np.array([velocity_xy[0], velocity_xy[1], 0.0])
        a_ref = np.array([acceleration_xy[0], acceleration_xy[1], 0.0])
        j_ref = np.array([jerk_xy[0], jerk_xy[1], 0.0])

        # 每条直线上的 yaw 固定；每个转弯仅平滑旋转 pi/2。这样顶部直线
        # 的翼尖沿速度（knife edge），底部直线 coordinated，且后者每圈
        # 在正飞和倒飞之间交替，复现论文 Sec. VI-C 的 yaw 设计。
        half_index = int(np.floor(distance_total / self._half_length))
        local_half = distance_total - half_index * self._half_length
        psi_ref = half_index * 0.5 * np.pi
        psi_dot_ref = 0.0
        if local_half > self.straight_length:
            tau = (local_half - self.straight_length) / self.turn_length
            yaw_progress, yaw_progress_1, _ = _quintic_smoothstep(tau)
            psi_ref += 0.5 * np.pi * yaw_progress
            psi_dot_ref = (
                0.5
                * np.pi
                * yaw_progress_1
                * self.speed
                / self.turn_length
            )

        return x_ref, v_ref, a_ref, j_ref, float(psi_ref), float(psi_dot_ref)


class CircularTrajectory:
    """论文 Sec. VI-D 的 3.5 m 圆轨迹，支持 coordinated 和 knife edge。"""

    def __init__(
        self,
        mode="coordinated",
        radius=3.5,
        speed=8.1,
        laps=4,
        center=(0.0, 0.0, -10.0),
        clockwise=True,
    ):
        if mode not in {"coordinated", "knife_edge"}:
            raise ValueError("mode 必须是 coordinated 或 knife_edge")
        if radius <= 0.0 or speed <= 0.0:
            raise ValueError("半径和速度必须大于 0")
        if int(laps) != laps or laps <= 0:
            raise ValueError("圈数必须为正整数")
        center_array = np.asarray(center, dtype=float)
        if center_array.shape != (3,):
            raise ValueError("center 必须是包含三个 NED 坐标的向量")

        self.mode = mode
        self.radius = float(radius)
        self.speed = float(speed)
        self.laps = int(laps)
        self.center = center_array.copy()
        self.direction = -1.0 if clockwise else 1.0
        self.angular_rate = self.direction * self.speed / self.radius
        self.lap_time = 2.0 * np.pi * self.radius / self.speed
        self.total_duration = self.laps * self.lap_time
        self.path_length = 2.0 * np.pi * self.radius
        self.initial_state_mode = "reference"

    def sample(self, t):
        t = float(t)
        if t < 0.0:
            raise ValueError("轨迹采样时间不能为负")

        angle = self.angular_rate * t
        radial = np.array([np.cos(angle), np.sin(angle)])
        tangent = np.array([-np.sin(angle), np.cos(angle)])
        angle_dot = self.angular_rate

        position_xy = self.radius * radial
        velocity_xy = self.radius * angle_dot * tangent
        acceleration_xy = -self.radius * angle_dot**2 * radial
        jerk_xy = -self.radius * angle_dot**3 * tangent

        x_ref = self.center + np.array([position_xy[0], position_xy[1], 0.0])
        v_ref = np.array([velocity_xy[0], velocity_xy[1], 0.0])
        a_ref = np.array([acceleration_xy[0], acceleration_xy[1], 0.0])
        j_ref = np.array([jerk_xy[0], jerk_xy[1], 0.0])

        heading = float(np.arctan2(v_ref[1], v_ref[0]))
        psi_ref = heading if self.mode == "coordinated" else heading - 0.5 * np.pi
        psi_dot_ref = angle_dot
        return x_ref, v_ref, a_ref, j_ref, psi_ref, float(psi_dot_ref)


class CircularTransitionTrajectory:
    """论文 Sec. VI-E 在 3.5 m 圆上的 hover/forward-flight 速度过渡。"""

    def __init__(
        self,
        transition="to_forward",
        radius=3.5,
        tangential_acceleration=2.7,
        duration=3.0,
        lead_duration=0.4,
        settle_duration=0.4,
        center=(0.0, 0.0, -10.0),
        clockwise=True,
    ):
        if transition not in {"to_forward", "to_hover"}:
            raise ValueError("transition 必须是 to_forward 或 to_hover")
        if radius <= 0.0 or tangential_acceleration <= 0.0 or duration <= 0.0:
            raise ValueError("半径、切向加速度和时长必须大于 0")
        if lead_duration < 0.0 or settle_duration < 0.0:
            raise ValueError("过渡前置和收尾时长不能为负")
        center_array = np.asarray(center, dtype=float)
        if center_array.shape != (3,):
            raise ValueError("center 必须是包含三个 NED 坐标的向量")

        self.transition = transition
        self.radius = float(radius)
        self.tangential_acceleration = float(tangential_acceleration)
        self.duration = float(duration)
        self.lead_duration = float(lead_duration)
        self.settle_duration = float(settle_duration)
        self.target_speed = self.tangential_acceleration * self.duration
        self.distance = 0.5 * self.tangential_acceleration * self.duration**2
        self.center = center_array.copy()
        self.direction = -1.0 if clockwise else 1.0
        self.transition_end = self.lead_duration + self.duration
        self.total_duration = self.transition_end + self.settle_duration
        self.initial_state_mode = (
            "hover" if transition == "to_forward" else "reference"
        )

    def sample(self, t):
        t = float(t)
        if t < 0.0:
            raise ValueError("轨迹采样时间不能为负")
        transition_time = np.clip(t - self.lead_duration, 0.0, self.duration)
        transition_started = t >= self.lead_duration
        transition_finished = t >= self.transition_end

        if self.transition == "to_forward":
            distance = 0.5 * self.tangential_acceleration * transition_time**2
            speed = self.tangential_acceleration * transition_time
            speed_dot = (
                self.tangential_acceleration
                if transition_started and not transition_finished
                else 0.0
            )
            if transition_finished:
                distance += self.target_speed * (t - self.transition_end)
                speed = self.target_speed
        else:
            # 减速开始前沿圆以目标速度飞行；这样 Fig. 10 具有论文中的
            # 前置定常速度平台，同时保持位置、速度在过渡起点连续。
            distance = (
                self.target_speed * self.lead_duration
                + self.target_speed * transition_time
                - 0.5 * self.tangential_acceleration * transition_time**2
            )
            speed = self.target_speed - self.tangential_acceleration * transition_time
            speed_dot = (
                -self.tangential_acceleration
                if transition_started and not transition_finished
                else 0.0
            )
            if not transition_started:
                distance = self.target_speed * t
                speed = self.target_speed
            if transition_finished:
                speed = 0.0

        angle = self.direction * distance / self.radius
        angle_dot = self.direction * speed / self.radius
        angle_ddot = self.direction * speed_dot / self.radius
        radial = np.array([np.cos(angle), np.sin(angle)])
        tangent = np.array([-np.sin(angle), np.cos(angle)])

        position_xy = self.radius * radial
        velocity_xy = self.radius * angle_dot * tangent
        acceleration_xy = self.radius * (
            angle_ddot * tangent - angle_dot**2 * radial
        )
        jerk_xy = self.radius * (
            -3.0 * angle_dot * angle_ddot * radial
            - angle_dot**3 * tangent
        )

        x_ref = self.center + np.array([position_xy[0], position_xy[1], 0.0])
        v_ref = np.array([velocity_xy[0], velocity_xy[1], 0.0])
        a_ref = np.array([acceleration_xy[0], acceleration_xy[1], 0.0])
        j_ref = np.array([jerk_xy[0], jerk_xy[1], 0.0])

        # 即使端点速度为零，圆切线仍唯一确定即将进入或刚退出的 coordinated yaw。
        travel_tangent = self.direction * tangent
        psi_ref = float(np.arctan2(travel_tangent[1], travel_tangent[0]))
        psi_dot_ref = float(angle_dot)
        return x_ref, v_ref, a_ref, j_ref, psi_ref, psi_dot_ref


class DifferentialThrustTurnTrajectory:
    """论文 Sec. VI-F 的直线反向和快速 pi-yaw differential-thrust turn。

    论文只明确给出进入速度 7 m/s、约 0.5 s 完成朝向反转、最大角速度
    650 deg/s 及最大比力 2.2g。位置速度用 1.6 s 七次 smoothstep 反向，
    其峰值比力为约 2.20g；yaw 用 0.52 s 五次 smoothstep 旋转 pi，峰值
    yaw rate 为约 649 deg/s。二者均保持参考位置在同一条直线上。
    """

    def __init__(
        self,
        speed=7.0,
        entry_duration=0.9,
        reversal_duration=1.6,
        exit_duration=0.8,
        yaw_duration=0.52,
        center=(0.0, 0.0, -10.0),
    ):
        durations = (entry_duration, reversal_duration, exit_duration, yaw_duration)
        if speed <= 0.0 or any(duration <= 0.0 for duration in durations):
            raise ValueError("速度和各段时长必须大于 0")
        if yaw_duration > reversal_duration:
            raise ValueError("yaw 翻转时长不能大于位置反向时长")
        center_array = np.asarray(center, dtype=float)
        if center_array.shape != (3,):
            raise ValueError("center 必须是包含三个 NED 坐标的向量")

        self.speed = float(speed)
        self.entry_duration = float(entry_duration)
        self.reversal_duration = float(reversal_duration)
        self.exit_duration = float(exit_duration)
        self.yaw_duration = float(yaw_duration)
        self.center = center_array.copy()
        self.total_duration = (
            self.entry_duration + self.reversal_duration + self.exit_duration
        )
        self.initial_state_mode = "reference"

    def sample(self, t):
        t = float(t)
        if t < 0.0:
            raise ValueError("轨迹采样时间不能为负")

        reversal_end = self.entry_duration + self.reversal_duration
        if t < self.entry_duration:
            position_y = self.speed * (t - self.entry_duration)
            velocity_y = self.speed
            acceleration_y = 0.0
            jerk_y = 0.0
        elif t <= reversal_end:
            tau = (t - self.entry_duration) / self.reversal_duration
            progress, progress_1, progress_2 = _septic_smoothstep(tau)
            progress_integral = _septic_smoothstep_integral(tau)
            position_y = self.speed * self.reversal_duration * (
                tau - 2.0 * progress_integral
            )
            velocity_y = self.speed * (1.0 - 2.0 * progress)
            acceleration_y = (
                -2.0 * self.speed * progress_1 / self.reversal_duration
            )
            jerk_y = (
                -2.0 * self.speed * progress_2 / self.reversal_duration**2
            )
        else:
            time_after = t - reversal_end
            position_y = -self.speed * time_after
            velocity_y = -self.speed
            acceleration_y = 0.0
            jerk_y = 0.0

        x_ref = self.center + np.array([0.0, position_y, 0.0])
        v_ref = np.array([0.0, velocity_y, 0.0])
        a_ref = np.array([0.0, acceleration_y, 0.0])
        j_ref = np.array([0.0, jerk_y, 0.0])

        yaw_time = t - self.entry_duration
        if yaw_time <= 0.0:
            yaw_progress = 0.0
            yaw_progress_1 = 0.0
        elif yaw_time >= self.yaw_duration:
            yaw_progress = 1.0
            yaw_progress_1 = 0.0
        else:
            yaw_progress, yaw_progress_1, _ = _quintic_smoothstep(
                yaw_time / self.yaw_duration
            )

        psi_ref = 0.5 * np.pi + np.pi * yaw_progress
        psi_dot_ref = np.pi * yaw_progress_1 / self.yaw_duration
        return x_ref, v_ref, a_ref, j_ref, float(psi_ref), float(psi_dot_ref)
