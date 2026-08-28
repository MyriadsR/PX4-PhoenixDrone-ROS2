# phoenix_tailsitter_control 代码说明

本文档说明 `ros2_ws/src/phoenix_tailsitter_control/phoenix_tailsitter_control` 目录下各个 Python 文件的作用，以及这些文件如何共同组成 Phoenix tailsitter 的 ROS 2 控制逻辑。

## 1. 总体结构

这个包实现的是一个面向 PX4 SITL/Gazebo 的 tailsitter 控制器。它不走 PX4 内部的姿态/位置控制器，而是使用 PX4 的 direct actuator offboard 接口，直接向 PX4 发送电机和舵机归一化控制量。

核心数据流是：

```text
PX4 状态输出
  /fmu/out/vehicle_attitude
  /fmu/out/vehicle_angular_velocity
  /fmu/out/vehicle_control_mode
  /fmu/out/vehicle_local_position
        |
        v
ROS 2 控制器
  controller_node.py 或 alpha_controller_node.py
        |
        v
PX4 direct actuator 输入
  /fmu/in/offboard_control_mode
  /fmu/in/actuator_motors
  /fmu/in/actuator_servos
  /fmu/in/vehicle_thrust_setpoint
        |
        v
PX4 Gazebo bridge
        |
        v
Gazebo 电机/舵面模型
```

包内有两条主要控制路线：

- `controller_node.py`：基础低速悬停控制器，使用简化的 PhoenixAero 悬停模型。
- `alpha_controller_node.py`：完整 alpha 理论控制器，使用 `Tailsitter-control` 中的 alpha 气动模型、微分平坦变换和速度相关控制分配。

两者都最终输出：

- `ActuatorMotors.control[0:2]`：两个电机归一化命令。
- `ActuatorServos.control[0:2]`：两个舵机归一化命令。

## 2. 坐标系约定

控制代码内部大量使用 Tailsitter-control 坐标系，简称 TS。PX4 使用 FRD 机体系。两者关系由 `frames.py` 统一定义：

```text
x_ts = -z_px4
y_ts =  y_px4
z_ts =  x_px4
```

也就是说：

```text
v_px4 = [v_ts.z, v_ts.y, -v_ts.x]
v_ts  = [-v_px4.z, v_px4.y, v_px4.x]
```

代码里要求所有 PX4/TS 坐标转换都通过 `frames.py` 完成，避免在不同文件里散落手写符号变换。

## 3. 文件逐项说明

### 3.1 `__init__.py`

包初始化文件，只包含一句 docstring：

```python
"""PhoenixDrone-specific port of the Tailsitter-control attitude inner loop."""
```

它的作用是让 `phoenix_tailsitter_control` 成为 Python package。没有运行逻辑。

### 3.2 `config.py`

`config.py` 定义全局控制参数类 `PhoenixHoverConfig`。这是控制器参数的集中入口。

主要内容：

- 飞机质量和重力：
  - `mass = 0.525`
  - `gravity = 9.81`
- PX4 机体系惯量：
  - `inertia_px4`
  - 通过 `inertia_ts` 属性转换到 TS 坐标系
- 电机模型：
  - `motor_thrust_coefficient`
  - `motor_moment_ratio`
  - `motor_arm_y`
  - `motor_speed_max`
  - `motor_time_constant_up`
  - `motor_time_constant_down`
  - `rotor_velocity_slowdown`
- 舵面模型：
  - `flap_lift_coefficient`
  - `flap_drag_coefficient`
  - `flap_pitch_coefficient`
  - `left_flap_limit`
  - `right_flap_limit`
- alpha 气动模型参数：
  - `alpha_zero_lift_rad`
  - `alpha_thrust_installation_rad`
  - `alpha_c_lv`
  - `alpha_c_lt`
  - `alpha_c_lv_delta`
  - `alpha_c_lt_delta`
  - `alpha_motor_torque_coefficient`
  - `alpha_motor_arm_y`
  - `alpha_flap_arm_y`
  - `alpha_flap_arm_x`
- 控制频率和滤波参数：
  - `control_rate_hz = 250`
  - `indi_lpf_cutoff_hz`
  - `linear_indi_lpf_cutoff_hz`
  - `flap_hpf_cutoff_hz`
- 姿态环增益：
  - `attitude_gain`
  - `rate_gain`
  - `angular_acceleration_limit`
  - `moment_limit`
- 位置环增益和限制：
  - `position_gain`
  - `velocity_gain`
  - `horizontal_acceleration_limit`
  - `vertical_acceleration_limit`
  - `position_tilt_limit_rad`
  - `horizontal_force_slew_rate_n_s`

关键属性：

- `inertia_ts`：把 PX4 FRD 惯量矩阵转换到 TS 坐标系。
- `maximum_total_thrust`：两个电机最大总推力。
- `hover_total_thrust`：悬停所需总推力，即 `mass * gravity`。
- `minimum_position_thrust` / `maximum_position_thrust`：位置环允许的推力上下限。
- `alpha_identification_defaults`：集中列出应当辨识或验证的 alpha 模型参数。

需要注意：这里的 alpha 参数和 Gazebo `model.sdf` 中 `TailsitterAlphaAero` 插件的参数是手工保持一致的，不是自动共享。

### 3.3 `frames.py`

`frames.py` 是坐标系转换工具文件。它定义 TS 与 PX4 body 坐标的唯一转换矩阵。

主要常量：

- `C_PX4_TS`：TS 向量转 PX4 向量。
- `C_TS_PX4`：PX4 向量转 TS 向量。

主要函数：

- `vector_ts_to_px4(vector)`：TS 机体系向量转 PX4 机体系向量。
- `vector_px4_to_ts(vector)`：PX4 机体系向量转 TS 机体系向量。
- `inertia_px4_to_ts(inertia_px4)`：惯量矩阵从 PX4 坐标转到 TS 坐标。
- `attitude_px4_to_ts(q_px4)`：把 PX4 的 body-to-NED 姿态四元数转换为 TS body-to-NED 姿态。
- `attitude_ts_to_px4(q_ts)`：反向转换。

控制节点从 PX4 收到姿态和角速度后，会先用这里的函数转换到 TS 坐标系，然后再进入姿态 INDI 和分配器。

### 3.4 `math_utils.py`

`math_utils.py` 提供四元数和旋转矩阵工具，统一使用 Hamilton 四元数顺序 `[w, x, y, z]`。

主要函数：

- `normalize_quaternion(q)`：检查四元数有限、非零，并归一化。
- `quaternion_inverse(q)`：四元数逆。
- `quaternion_multiply(q1, q2)`：Hamilton 四元数乘法。
- `quaternion_to_matrix(q)`：四元数转旋转矩阵。
- `matrix_to_quaternion(matrix)`：旋转矩阵转四元数，并保证实部非负。
- `rotation_vector_error(q_current, q_desired)`：计算从当前姿态到期望姿态的最短 body-frame 旋转误差向量。

`rotation_vector_error()` 是姿态控制器的核心误差输入。它输出的是三维小角度/旋转向量误差，而不是欧拉角误差。

### 3.5 `filters.py`

`filters.py` 实现无外部依赖的二阶 Butterworth 滤波器。

主要类：

- `ButterworthLowPass`
  - 二阶低通滤波器。
  - 用于角速度、估计力矩、线加速度、电机转速、舵面角等信号滤波。
  - 初始化时用稳态初值，避免刚启动时产生滤波器爬升瞬态。

- `ButterworthHighPass`
  - 二阶高通滤波器。
  - alpha 控制器中用于分离舵面瞬态量。
  - 主要帮助估计并剔除 flap transient force 对线加速度反馈的影响。

两个滤波器都要求：

- 截止频率大于 0。
- 采样频率大于 0。
- 截止频率低于 Nyquist 频率。
- 输入通道数量和数据形状固定。

### 3.6 `attitude_control.py`

`attitude_control.py` 实现姿态 PD 加角加速度 INDI 内环。

核心类：`AttitudeINDIController`

初始化时从 `PhoenixHoverConfig` 读取：

- `attitude_gain`
- `rate_gain`
- `angular_acceleration_limit`
- `moment_limit`
- `inertia_ts`

主要函数：

- `angular_acceleration_command(q_current_ts, q_desired_ts, omega_lpf_ts, omega_reference_ts=None)`

  根据姿态误差和角速度误差计算期望角加速度：

  ```text
  angular_acc_cmd =
      attitude_gain * attitude_error
    + rate_gain * (omega_reference - omega_lpf)
  ```

  然后用 `angular_acceleration_limit` 限幅。

- `moment_command(acceleration_command_ts, acceleration_lpf_ts, estimated_moment_lpf_ts)`

  INDI 思路是：在当前估计力矩基础上，补偿期望角加速度和实测角加速度之间的差：

  ```text
  desired_moment =
      inertia_ts * (acc_cmd - acc_lpf)
    + estimated_moment_lpf
  ```

  最后用 `moment_limit` 限幅。

这个文件只负责把姿态/角速度误差变成期望力矩，不负责把力矩分配到电机和舵面。

### 3.7 `allocator.py`

`allocator.py` 是基础悬停模型的执行器分配器。

主要数据类：

- `AllocationResult`
  - `motor_speed_left_right`
  - `flap_angle_left_right`
  - `achieved_moment_ts`

核心类：`PhoenixHoverAllocator`

内部执行器顺序固定为：

```text
[left, right]
```

但是输出到 PX4 时要按 PX4/Gazebo 通道映射转换。文件顶部注释说明：

- 电机 direct actuator 通道为 `[left, right]`。
- 舵机 direct actuator 通道为 `[right, left]`。

主要函数：

- `_bounded_two_by_two(matrix, target, lower, upper)`

  解决带上下限的二维线性分配问题。它先求无约束解，如果超限，再检查四条边界上的最优解，选残差最小者。

- `_flap_matrix(motor_speed_left_right)`

  根据当前左右电机转速估算左右舵面对 TS x/y 力矩的控制效率矩阵。

- `estimate_moment(motor_speed_left_right, flap_angle_left_right)`

  根据简化悬停模型估计当前电机和舵面产生的 TS 力矩。包括：

  - 左右电机差动推力产生 z 轴力矩。
  - 电机反扭矩产生 x 轴力矩。
  - 舵面升力产生 x/y 力矩。

- `allocate(total_thrust, desired_moment_ts)`

  把总推力和期望三轴力矩分配成：

  - 左右电机转速。
  - 左右舵面角。

  z 轴力矩主要由左右电机差动实现，x/y 力矩由舵面实现。

- `to_px4_controls(allocation)`

  把物理量转换成 PX4 direct actuator 归一化命令：

  - 电机：`omega / motor_speed_max`，范围 `[0, 1]`。
  - 舵面：按 PX4/Gazebo 通道和符号转换，范围 `[-1, 1]`。

### 3.8 `alpha_aerodynamics.py`

`alpha_aerodynamics.py` 实现完整 alpha 理论气动力/力矩模型。它和 Gazebo 插件 `TailsitterAlphaAero.cpp` 的计算意图一致，用于让 ROS 控制器内部的模型和 Gazebo 被控对象尽量一致。

主要数据类：

- `AlphaAerodynamicResult`
  - `force_alpha`：alpha 坐标系下的总力。
  - `moment_body_ts`：TS 机体系下的总力矩。
  - `flap_force_alpha`：alpha 坐标系下舵面贡献的力。

核心类：`AlphaAerodynamics`

初始化时建立：

- `alpha_zero`
- `alpha_thrust`
- `alpha_tilde = alpha_zero + alpha_thrust`
- `rotation_body_to_alpha`
- `rotation_alpha_to_body`

主要函数：

- `compute(velocity_body_ts, motor_speed_left_right, flap_angle_left_right)`

  输入：

  - TS 机体系速度。
  - 左右电机实际转速。
  - 左右舵面角。

  输出：

  - alpha 坐标系总力。
  - TS 机体系总力矩。
  - alpha 坐标系舵面力。

  计算内容包括：

  - 电机推力 `T = c_T * omega^2`。
  - alpha 坐标下推力方向修正。
  - 舵面力。
  - 机翼速度相关升阻力。
  - 左右推力差产生的力矩。
  - 电机反扭矩。
  - 舵面力臂产生的力矩。

- `force_body_ts(result)`

  把 `force_alpha` 转回 TS 机体系。

模块底部还有两个 Gazebo FLU 和 TS 之间的向量转换工具：

- `vector_gz_flu_to_ts(vector_gz)`
- `vector_ts_to_gz_flu(vector_ts)`

### 3.9 `alpha_allocator.py`

`alpha_allocator.py` 是完整 alpha 理论的控制分配器，对应 Tailsitter-control 中速度相关分配公式。

核心类：`AlphaTheoryAllocator`

初始化时：

- 保存 `PhoenixHoverConfig`。
- 创建 `AlphaAerodynamics`。
- 计算 `alpha_tilde`。

主要函数：

- `allocate(total_thrust, desired_moment_ts, velocity_body_ts)`

  输入：

  - 期望总推力。
  - 期望 TS 三轴力矩。
  - 当前 TS 机体系速度。

  分配逻辑：

  1. 根据 z 轴期望力矩求左右电机差动推力。
  2. 根据总推力和差动推力求左右电机推力。
  3. 推力限幅到单电机最大能力范围。
  4. 根据电机推力和速度相关项建立舵面控制效率矩阵。
  5. 用 `PhoenixHoverAllocator._bounded_two_by_two()` 求左右舵面角。
  6. 用 `AlphaAerodynamics.compute()` 估计最终实际可达力矩。

- `to_px4_controls(allocation)`

  和基础分配器类似，把 `[left, right]` 物理执行器量转换为 PX4 direct actuator 通道：

  ```text
  motors = [omega_left, omega_right] / motor_speed_max
  servos = [-delta_right / right_limit,
            -delta_left  / left_limit]
  ```

这个文件是 `alpha_controller_node.py` 区别于基础控制器的重要部分。

### 3.10 `position_control.py`

`position_control.py` 实现低速位置环、线加速度 INDI、推力限幅和由力向量生成 tailsitter 姿态的工具函数。

主要数据类：

- `TrajectoryReference`
  - `position`
  - `velocity`
  - `acceleration`
  - `jerk`
  - `yaw`
  - `yawspeed`
  - `position_mask`
  - `velocity_mask`

主要函数和类：

- `resolve_trajectory_reference(message, position, velocity, fallback_yaw)`

  解析 PX4 `TrajectorySetpoint` 的 NaN 语义。

  PX4 setpoint 中某些字段可以为 NaN，表示这个轴没有显式控制。该函数会把 NaN 转换成控制器可用的有限参考量：

  - 有 position 时，未给 velocity 就认为该轴期望速度为 0。
  - 没有 position 但有 velocity 时，按速度控制。
  - acceleration NaN 时使用 0。
  - yaw NaN 时使用 fallback yaw。
  - yawspeed NaN 时使用 0。

- `PositionController.acceleration_command(...)`

  位置环。它在 NED 世界系中接收位置、速度、加速度状态，但把误差旋转到 TS body 坐标中乘增益，再旋回 NED：

  ```text
  feedback_ts =
      position_gain * position_error_ts
    + velocity_gain * velocity_error_ts
    + linear_acceleration_gain * acceleration_error_ts

  acceleration_command_ned =
      reference.acceleration + R_ts_to_ned * feedback_ts
  ```

  最后对水平和垂直加速度分别限幅。

- `LinearAccelerationINDIController`

  基础控制器使用的线加速度 INDI。它根据简化 PhoenixAero 悬停模型估计当前执行器产生的力，然后计算增量力命令。

  重要函数：

  - `estimate_force_components_ts()`
  - `estimate_force_ts()`
  - `force_command()`

- `HoverForceSlewLimiter`

  对水平力命令做变化率限制，避免位置环要求的水平力变化超过姿态环能跟上的速度。

- `tailsitter_heading_from_attitude(q_ts)`

  根据 TS body z 轴在 NED 水平面的投影提取当前航向。

- `limit_hover_force(force_ned, config)`

  对低速悬停位置控制的力命令做限制：

  - 垂直推力限制在 `[minimum_position_thrust, maximum_position_thrust]`。
  - 水平力受最大倾角 `position_tilt_limit_rad` 限制。
  - 总推力不超过最大位置推力。

- `force_to_tailsitter_attitude(force_ned, yaw)`

  把期望力向量转换成 tailsitter 期望姿态：

  - TS +x 轴对齐期望力方向。
  - TS +z 轴尽量满足给定 yaw heading。

- `degraded_vertical_force(...)`

  当水平 EKF 状态无效时，构造只控制垂直方向的保守力命令，避免因为水平位置无效而直接零推力掉落。

### 3.11 `flatness_control.py`

`flatness_control.py` 实现 Tailsitter-control 的微分平坦变换，用于 alpha 控制器的位置轨迹跟踪。

主要函数：

- `zxy_euler_to_quaternion(yaw, roll, pitch)`

  按 Z-X-Y 欧拉角顺序生成 Hamilton 四元数。

核心类：`FlatnessAttitudeController`

主要函数：

- `attitude_and_thrust(force_ned, velocity_ned, flap_sum, yaw, q_current_ts, return_angles=False)`

  根据期望力、速度、舵面总偏角、yaw 和当前姿态，求：

  - 期望姿态 `q_desired`
  - 期望总推力 `thrust`
  - 可选返回中间角 `roll` 和 `pitch_bar`

  这是 alpha 控制器中从位置环力命令到姿态/推力命令的关键步骤。它考虑了：

  - alpha 气动力参数。
  - 速度相关升阻力。
  - 舵面总偏角对平坦变换的影响。
  - 推力方向和 zero-lift angle 的关系。

- `feedforward_rates(...)`

  根据参考速度、加速度、jerk、yaw/yawspeed 和参考力，计算期望 body rates。这个前馈角速度用于减小轨迹跟踪中的姿态滞后。

基础 `controller_node.py` 不使用这个完整平坦变换；`alpha_controller_node.py` 使用它。

### 3.12 `debug.py`

`debug.py` 定义调试 topic 的字段顺序和打包函数。所有调试消息都用 `std_msgs/Float64MultiArray` 发布，因此必须有稳定字段顺序方便后处理。

主要字段列表：

- `CONTROL_DEBUG_FIELDS`
  - 姿态误差、滤波角速度、角加速度、期望力矩、分配力矩、电机命令、舵面角、总推力、输出是否激活。

- `POSITION_DEBUG_FIELDS`
  - 当前/参考位置、当前/参考速度、滤波加速度、参考加速度、加速度命令、估计力、命令力、限幅后力、yaw、总推力、轨迹是否激活。

- `ACTUATOR_FEEDBACK_DEBUG_FIELDS`
  - 预测执行器状态、Gazebo 反馈执行器状态、误差、反馈年龄、反馈是否完整/有效。

- `ALPHA_MODEL_DEBUG_FIELDS`
  - TS 机体系速度、alpha 模型力、TS 力矩、舵面瞬态力、电机/舵面低通和高通状态。

主要函数：

- `pack_control_debug(...)`
- `pack_position_debug(...)`
- `pack_actuator_feedback_debug(...)`
- `pack_alpha_model_debug(...)`

这些函数会检查打包后的向量长度和有限性。如果数量不对或包含 NaN/Inf，会抛出错误。

### 3.13 `qos.py`

`qos.py` 定义 PX4 uORB ROS 2 bridge 使用的 QoS。

主要常量：

- `PX4_INPUT_QOS`

  用于发布到 `/fmu/in/...`：

  - reliable
  - volatile
  - keep last
  - depth 1

- `PX4_OUTPUT_QOS`

  用于订阅 `/fmu/out/...`：

  - best effort
  - volatile
  - keep last
  - depth 1

PX4 bridge 的 QoS 要匹配，否则 ROS 2 节点可能看不到 PX4 topic 或无法让 PX4 收到命令。

### 3.14 `actuator_feedback.py`

`actuator_feedback.py` 解析 Gazebo joint state，得到控制器内部使用的实际执行器状态。

主要数据类：

- `ActuatorJointFeedback`
  - `motor_speed_left_right`
  - `flap_angle_left_right`

主要函数：

- `parse_actuator_joint_state(message, rotor_velocity_slowdown)`

  输入 `sensor_msgs/JointState`，输出 `[left, right]` 顺序的：

  - 电机物理转速。
  - 舵面实际角度。

关键处理：

- 检查 joint 名字唯一。
- 检查 `position` 和 `velocity` 长度与名字数组一致。
- 必须包含：
  - `rotor_left_joint`
  - `rotor_right_joint`
  - `left_elevon_joint`
  - `right_elevon_joint`
- 电机 Gazebo joint velocity 乘以 `rotor_velocity_slowdown` 才是物理桨速。
- 由于 Gazebo FLU 到 PX4/TS 坐标变换，`rotor_right_joint` 在控制器内部对应 TS left，`rotor_left_joint` 对应 TS right。

控制节点可通过参数 `actuator_feedback_mode` 决定使用预测执行器状态还是 Gazebo joint state 反馈。

### 3.15 `controller_node.py`

`controller_node.py` 是基础控制器节点，console script 名称是：

```text
tailsitter_controller
```

节点名：

```text
phoenix_tailsitter_controller
```

它实现一个低速悬停近似控制器，包含：

- ROS 2/PX4 topic 接入。
- Offboard direct actuator 模式保持。
- 姿态 setpoint 或轨迹 setpoint 解析。
- 位置环。
- 姿态 INDI。
- 基础执行器分配。
- 安全 gating。
- 执行器反馈/预测。
- 调试 topic 发布。

#### 订阅 topic

- `/fmu/out/vehicle_attitude`
  - PX4 当前姿态。
- `/fmu/out/vehicle_angular_velocity`
  - PX4 当前角速度。
- `/fmu/out/vehicle_control_mode`
  - PX4 当前控制模式、是否 armed、是否 offboard。
- `/fmu/out/vehicle_local_position`
  - PX4 本地位置、速度、加速度。
- `/phoenix_tailsitter/attitude_setpoint`
  - 自定义姿态 setpoint。
- `/fmu/in/trajectory_setpoint`
  - 轨迹 setpoint。这里复用 PX4 input topic 作为 ROS 控制器的轨迹输入。
- `actuator_feedback_topic`
  - 默认 `/world/stars_ts/model/phoenixdrone_0/joint_state`，用于读取 Gazebo joint state。

#### 发布 topic

- `/fmu/in/offboard_control_mode`
  - 设置 `direct_actuator = True`。
- `/fmu/in/actuator_motors`
  - 直接电机输出。
- `/fmu/in/actuator_servos`
  - 直接舵机输出。
- `/fmu/in/vehicle_thrust_setpoint`
  - 发布总推力意图，帮助 PX4 land detector 判断飞机状态。
- `/phoenix_tailsitter/control_debug`
- `/phoenix_tailsitter/position_debug`
- `/phoenix_tailsitter/actuator_feedback_debug`

#### 参数

- `output_enabled`
  - 是否真正输出非零执行器命令。
  - 即使为 true，也还要满足 armed、offboard、状态新鲜等安全条件。
- `setpoint_source`
  - `latch`：锁存当前姿态作为悬停姿态。
  - `topic`：使用 `/phoenix_tailsitter/attitude_setpoint`。
  - `trajectory`：使用 `/fmu/in/trajectory_setpoint` 做位置控制。
- `setpoint_frame`
  - `px4` 或 `tailsitter`，用于解释 topic 姿态 setpoint。
- `hover_thrust_scale`
  - latch/topic 模式下的悬停推力缩放。
- `actuator_feedback_mode`
  - `auto`：有新鲜 joint feedback 就用反馈，否则用内部预测。
  - `estimate`：只用内部预测。
  - `required`：必须有新鲜反馈，否则禁止输出。
- `actuator_feedback_topic`
  - joint state topic 名称。

#### 控制周期 `_control_tick()`

控制周期频率为 `cfg.control_rate_hz`，默认 250 Hz。每个周期做：

1. 发布 `OffboardControlMode(direct_actuator=True)`。
2. 更新执行器状态估计。
3. 发布执行器反馈调试信息。
4. 检查姿态和角速度是否新鲜。
5. 把 PX4 姿态和角速度转换到 TS 坐标。
6. 检查 PX4 是否 armed 且处于 offboard。
7. 根据 `setpoint_source` 生成：
   - `desired_q_ts`
   - `total_thrust`
   - `desired_rates_ts`
8. 对角速度和估计力矩低通滤波。
9. 用 `AttitudeINDIController` 计算期望角加速度和期望力矩。
10. 用 `PhoenixHoverAllocator` 分配左右电机和左右舵面。
11. 发布 debug。
12. 若所有安全条件满足且 `output_enabled=true`，发布真实 actuator 命令；否则发布零输出。

#### 轨迹模式 `_trajectory_desired_state()`

基础控制器的轨迹模式会：

1. 检查 local position 和 trajectory setpoint 是否新鲜。
2. 解析 `TrajectorySetpoint` 的 NaN 语义。
3. 低通滤波 PX4 local acceleration。
4. 使用 `PositionController` 计算期望线加速度。
5. 使用 `LinearAccelerationINDIController` 计算期望净力。
6. 减去估计的舵面气动力，得到推进力命令。
7. 用 `limit_hover_force()` 限制低速悬停推力和倾角。
8. 用 `HoverForceSlewLimiter` 限制水平力变化率。
9. 用 `force_to_tailsitter_attitude()` 把力向量转换成姿态和总推力。

这个基础控制器更适合低速悬停/小步长测试，不是完整高速过渡飞行控制。

### 3.16 `alpha_controller_node.py`

`alpha_controller_node.py` 是完整 alpha 理论控制器节点，console script 名称是：

```text
alpha_tailsitter_controller
```

它继承 `TailsitterController`，复用基础节点的：

- ROS topic 订阅/发布。
- PX4 offboard/direct actuator 接口。
- 状态新鲜度检查。
- 执行器反馈/预测框架。
- debug topic 框架。

然后替换/扩展核心控制模型：

- `self.aerodynamics = AlphaAerodynamics(self.cfg)`
- `self.allocator = AlphaTheoryAllocator(self.cfg)`
- `self.flatness_controller = FlatnessAttitudeController(self.cfg)`

新增调试 topic：

```text
/phoenix_tailsitter/alpha_model_debug
```

#### alpha 滤波状态

新增滤波器和状态：

- `alpha_motor_filter`
- `alpha_flap_filter`
- `alpha_flap_hpf_filter`
- `motor_speed_lpf`
- `flap_angle_lpf`
- `flap_angle_hpf`
- `flap_angle_without_transient`

其中：

- 电机转速和舵面角低通后用于估计稳态气动力。
- 舵面高通量用于估计 flap transient force。
- `flap_angle_without_transient = flap_angle_lpf - flap_angle_hpf`，用于位置环稳态气动力估计。

#### `_body_velocity()`

从 PX4 local position 消息中取 NED 速度：

```text
[vx, vy, vz]
```

然后用当前姿态旋转到 TS 机体系：

```text
velocity_body_ts = R_ts_to_ned.T @ velocity_ned
```

这是 alpha 气动模型必须使用的输入。

#### alpha 轨迹控制 `_trajectory_desired_state()`

相比基础控制器，alpha 版本的轨迹模式有几处关键区别：

1. 允许水平位置无效时降级到垂直控制：
   - 如果 z 和 vz 有效，但 xy 无效，就调用 `_degraded_vertical_desired_state()`。
2. 根据当前速度、电机低通转速、舵面状态计算 alpha 气动力。
3. 用舵面高通量估计瞬态 flap force，并从滤波加速度中扣除：

   ```text
   acceleration_without_flap_transient =
       acceleration_lpf - R_alpha_to_ned * flap_transient / mass
   ```

4. 位置环输出期望线加速度。
5. 用 alpha 稳态模型估计当前总气动力。
6. 构造期望力命令：

   ```text
   force_command =
       estimated_force_lpf_ned
     + mass * (acceleration_command - acceleration_without_flap_transient)
   ```

7. 用 `FlatnessAttitudeController.attitude_and_thrust()` 把期望力转换为姿态和总推力。
8. 用 `FlatnessAttitudeController.feedforward_rates()` 生成期望角速度前馈。

#### alpha 控制周期 `_control_tick()`

整体流程与基础控制器类似，但关键不同是：

- 先更新 alpha 电机/舵面滤波器。
- 使用 `AlphaAerodynamics.compute()` 估计当前 alpha 模型力和力矩。
- 姿态 INDI 中的 `estimated_moment_lpf` 来自 alpha 模型，而不是基础分配器的简化估计。
- 执行器分配使用 `AlphaTheoryAllocator.allocate(total_thrust, desired_moment, velocity_body_ts)`，分配结果随速度变化。
- 发布 `alpha_model_debug`。

#### 与基础控制器的关系

`alpha_controller_node.py` 不是独立重写所有 ROS 逻辑，而是继承基础控制器，并覆盖：

- 动态状态 reset 中的 alpha 滤波器。
- 轨迹期望状态生成。
- 控制周期。
- 分配器和气动模型。

因此它仍然使用同一套 PX4 direct actuator 交互机制，但控制模型更接近 Tailsitter-control 原算法。

### 3.17 `actuator_probe.py`

`actuator_probe.py` 是单通道执行器测试工具，console script 名称是：

```text
actuator_probe
```

它永远不会主动 arm 飞机，只在已经 armed + offboard 的情况下，短时间输出一个单通道命令，用来验证通道映射。

安全确认 token：

```text
PHOENIX_SINGLE_CHANNEL_TEST
```

主要参数：

- `enabled`
  - 必须为 true 才会输出。
- `confirmation`
  - 必须等于确认 token。
- `kind`
  - `motor` 或 `servo`。
- `index`
  - 0 或 1。
- `value`
  - 测试通道输出值。
- `motor_idle`
  - 测试舵机时可给电机一个很小 idle。
- `duration_s`
  - 输出持续时间，最大限制 10 秒。

运行逻辑：

1. 每 0.01 s 发布 `OffboardControlMode(direct_actuator=True)`。
2. 检查：
   - `enabled=true`
   - confirmation 正确
   - PX4 已 armed
   - PX4 offboard 已启用
   - control mode 消息新鲜
3. 条件满足后，只在 `duration_s` 时间内输出指定通道。
4. 超时或条件不满足时输出全零。

这个工具适合在真正跑闭环控制前检查：

- 电机 0/1 是否对应预期左右电机。
- servo 0/1 是否对应预期左右舵面。
- 舵面正负方向是否正确。

### 3.18 `attitude_step_test.py`

`attitude_step_test.py` 是姿态阶跃测试工具，console script 名称是：

```text
attitude_step_test
```

它发布一个有限幅度的 TS 轴向姿态阶跃，并根据 PX4 返回的姿态/角速度检查响应是否方向正确、是否有明显越界。

安全确认 token：

```text
PHOENIX_ATTITUDE_STEP_TEST
```

主要辅助函数：

- `axis_angle_quaternion(axis_index, angle)`

  生成绕 TS x/y/z 某轴的小角度四元数。

- `step_phase(elapsed_s, baseline_s, step_s, recovery_s)`

  生成测试阶段：

  ```text
  baseline_pre -> plus -> baseline_mid -> minus -> baseline_post -> done
  ```

- `summarize_axis_response(records, axis_index, amplitude)`

  汇总响应：

  - plus 阶段稳态响应是否为正。
  - minus 阶段稳态响应是否为负。
  - 主轴峰值是否过大。
  - 交叉轴响应是否过大。
  - 角速度峰值是否过大。

核心类：`AttitudeStepTest`

运行逻辑：

1. 等待 PX4 姿态消息。
2. 在未开始前持续跟随当前姿态，避免起始姿态漂移影响测试。
3. 等待 operator 或外部脚本让飞机进入 armed + offboard。
4. 记录初始 TS 姿态。
5. 按阶段发布姿态 setpoint 到 `/phoenix_tailsitter/attitude_setpoint`。
6. 收集实际姿态响应。
7. 输出 `ATTITUDE_STEP_RESULT` JSON。

它本身不负责 arm/offboard，只负责发布姿态 setpoint 和判断响应。

### 3.19 `position_step_test.py`

`position_step_test.py` 是位置小阶跃测试工具，console script 名称是：

```text
position_step_test
```

安全确认 token：

```text
PHOENIX_POSITION_STEP_TEST
```

主要辅助函数：

- `position_step_phase(elapsed_s, climb_s, baseline_s, step_s, recovery_s)`

  生成测试阶段：

  ```text
  climb -> baseline_pre -> plus -> baseline_mid -> minus -> baseline_post -> done
  ```

- `summarize_position_response(records, axis_index, amplitude)`

  检查位置阶跃响应：

  - plus 响应方向。
  - minus 响应方向。
  - 主轴位置跨度。
  - 交叉轴位置跨度。
  - 速度峰值。

- `summarize_takeoff_response(records, origin, hover_reference, takeoff_height)`

  检查起飞是否真正达到高度，并在 hover reference 附近稳定。

核心类：`PositionStepTest`

运行逻辑：

1. 等待有效 local position。
2. 在 armed/offboard 前持续发布当前位置作为 setpoint。
3. 一旦 armed/offboard，记录 origin。
4. 生成 hover reference：

   ```text
   hover_reference.z = origin.z - takeoff_height
   ```

   PX4 NED 中 z 向下为正，所以起飞是 z 减小。

5. 发布 climb、baseline、正阶跃、回中、负阶跃、回中 setpoint。
6. 收集位置和速度。
7. 如果水平位移或速度超过安全边界，立即失败。
8. 输出 `POSITION_STEP_RESULT` JSON。
9. 如果测试完成且飞机仍 armed/offboard，则继续保持 hover setpoint，直到操作者 disarm。

这个工具用于验证位置环小幅跟踪能力，不自动 arm/disarm。

### 3.20 `position_mission_test.py`

`position_mission_test.py` 是自动位置任务测试工具，console script 名称是：

```text
position_mission_test
```

它比 `position_step_test.py` 更自动化：会自动请求 offboard、arm、起飞、执行任务、降落、强制 disarm。

安全确认 token：

```text
PHOENIX_POSITION_MISSION_TEST
```

兼容旧 token：

```text
PHOENIX_CROSS_MISSION_TEST
```

主要数据类：

- `MissionPhase`
  - `name`
  - `offset_ned`
  - `dwell_s`

主要任务生成函数：

- `build_cross_mission(distance_m, takeoff_height_m, dwell_s)`

  生成十字形任务：

  ```text
  hover -> forward -> backward/home -> left -> right/home -> land
  ```

- `build_rectangle_mission(side_length_m, takeoff_height_m, dwell_s)`

  生成边长为 `side_length_m` 的矩形/方形任务：

  ```text
  hover
  -> forward
  -> forward_left
  -> left
  -> home
  -> land
  ```

  在当前模型约定中：

  - NED +x 是 forward。
  - NED -y 是 left。

- `build_mission(shape, distance_m, takeoff_height_m, dwell_s)`

  支持：

  - `cross`
  - `rectangle`
  - `square`

主要轨迹辅助函数：

- `slew_setpoint(current, target, dt, horizontal_speed, vertical_speed)`

  以有限水平/垂直速度把当前位置 setpoint 平滑移动到目标。水平用二维向量限速，避免对角线运动超速。

- `slew_setpoint_with_velocity(...)`

  返回平滑后 setpoint 和对应前馈速度。

- `target_is_stable(...)`

  判断：

  - setpoint 是否已经走到目标附近。
  - 实际位置是否在水平/垂直误差容差内。
  - 实际速度是否足够小。

核心类：`PositionMissionTest`

主要参数：

- `mission_shape`
  - `cross` / `rectangle` / `square`
- `distance_m`
  - cross 的单边距离，或 rectangle/square 的边长。
- `takeoff_height_m`
- `dwell_s`
- `horizontal_speed_m_s`
- `climb_speed_m_s`
- `descent_speed_m_s`
- `horizontal_tolerance_m`
- `vertical_tolerance_m`
- `speed_tolerance_m_s`
- `phase_timeout_s`
- `wait_timeout_s`

运行状态机：

1. 等待有效 local position。
2. 预流 setpoint 至少 2 秒，满足 PX4 offboard 切换要求。
3. 通过 `VehicleCommand` 请求：
   - `VEHICLE_CMD_DO_SET_MODE`，param2 = 6，进入 offboard。
   - `VEHICLE_CMD_COMPONENT_ARM_DISARM`，param1 = 1，arm。
4. armed/offboard 后记录 origin。
5. 进入第一个任务 phase。
6. 每 0.05 s 平滑更新 setpoint 和前馈速度，发布到 `/fmu/in/trajectory_setpoint`。
7. 检查安全边界：
   - 水平位移不能超过任务最大水平偏移 + 1 m。
   - 高度不能超过起飞高度 + 0.8 m。
   - 不能明显低于起点。
   - 速度不能超过 1.8 m/s。
8. 每个 phase 达到目标并稳定 `dwell_s` 后进入下一阶段。
9. land phase 完成后发送强制 disarm：

   ```text
   VEHICLE_CMD_COMPONENT_ARM_DISARM
   param1 = 0
   param2 = 21196
   ```

10. 输出 `POSITION_MISSION_RESULT` JSON。

如果中途 offboard 丢失、超出安全边界或 phase 超时，会进入 `_abort_to_land()`，保持受控下降，而不是立即关输出。

这个文件是当前最适合测试 `alpha_tailsitter_controller` 轨迹跟踪的脚本。

## 4. 基础控制器和 alpha 控制器的差异

### 4.1 基础控制器 `tailsitter_controller`

基础控制器使用：

- `PositionController`
- `LinearAccelerationINDIController`
- `AttitudeINDIController`
- `PhoenixHoverAllocator`

它的建模假设偏悬停和低速：

- 推进力主要沿 TS +x。
- 舵面力用简化的转速相关模型估计。
- 力到姿态用 `force_to_tailsitter_attitude()`，不使用完整 alpha 平坦变换。

适合：

- 悬停附近。
- 小姿态阶跃。
- 小位置阶跃。
- 初步通道和符号验证。

### 4.2 alpha 控制器 `alpha_tailsitter_controller`

alpha 控制器使用：

- `AlphaAerodynamics`
- `AlphaTheoryAllocator`
- `FlatnessAttitudeController`
- `AttitudeINDIController`
- `PositionController`

它显式考虑：

- 速度方向。
- alpha 坐标。
- zero-lift angle。
- 推力安装角。
- 速度相关升阻力。
- 舵面瞬态和稳态分量。
- 速度相关舵面控制效率。
- 微分平坦生成的姿态和角速度前馈。

适合：

- 更接近 Tailsitter-control 原始算法的轨迹跟踪测试。
- 与 Gazebo `TailsitterAlphaAero` 插件进行模型一致性验证。
- 参数辨识和 alpha 模型调试。

## 5. 与 PX4/Gazebo 的交互关系

ROS2 控制器只直接和 PX4 ROS 2 bridge 交互，不直接控制 Gazebo joint。

### 5.1 ROS2 到 PX4

控制节点发布：

```text
/fmu/in/offboard_control_mode
/fmu/in/actuator_motors
/fmu/in/actuator_servos
/fmu/in/vehicle_thrust_setpoint
```

其中 `offboard_control_mode.direct_actuator = True`，表示使用 direct actuator offboard。

### 5.2 PX4 到 Gazebo

PX4 根据 airframe 参数把 actuator 输出映射到 Gazebo：

- ESC 输出映射到 Gazebo motor speed topic。
- Servo 输出映射到 Gazebo servo topic。

Gazebo 侧：

- `MulticopterMotorModel` 只负责 rotor joint 转速动态。
- `TailsitterAlphaAero` 读取 rotor joint 速度和 servo topic，计算并施加气动力/力矩。

### 5.3 Gazebo 到 ROS2 控制器

如果启用 joint feedback，控制器订阅 Gazebo joint state topic，经过 `actuator_feedback.py` 解析成：

```text
motor_speed_left_right
flap_angle_left_right
```

再用于：

- 当前执行器状态估计。
- INDI 力/力矩估计。
- debug 输出。

如果 joint feedback 不可用，控制器会用内部一阶电机模型和目标舵面角预测执行器状态。

## 6. 常用运行入口

`setup.py` 注册了以下 console scripts：

```text
tailsitter_controller
alpha_tailsitter_controller
actuator_probe
attitude_step_test
position_step_test
position_mission_test
```

用途对应：

- `tailsitter_controller`：基础悬停控制器。
- `alpha_tailsitter_controller`：完整 alpha 理论控制器。
- `actuator_probe`：单通道 actuator 映射测试。
- `attitude_step_test`：姿态阶跃响应测试。
- `position_step_test`：小范围位置阶跃测试。
- `position_mission_test`：自动起飞、轨迹任务、降落、disarm 测试。

## 7. 调试建议

观察控制器状态时优先看：

```text
/phoenix_tailsitter/control_debug
/phoenix_tailsitter/position_debug
/phoenix_tailsitter/actuator_feedback_debug
/phoenix_tailsitter/alpha_model_debug
```

其中 `/phoenix_tailsitter/alpha_model_debug` 只由 `alpha_tailsitter_controller` 发布。

如果飞机不动，先检查：

1. `output_enabled` 是否为 true。
2. PX4 是否 armed。
3. PX4 是否进入 offboard。
4. `/fmu/in/offboard_control_mode` 是否持续发布。
5. actuator topic 是否有非零输出。
6. Gazebo joint state 是否能被解析。

如果跟踪效果差，重点检查：

1. `config.py` 中 alpha 参数是否与 `models/phoenixdrone_alpha/model.sdf` 一致。
2. 电机和舵面通道顺序是否正确。
3. `actuator_feedback_mode` 是否拿到了新鲜反馈。
4. `position_debug` 中 `force_command` 和 `limited_force` 是否被限幅。
5. `alpha_model_debug` 中模型估计力/力矩是否合理。

## 8. 矩形轨迹测试入口

当前 `position_mission_test.py` 支持矩形/方形任务。典型命令：

```bash
ros2 run phoenix_tailsitter_control position_mission_test --ros-args \
  -p confirmation:=PHOENIX_POSITION_MISSION_TEST \
  -p mission_shape:=rectangle \
  -p distance_m:=2.0 \
  -p takeoff_height_m:=2.0 \
  -p dwell_s:=2.0 \
  -p horizontal_speed_m_s:=0.15 \
  -p climb_speed_m_s:=0.30 \
  -p descent_speed_m_s:=0.18
```

该命令会让飞机执行：

```text
起飞到 2 m
-> NED +x 前进 2 m
-> NED -y 向左 2 m
-> NED -x 返回
-> NED +y 回 home 上方
-> 降落
-> disarm
```

需要同时运行 `alpha_tailsitter_controller`，并且控制器参数中：

```text
output_enabled:=true
setpoint_source:=trajectory
```

否则测试脚本只会发布轨迹 setpoint，控制器不会真正输出执行器命令。
