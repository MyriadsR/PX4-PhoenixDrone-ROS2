# PX4-PhoenixDrone-ROS2 修改总结

本文档总结当前工作区相对原 PX4 工程新增或修改的主要内容。核心目标是把 PhoenixDrone 双旋翼尾坐式无人机从旧的 PX4-PhoenixDrone/Gazebo Classic/ROS 1 流程，迁移到 PX4 v1.16、Gazebo Harmonic 和 ROS 2 Jazzy 的仿真与离板控制流程。

## 1. 修改总览

当前有价值的代码改动集中在以下几类：

| 类别 | 路径 | 作用 |
| --- | --- | --- |
| PX4 机架脚本 | `ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_phoenixdrone` | 新增 `gz_phoenixdrone` SITL 机架，设置仿真模型、控制分配、VTOL 和控制器参数 |
| 机架注册 | `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt` | 把 `4022_gz_phoenixdrone` 加入 ROMFS 打包列表 |
| Gazebo 插件 | `src/modules/simulation/gz_plugins/phoenix_aero/` | 新增 `PhoenixAero`，模拟旋翼滑流中舵面的升力、阻力和俯仰力矩 |
| 插件构建 | `src/modules/simulation/gz_plugins/CMakeLists.txt` | 编译并纳入 `PhoenixAero` 到 `px4_gz_plugins` |
| Gazebo 模型/世界 | `Tools/simulation/gz/models/phoenixdrone/`、`Tools/simulation/gz/worlds/stars_ts.sdf` | 新增 PhoenixDrone SDF 模型、网格资源和 `stars_ts` 世界 |
| DDS 话题 | `src/modules/uxrce_dds_client/dds_topics.yaml` | 打开 ROS 2 控制器需要订阅的 PX4 输出话题 |
| ROS 2 工作区 | `ros2_ws/src/phoenix_offboard/` | 新增 ROS 2 Jazzy 离板控制包，包含位置、姿态、矩形轨迹 demo 和控制律移植 |
| 开发配置 | `.vscode/settings.json` | 配置 ROS 2 Jazzy 与 Python 补全路径 |

另外，工作区中还存在 `.venv/`、`ros2_ws/build/`、`ros2_ws/install/`、`ros2_ws/log/`、`.pytest_cache/` 和若干 `.orig` 文件。这些更像本地环境、构建产物或备份文件，不是核心功能代码，提交前建议清理或加入忽略规则。

## 2. PX4 机架和参数改动

### 2.1 新增 `4022_gz_phoenixdrone`

文件：`ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_phoenixdrone`

这个脚本定义了 PX4 SITL 启动时使用的机架、模型和关键控制参数：

```sh
PX4_SIMULATOR=${PX4_SIMULATOR:=gz}
PX4_GZ_WORLD=${PX4_GZ_WORLD:=stars_ts}
PX4_SIM_MODEL=${PX4_SIM_MODEL:=phoenixdrone}

param set-default SIM_GZ_EN 1
param set-default MAV_TYPE 19
```

含义：

- 使用 Gazebo 新版 `gz` 仿真后端。
- 默认加载 `stars_ts` 世界和 `phoenixdrone` 模型。
- `MAV_TYPE 19` 表示 VTOL 类机体。

控制分配器配置为双旋翼、两片控制舵面：

```sh
param set-default CA_AIRFRAME 4
param set-default CA_ROTOR_COUNT 2
param set-default CA_ROTOR0_PY 0.195
param set-default CA_ROTOR0_KM -0.023
param set-default CA_ROTOR1_PY -0.195
param set-default CA_ROTOR1_KM 0.023

param set-default CA_SV_CS_COUNT 2
param set-default CA_SV_CS0_TYPE 5
param set-default CA_SV_CS0_TRQ_P 0.5
param set-default CA_SV_CS0_TRQ_Y 0.5
param set-default CA_SV_CS1_TYPE 6
param set-default CA_SV_CS1_TRQ_P 0.5
param set-default CA_SV_CS1_TRQ_Y -0.5
```

这里的关键点是：

- 两个旋翼沿机体 `Y` 轴对称放置，位置为 `+/-0.195 m`。
- `KM` 符号相反，用于模拟左右旋翼反扭矩。
- 两个舵面分别参与俯仰和偏航力矩分配。

Gazebo actuator bridge 映射如下：

```sh
param set-default SIM_GZ_EC_FUNC1 101
param set-default SIM_GZ_EC_FUNC2 102
param set-default SIM_GZ_SV_FUNC1 201
param set-default SIM_GZ_SV_FUNC2 202
```

这让 PX4 的执行器输出和 Gazebo 模型中的 motor/servo topic 对应起来。后续 `phoenix_controller.py` 发布的 `/fmu/in/actuator_motors` 与 `/fmu/in/actuator_servos` 会进入这条链路。

脚本还对尾坐式飞行需要的控制参数做了定制：

- 禁用直接执行器离板控制时可能误触发的自动落地解锁：

```sh
param set-default COM_DISARM_LAND -1
```

- 调整固定翼控制器、空速范围、VTOL 转换参数和多旋翼姿态/角速度环参数。
- `WV_EN 0` 关闭 weather vane。
- `EKF2_FUSE_BETA 0` 关闭侧滑融合。

### 2.2 注册机架

文件：`ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`

新增：

```cmake
4022_gz_phoenixdrone
```

没有这一步，PX4 ROMFS 不会把新机架脚本打包进 SITL，`make px4_sitl_default gz_phoenixdrone` 无法找到该 airframe。

## 3. Gazebo Harmonic 模型和空气动力插件

### 3.1 新增 PhoenixDrone 模型

路径：`Tools/simulation/gz/models/phoenixdrone/`

主要文件：

- `model.sdf`：Gazebo Harmonic 使用的 SDF 模型。
- `model.config`：模型元信息。
- `meshes/`：机体和螺旋桨网格。
- `model.classic.sdf`、`model.sdf.orig`：旧模型或备份文件，提交前建议确认是否保留。

`model.sdf` 中定义了：

- `base_link`：主机体，质量 `0.5 kg`，惯量约 `ixx=0.0147563`、`iyy=0.00638929`、`izz=0.0177`。
- `rotor_left`、`rotor_right`：两个旋翼 link，分别通过 `rotor_left_joint` 和 `rotor_right_joint` 连接到机体。
- `left_elevon`、`right_elevon`：两个舵面 link，通过 revolute joint 连接。
- IMU、气压计、磁力计、GPS 和 airspeed 模型。

传感器放在 `base_link` 下的原因写在 SDF 注释里：

```xml
<!-- PX4 v1.16 gz_bridge uses fixed base_link sensor topic paths. -->
```

也就是为了匹配 PX4 v1.16 的 Gazebo bridge 对传感器 topic 路径的假设。

旋翼 motor plugin 使用 Gazebo 自带的 `MulticopterMotorModel`：

```xml
<plugin name="gz::sim::systems::MulticopterMotorModel"
        filename="gz-sim-multicopter-motor-model-system">
  <jointName>rotor_left_joint</jointName>
  <linkName>rotor_left</linkName>
  <turningDirection>cw</turningDirection>
  <maxRotVelocity>800</maxRotVelocity>
  <motorConstant>7.864e-06</motorConstant>
  <momentConstant>0.023</momentConstant>
  <commandSubTopic>command/motor_speed</commandSubTopic>
  <motorNumber>1</motorNumber>
  <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
  <motorType>velocity</motorType>
</plugin>
```

右旋翼同理，但 `turningDirection` 为 `ccw`，`motorNumber` 为 `0`。

### 3.2 新增 `stars_ts` 世界

文件：`Tools/simulation/gz/worlds/stars_ts.sdf`

该世界是一个轻量的尾坐式测试环境：

```xml
<world name="stars_ts">
  <physics type="ode">
    <max_step_size>0.002</max_step_size>
    <real_time_factor>1.0</real_time_factor>
    <real_time_update_rate>500</real_time_update_rate>
  </physics>
  <gravity>0 0 -9.8066</gravity>
</world>
```

要点：

- ODE 物理步长 `0.002 s`，等价 `500 Hz`。
- 简单地面和方向光。
- 经纬度设置为 PX4 常见 SITL 默认位置附近。

### 3.3 新增 `PhoenixAero` 插件

路径：`src/modules/simulation/gz_plugins/phoenix_aero/`

构建文件：

```cmake
project(PhoenixAero)

add_library(${PROJECT_NAME} SHARED
    PhoenixAero.cpp
)
```

并在上层 `src/modules/simulation/gz_plugins/CMakeLists.txt` 中新增：

```cmake
add_subdirectory(phoenix_aero)
```

同时把 `PhoenixAero` 加入 `px4_gz_plugins` 依赖，保证默认构建插件集合时会生成 `libPhoenixAero.so`。

插件类定义：

```cpp
class PhoenixAero final : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
```

它实现两个阶段：

- `Configure()`：读取 SDF 参数、找到 link/joint、订阅舵面控制 topic。
- `PreUpdate()`：每个仿真步读取旋翼转速和舵面命令，计算力/力矩并施加到机体。

SDF 中每个舵面各挂一个插件实例：

```xml
<plugin name="phoenix::PhoenixAero" filename="libPhoenixAero.so">
  <rotor_velocity_slowdown>10</rotor_velocity_slowdown>
  <k_lift>0.00000348</k_lift>
  <k_drag>0.00000175</k_drag>
  <k_pitch>-0.000000344</k_pitch>
  <cp>0 0.195 0.036</cp>
  <forward>0 0 1</forward>
  <upward>1 0 0</upward>
  <link_name>base_link</link_name>
  <motor_joint_name>rotor_left_joint</motor_joint_name>
  <control_sub_topic>servo_1</control_sub_topic>
  <control_joint_name>left_elevon_joint</control_joint_name>
</plugin>
```

插件的核心计算在 `PhoenixAero::PreUpdate()`：

```cpp
const double omega = std::abs((*motor_velocity)[0]) * _rotor_velocity_slowdown;
const double delta = _control_command.load(std::memory_order_relaxed);

const math::Vector3d force = upward_world * (_k_lift * omega_squared * delta)
    -forward_world * (_k_drag * omega_squared * delta * delta);
const math::Vector3d torque = spanwise_world * (_k_pitch * omega_squared * delta);

_link.AddWorldWrench(_ecm, force, torque, _cp);
```

物理含义：

- `omega` 是 Gazebo joint velocity 乘以 `rotor_velocity_slowdown` 后的实际近似旋翼角速度。
- `delta` 是舵面偏角命令。
- 升力项和 `omega^2 * delta` 成正比。
- 阻力项和 `omega^2 * delta^2` 成正比，方向沿 `-forward_world`。
- 俯仰力矩和 `omega^2 * delta` 成正比，方向沿 spanwise 轴。
- 最后通过 `AddWorldWrench()` 作用到 `base_link` 的指定压力中心 `cp`。

插件还使用 `JointPositionReset` 把舵面 joint 位置直接设为命令值：

```cpp
components::JointPositionReset(joint_position)
```

这对应旧 Gazebo Classic 模型中的 kinematic 舵机行为，不额外模拟舵机 PID 动态。

## 4. DDS 话题改动

文件：`src/modules/uxrce_dds_client/dds_topics.yaml`

新增或启用的 PX4 输出话题：

```yaml
- topic: /fmu/out/vehicle_angular_velocity
  type: px4_msgs::msg::VehicleAngularVelocity

- topic: /fmu/out/vehicle_attitude_setpoint
  type: px4_msgs::msg::VehicleAttitudeSetpoint
```

原因：

- `phoenix_controller.py` 需要 `/fmu/out/vehicle_angular_velocity` 作为角速度反馈。
- `phoenix_controller.py` 既可以消费 PX4 内部姿态设定值 `/fmu/out/vehicle_attitude_setpoint`，也可以消费 ROS 2 输入端的 `/fmu/in/vehicle_attitude_setpoint`，从而支持 position controller 和 attitude demo 两条路径。

## 5. ROS 2 Jazzy 离板控制工作区

路径：`ros2_ws/`

### 5.1 工作区定位

`ros2_ws/README.md` 写明：

- 使用 ROS 2 Jazzy。
- `px4_msgs` 固定在 PX4 v1.16.2 对应的 `392e831`。
- `phoenix_offboard` 移植自 PX4-PhoenixDrone commit `70af6cbc` 的两个 ROS 1 offboard demo。
- demo 不自动切 Offboard、不自动 arm，需要用户手动发送 VehicleCommand。

运行方式：

```sh
source /opt/ros/jazzy/setup.bash
cd /home/zr/PX4-PhoenixDrone-ROS2/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch phoenix_offboard phoenix_sitl.launch.py demo:=position
```

进入 Offboard 和 arm 需要额外发布：

```sh
ros2 topic pub --once /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand \
  "{timestamp: 1, param1: 1.0, param2: 6.0, command: 176, target_system: 1, target_component: 1, source_system: 1, source_component: 1, from_external: true}"

ros2 topic pub --once /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand \
  "{timestamp: 1, param1: 1.0, command: 400, target_system: 1, target_component: 1, source_system: 1, source_component: 1, from_external: true}"
```

### 5.2 `phoenix_sitl.launch.py`

文件：`ros2_ws/src/phoenix_offboard/launch/phoenix_sitl.launch.py`

该 launch 同时启动：

- Micro XRCE-DDS Agent：

```python
cmd=[str(AGENT), 'udp4', '-p', '8888']
```

- PX4 SITL：

```python
cmd=['make', 'px4_sitl_default', 'gz_phoenixdrone']
```

- 总是启动 `phoenix_controller`。
- `demo:=position` 或 `demo:=rectangle` 时启动 `phoenix_position_controller`。
- 根据 `demo` 参数选择 `position_demo`、`rectangle_demo` 或 `attitude_demo`。

这里的控制链路是：

```text
position_demo / rectangle_demo
  -> /fmu/in/trajectory_setpoint
  -> phoenix_position_controller
  -> /fmu/in/vehicle_attitude_setpoint
  -> phoenix_controller
  -> /fmu/in/actuator_motors + /fmu/in/actuator_servos
  -> PX4 gz bridge
  -> Gazebo motor plugin + PhoenixAero plugin
```

`attitude_demo` 则绕过位置控制器，直接发布姿态设定值给 `phoenix_controller`。

### 5.3 QoS 设置

文件：`ros2_ws/src/phoenix_offboard/phoenix_offboard/qos.py`

输入输出都使用 PX4 DDS 常见 QoS：

```python
QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
```

这和 PX4 uXRCE-DDS 的 topic 行为匹配，避免可靠性策略不一致导致 ROS 2 节点收不到数据。

## 6. 控制器移植细节

### 6.1 姿态/角速度/执行器控制器

文件：`ros2_ws/src/phoenix_offboard/phoenix_offboard/phoenix_controller.py`

这个节点是原 `ts_att_control` 和 `ts_rate_control` 的 ROS 2 Python 移植。它订阅：

```python
'/fmu/out/vehicle_attitude'
'/fmu/out/vehicle_angular_velocity'
'/fmu/out/vehicle_attitude_setpoint'
'/fmu/in/vehicle_attitude_setpoint'
'/fmu/out/vehicle_control_mode'
```

发布：

```python
'/fmu/in/offboard_control_mode'
'/fmu/in/actuator_motors'
'/fmu/in/actuator_servos'
```

关键常量来自旧 commit：

```python
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
```

姿态误差计算保留 reduced-attitude 思路：

```python
r_z = [r[0][2], r[1][2], r[2][2]]
r_sp_z = [r_sp[0][2], r_sp[1][2], r_sp[2][2]]
e_r = mat_vec(transpose(r), cross(r_z, r_sp_z))
```

也就是优先对齐当前机体系 `Z` 轴和期望机体系 `Z` 轴，再处理偏航误差。

控制主循环在每次收到角速度时触发：

```python
rates_sp = [constrain(self.ATT_P[i] * e_r[i], -self.RATE_MAX[i], self.RATE_MAX[i])
            for i in range(3)]
rates_err = [rates_sp[i] - rates[i] for i in range(3)]
gyroscopic = cross(rates, j_rates)
moment = [self.INERTIA[i] * rates_err[i] / self.RATE_TC[i] + gyroscopic[i]
          for i in range(3)]
```

然后使用非线性混控 `_mix()` 把总推力和三轴力矩转换为两个电机和两个舵面：

```python
omega_l2 = (mx + 2.0 * force_per_motor * l) / (2.0 * self.KT * l)
omega_r2 = -(mx - 2.0 * force_per_motor * l) / (2.0 * self.KT * l)

motors = [
    constrain(math.sqrt(max(0.0, omega_l2)) / self.MAX_OMEGA, -1.0, 1.0),
    constrain(math.sqrt(max(0.0, omega_r2)) / self.MAX_OMEGA, -1.0, 1.0),
]
```

舵面输出按旧 bridge 的度数范围归一化：

```python
servos = [
    constrain(deg_r / 30.0, -1.0, 1.0),
    constrain(deg_l / 60.0, -1.0, 1.0),
]
```

安全逻辑：

```python
motors, servos = self._mix(force_per_motor, moment) if self._armed else ([0.0, 0.0], [0.0, 0.0])
```

只有 PX4 报告 `flag_armed` 后才真正输出电机和舵面命令。

### 6.2 位置控制器

文件：`ros2_ws/src/phoenix_offboard/phoenix_offboard/phoenix_position_controller.py`

这个节点移植原 `mc_pos_control` 的 TS_POS 逻辑。订阅：

```python
'/fmu/in/trajectory_setpoint'
'/fmu/out/vehicle_local_position'
```

发布：

```python
'/fmu/in/vehicle_attitude_setpoint'
```

核心参数：

```python
POS_TC = [1.0, 1.0, 0.2]
POS_DR = [2.0 * 0.6, 2.0 * 0.6, 2.0 * 1.5]
MASS = 0.63
GRAVITY = 9.8066
THR_MIN = 0.2
THR_MAX = 10.0
TILT_MAX = math.radians(30.0)
```

如果 trajectory 没有直接给 acceleration，就使用位置/速度误差生成加速度：

```python
acceleration = [
    (position_sp[i] - position[i]) / (self.POS_TC[i] ** 2)
    + (velocity_sp[i] - velocity[i]) / self.POS_TC[i] * self.POS_DR[i]
    for i in range(3)
]
```

再转为 NED 力：

```python
force = [
    self.MASS * acceleration[0],
    self.MASS * acceleration[1],
    self.MASS * acceleration[2] - self.MASS * self.GRAVITY,
]
```

`_limit_force()` 保留旧 offboard 分支的限制顺序：

- 最小推力限制。
- 水平力按最大倾角 `30 deg` 限制。
- 总力幅值按 `THR_MAX` 限制。

最后 `_force_to_attitude()` 把力向量转成姿态四元数和单电机推力：

```python
body_z = [-value / force_abs for value in force]
rotation = [[body_x[i], body_y[i], body_z[i]] for i in range(3)]
return dcm_to_quat(rotation), force_abs / 2.0
```

代码中特意保留了旧实现的固定航向行为：

```python
y_c = [-1.0, 0.0, 0.0]
```

所以 position demo 的轨迹命令只主要控制位置，不改变航向。

### 6.3 Demo 节点

`position_demo.py` 每 `0.1 s` 发布一次 OffboardControlMode 和 NED 位置设定值：

```python
mode.position = True
setpoint.position = [0.0, 0.0, -1.0]
```

注意这里从原 MAVROS ENU `(0, 0, +1)` 转成了 PX4 DDS NED `(0, 0, -1)`。

`attitude_demo.py` 直接发布姿态设定值：

```python
pitch = 0.1 * math.sin(0.5 * elapsed)
setpoint.q_d = [math.cos(0.5 * pitch), 0.0, math.sin(0.5 * pitch), 0.0]
setpoint.thrust_body = [0.0, 0.0, -throttle]
```

它用于绕过位置控制器，单独测试姿态和执行器控制链路。

`rectangle_demo.py` 定义四个 NED 航点：

```python
WAYPOINTS = [
    [0.0, 0.0, -1.0],
    [0.5, 0.0, -1.0],
    [0.5, 0.3, -1.0],
    [0.0, 0.3, -1.0],
]
```

状态机为：

```text
wait_for_arm -> fly -> land -> disarm
```

它会等待 PX4 已解锁后开始移动 setpoint，完成矩形后下降到 `LAND_Z=-0.04`，再发送 `VEHICLE_CMD_COMPONENT_ARM_DISARM` 执行 disarm。

## 7. 测试

文件：`ros2_ws/src/phoenix_offboard/test/test_control_laws.py`

当前测试覆盖四个关键点：

- 单位姿态到单位姿态的 reduced-attitude 误差为零。
- 零力矩时非线性混控输出左右电机对称、舵面为零。
- 悬停力转换到姿态时保留旧实现的固定航向四元数。
- 位置控制力限制逻辑匹配旧 offboard 分支。

示例：

```python
def test_nonlinear_mixer_is_symmetric_at_zero_moment():
    controller = PhoenixController.__new__(PhoenixController)
    motors, servos = controller._mix(0.4, [0.0, 0.0, 0.0])
    expected_motor = math.sqrt(0.4 / PhoenixController.KT) / PhoenixController.MAX_OMEGA
    assert motors == pytest.approx([expected_motor, expected_motor])
    assert servos == pytest.approx([0.0, 0.0])
```

这些测试主要验证移植控制律的数学行为，不覆盖完整 PX4/Gazebo 联合仿真。

## 8. 开发环境配置

文件：`.vscode/settings.json`

新增：

```json
"ROS2.distro": "jazzy",
"python.autoComplete.extraPaths": [
  "/opt/ros/jazzy/lib/python3.12/site-packages"
],
"python.analysis.extraPaths": [
  "/opt/ros/jazzy/lib/python3.12/site-packages"
]
```

这让 VS Code 能识别 ROS 2 Jazzy 的 Python 包，例如 `rclpy` 和 `px4_msgs`。

## 9. 当前版本控制注意项

以下内容从功能上看不应作为项目源码提交，建议后续清理：

- `.venv/`：本地 Python 虚拟环境。
- `ros2_ws/build/`、`ros2_ws/install/`、`ros2_ws/log/`：colcon 构建产物。
- `ros2_ws/src/phoenix_offboard/.pytest_cache/`：pytest 缓存。
- `*.orig`：备份文件，例如 `PhoenixAero.cpp.orig`、`model.sdf.orig`。

另外，`Tools/simulation/gz` 是一个嵌套 Git 仓库，当前主仓库显示为 `? Tools/simulation/gz`。其中真正和本次移植相关的新增内容是：

- `models/phoenixdrone/`
- `worlds/stars_ts.sdf`

如果要正式提交，需要明确 `Tools/simulation/gz` 是作为子模块、subtree，还是普通目录纳入当前仓库。

## 10. 总体控制闭环

整体运行时数据流可以概括为：

```text
ROS 2 demo
  -> TrajectorySetpoint 或 VehicleAttitudeSetpoint
  -> phoenix_position_controller 生成姿态/推力
  -> phoenix_controller 生成 direct actuator
  -> PX4 uXRCE-DDS client
  -> PX4 Gazebo bridge
  -> Gazebo 双电机模型和 PhoenixAero 舵面空气动力
  -> 机体运动与传感器反馈
  -> PX4 estimator/controller topic
  -> ROS 2 控制器反馈闭环
```

这套修改的关键价值在于：不改 PX4 主控制器的大结构，而是在 ROS 2 侧复刻旧 PhoenixDrone 的控制律，通过 PX4 v1.16 的 direct actuator DDS 输入驱动 Gazebo Harmonic 中的新 PhoenixDrone 模型。
