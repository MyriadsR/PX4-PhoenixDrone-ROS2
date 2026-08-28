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

## 11. 将 Tailsitter-control 控制器用于当前 PX4 仿真平台需要的工作

如果要把 `/home/zr/Tailsitter-control` 项目里的控制器迁移到当前 `PX4-PhoenixDrone-ROS2` 仿真平台，不能只复制一个简单控制函数。该项目实际是一套完整的串级控制链路：

```text
参考轨迹 position/velocity/acceleration/jerk/yaw
  -> 位置反馈控制
  -> 线加速度 INDI
  -> differential-flatness 姿态/推力求解
  -> 姿态 PD + 角速度前馈
  -> 角加速度 INDI
  -> 双电机/双舵面分配
  -> 执行器一阶动态 + 气动/动力学反馈
```

主循环主要在 `/home/zr/Tailsitter-control/main_sim.py` 中，核心控制模块包括：

- `/home/zr/Tailsitter-control/controller/pos_ctrl.py`
- `/home/zr/Tailsitter-control/controller/INDI_acc_ctrl.py`
- `/home/zr/Tailsitter-control/controller/att_ctrl.py`
- `/home/zr/Tailsitter-control/controller/actuator_ctrl.py`
- `/home/zr/Tailsitter-control/environment/aerodynamics.py`
- `/home/zr/Tailsitter-control/utils/filters.py`
- `/home/zr/Tailsitter-control/config.py`

### 11.1 推荐接入方式

当前工程已经有 ROS 2 offboard 控制框架，因此最现实的做法是先把 Tailsitter-control 作为一个新的 ROS 2 节点接入，而不是直接移植成 PX4 内部 C++ 控制模块。

建议新增类似下面的结构：

```text
ros2_ws/src/phoenix_offboard/phoenix_offboard/tailsitter_control/
  __init__.py
  config.py
  filters.py
  pos_ctrl.py
  indi_acc_ctrl.py
  att_ctrl.py
  actuator_ctrl.py
  math_utils.py

ros2_ws/src/phoenix_offboard/phoenix_offboard/tailsitter_controller.py
```

新的 `tailsitter_controller.py` 负责：

- 订阅 PX4 通过 uXRCE-DDS 输出的状态量。
- 订阅或生成 `TrajectorySetpoint` 参考轨迹。
- 调用 Tailsitter-control 的位置、加速度、姿态、角加速度和执行器分配控制律。
- 发布 `OffboardControlMode`，并设置 `direct_actuator=True`。
- 发布 `ActuatorMotors` 和 `ActuatorServos`。
- 做解锁状态、数据超时、有限值检查和输出限幅。

接入新控制器时，不能同时运行当前已有的 `phoenix_position_controller.py` 和 `phoenix_controller.py`，否则多个节点会同时向 PX4 写入控制输入，导致控制权冲突。

### 11.2 需要对齐的状态输入

Tailsitter-control 依赖的状态量比当前简化控制器更多。当前工程已有的 DDS 话题能提供一部分：

- `/fmu/out/vehicle_attitude`：姿态四元数。
- `/fmu/out/vehicle_angular_velocity`：机体系角速度。
- `/fmu/out/vehicle_local_position`：NED 位置、速度，以及估计加速度字段。
- `/fmu/out/vehicle_odometry`：位置、速度、姿态综合状态。
- `/fmu/out/airspeed_validated`：空速相关估计。
- `/fmu/in/actuator_motors`：电机 direct actuator 输入。
- `/fmu/in/actuator_servos`：舵面 direct actuator 输入。
- `/fmu/in/offboard_control_mode`：offboard 控制模式心跳。

其中需要特别注意坐标系：

- PX4 的世界坐标通常是 NED。
- 机体系通常是 FRD。
- `VehicleAttitude.q` 是 Hamilton 四元数 `[w, x, y, z]`，表示 FRD body 到 NED。
- `VehicleAngularVelocity.xyz` 是 FRD 机体系角速度。
- Tailsitter-control 里的数学推导和当前 PX4/Gazebo 数据之间必须逐项确认坐标方向、重力符号和四元数乘法顺序。

Tailsitter-control 的 INDI 控制还需要线加速度和角加速度反馈。当前可以先用以下方式做最小实现：

- 线加速度：先使用 `VehicleLocalPosition.ax/ay/az` 或从机体系加速度转换得到世界系加速度。
- 角加速度：对 `VehicleAngularVelocity.xyz` 做低通滤波后差分。
- 执行器反馈：先用上一周期指令加一阶执行器模型估计电机转速和舵面角度。

更完整的实现应该把 Gazebo 中实际电机转速和舵面位置反馈回 ROS 2 或 PX4。否则 INDI 控制器拿到的是“估计执行器状态”，不是实际执行器状态，快速机动时误差会比较明显。

### 11.3 需要补充或确认的 DDS topic

当前 `src/modules/uxrce_dds_client/dds_topics.yaml` 中已经暴露了大部分 offboard 控制需要的话题。但如果要更接近 Tailsitter-control 原始控制律，建议检查并按需加入：

- `VehicleAcceleration`：更直接的机体系加速度反馈。
- 执行器实际状态相关 topic：用于拿到真实电机转速和舵面位置。
- 调试输出 topic：用于记录 INDI 中间量、控制分配结果、滤波后的加速度和角加速度。

其中 `VehicleAcceleration` 是否能直接用于线加速度控制，还要通过静止仿真测试确认符号。PX4 中该消息通常是机体系 FRD 下的加速度测量，可能包含重力或需要补偿重力，不能直接假设它等于世界系轨迹二阶导数。

### 11.4 飞机参数不一致的问题

Tailsitter-control 的参数和当前 Gazebo 模型差异很大，不能直接照搬增益和模型参数。

Tailsitter-control 中的典型参数包括：

- 质量约 `0.7 kg`。
- 惯量约为 `diag([0.0095, 0.0030, 0.0115])`。
- 电机力臂 `l_Ty=0.15 m`。
- 舵面力臂 `l_dy=0.12 m`，`l_dx=0.075 m`。
- 电机最大角速度 `2500 rad/s`。
- 电机时间常数约 `0.04 s`。
- 舵机时间常数约 `0.03 s`。
- 舵面最大角度约 `1 rad`。
- 控制频率 `500 Hz`。
- INDI 常用 `15 Hz` Butterworth 低通滤波。

当前 Gazebo PhoenixDrone 模型大致是：

- 机体主体质量 `0.5 kg`，加上附加部件后总质量约 `0.525 kg`。
- 惯量来自 `Tools/simulation/gz/models/phoenixdrone/model.sdf`。
- 电机 y 向力臂约 `0.195 m`。
- 电机最大角速度约 `800 rad/s`。
- 电机力常数 `7.864e-6`。
- 电机力矩常数约 `0.023`。
- 电机上升/下降时间常数约 `0.016 s / 0.020 s`。

因此必须重新整理一份当前 PX4/Gazebo 模型专用的 `config.py`。最少要重设：

- 质量 `m`。
- 惯量矩阵 `J`。
- 电机最大转速。
- 电机推力系数和反扭矩系数。
- 电机/舵机时间常数。
- 电机和舵面的安装位置、力臂、符号。
- 控制增益和 INDI 控制矩阵。

### 11.5 当前 Gazebo 气动模型的限制

当前工程中的 `src/modules/simulation/gz_plugins/phoenix_aero/PhoenixAero.cpp` 是一个简化舵面气动插件，核心模型近似为：

```text
force = upward * k_lift * omega^2 * delta
      - forward * k_drag * omega^2 * delta^2

torque = spanwise * k_pitch * omega^2 * delta
```

这和 Tailsitter-control 的完整气动模型不是一回事。Tailsitter-control 的气动部分考虑了迎角、来流速度、螺旋桨滑流、舵面诱导升力、推力安装角等因素。也就是说：

- 如果只做低速悬停和小范围姿态控制，当前 `PhoenixAero` 插件可以先用于初步验证。
- 如果要复现 Tailsitter-control 的过渡飞行、前飞、刀锋飞行或差动转弯，需要升级 Gazebo 气动插件。
- 完整移植时，应把 `/home/zr/Tailsitter-control/environment/aerodynamics.py` 中的 alpha-theory 模型改写到 `PhoenixAero.cpp`，或者重新辨识一套适合当前简化 Gazebo 模型的控制参数。

### 11.6 执行器映射必须重新标定

当前模型有两个电机和两个舵面，但索引和符号不能凭直觉使用。

需要逐项确认：

- `ActuatorMotors.control[0]` 对应哪一个 Gazebo 电机。
- `ActuatorMotors.control[1]` 对应哪一个 Gazebo 电机。
- `ActuatorServos.control[0]` 对应左舵面还是右舵面。
- `ActuatorServos.control[1]` 对应左舵面还是右舵面。
- 正舵偏到底产生的是正滚转、负滚转、正俯仰还是负俯仰力矩。
- 电机差动正负号是否和 Tailsitter-control 的分配矩阵一致。

当前 airframe 中舵面参数存在反向映射，例如 `SIM_GZ_SV_MINA1` 大于 `SIM_GZ_SV_MAXA1`，说明归一化指令到实际舵角之间存在符号翻转。必须做单通道测试后再飞闭环。

建议验证顺序：

1. 固定机体或低推力状态下，只给左电机指令，看 Gazebo 中哪个电机转动。
2. 只给右电机指令，确认索引。
3. 只给 `servo_0` 正指令，看舵面方向和产生力矩方向。
4. 只给 `servo_1` 正指令，看舵面方向和产生力矩方向。
5. 把结果写成明确的控制分配矩阵，避免在控制律里靠临时负号修正。

### 11.7 最小可行移植步骤

建议不要一次性把所有功能移完。比较稳妥的顺序是：

1. 新建独立 ROS 2 节点 `tailsitter_controller.py`，只接管 direct actuator 输出。
2. 先移植姿态 PD、角速度前馈、角加速度 INDI 和执行器分配。
3. 使用当前 Gazebo 模型参数重写 `config.py`。
4. 做电机和舵面单通道映射测试。
5. 在悬停附近用很保守的增益测试姿态闭环。
6. 加入位置控制和线加速度 INDI。
7. 用 `TrajectorySetpoint` 提供 position、velocity、acceleration、jerk、yaw 参考。
8. 加入执行器实际反馈或更准确的一阶执行器状态估计。
9. 升级 `PhoenixAero`，使其更接近 Tailsitter-control 的气动模型。
10. 最后再测试矩形轨迹、圆轨迹、过渡飞行和复杂机动。

### 11.8 需要修改的当前工程文件

较小范围的 ROS 2 移植通常会涉及：

- `ros2_ws/src/phoenix_offboard/phoenix_offboard/`：新增 Tailsitter 控制代码。
- `ros2_ws/src/phoenix_offboard/setup.py`：注册新的 console script。
- `ros2_ws/src/phoenix_offboard/package.xml`：补充依赖，例如 `numpy`、`scipy`。
- `ros2_ws/src/phoenix_offboard/launch/`：新增或修改 launch 文件，启动 Tailsitter 控制器。
- `src/modules/uxrce_dds_client/dds_topics.yaml`：按需补充加速度、执行器反馈或调试 topic。

完整气动移植还会涉及：

- `src/modules/simulation/gz_plugins/phoenix_aero/PhoenixAero.cpp`
- `src/modules/simulation/gz_plugins/phoenix_aero/PhoenixAero.hpp`
- `src/modules/simulation/gz_plugins/phoenix_aero/CMakeLists.txt`
- `Tools/simulation/gz/models/phoenixdrone/model.sdf`

### 11.9 验证标准

移植完成后至少要验证以下内容：

- ROS 2 节点能稳定收到 PX4 状态 topic。
- `OffboardControlMode.direct_actuator` 心跳持续发布。
- PX4 能进入 offboard 并保持 direct actuator 控制。
- 电机和舵面输出没有 NaN、越界或突变。
- 姿态静态误差能收敛。
- 小角度姿态阶跃响应方向正确。
- 位置小阶跃不会发散。
- rosbag 中记录的加速度、角加速度、执行器估计值和 INDI 输出相位基本一致。
- Gazebo 中模型不会因为舵面气动力符号错误而瞬间翻转。

### 11.10 结论

把 Tailsitter-control 用在当前 PX4 仿真平台下，最小版本主要改 ROS 2 控制节点；完整版本还必须改 Gazebo 气动插件。

工程上建议先完成“低速悬停版本”：只移植姿态/角速度/角加速度 INDI 和执行器分配，完成状态、坐标系、执行器映射验证后，再加入位置/线加速度 INDI。等低速闭环可靠以后，再移植完整 alpha-theory 气动模型，用于过渡飞行和前飞等复杂工况。

### 11.11 完全参照 Tailsitter-control 气动模型所需的修改

如果要完整参照 `/home/zr/Tailsitter-control`，不能只替换 `PhoenixAero`。必须同时统一仿真气动力、控制器内部模型、控制分配、执行器动态和参数，否则 INDI 使用的模型会与 Gazebo 中的被控对象不一致。

首先需要区分两种目标：

- 推荐方案是完整移植 Tailsitter-control 的模型结构和公式，但针对当前 PhoenixDrone 重新辨识参数。
- 完全复现原项目则要求公式和参数全部照搬，包括 `0.7 kg` 质量、`2500 rad/s` 电机、惯量、力臂和舵面范围等。这会使仿真对象不再是当前参数下的 PhoenixDrone。

主要差异和修改范围如下：

| 层级 | 当前实现 | 完整移植需要做的事 |
|---|---|---|
| Gazebo 气动 | 两个简化 `PhoenixAero`，只计算滑流舵效 | 改成一个整机 alpha-theory 插件 |
| 电机推力 | Gazebo `MulticopterMotorModel` 直接施加 | 避免与新气动插件重复计算推力和反扭矩 |
| 舵机动态 | 舵面角度直接复位到命令 | 加入一阶动态、速率限制和真实状态 |
| 控制器气动估计 | 简化的 `omega^2 delta` 和 `omega^2 delta^2` | 使用与 Gazebo 完全相同的 alpha-theory 方程 |
| 控制分配 | 悬停附近固定增益 | 移植速度相关的 Eq.37--40 分配 |
| 线加速度 INDI | 简化力估计与补偿 | 加入零升力坐标系和 Eq.43 舵面瞬态修正 |
| 姿态/推力求解 | 简化力向量到姿态映射 | 移植完整 differential-flatness 求解 |

#### 11.11.1 重写 Gazebo 气动插件

当前 `src/modules/simulation/gz_plugins/phoenix_aero/PhoenixAero.cpp` 的模型主要是：

```text
F_lift  proportional to omega^2 delta
F_drag  proportional to -omega^2 delta^2
M_pitch proportional to omega^2 delta
```

完整插件需要读取：

- 机体真实线速度；
- 左右电机真实转速；
- 左右舵面真实角度；
- 机体姿态；
- 可选的风速，用于计算空气相对速度。

然后按照 `/home/zr/Tailsitter-control/environment/aerodynamics.py` 实现：

```text
T_i = c_T omega_i^2

f_T      推力及滑流诱导升阻力
f_delta  舵面偏转产生的滑流/来流气动力
f_w      机翼速度相关升阻力

m_T      推力偏心力矩
m_mu     螺旋桨反扭矩
m_delta  舵面气动力矩

f_alpha = f_T + f_delta + f_w
m_body  = m_T + m_mu + m_delta
```

建议使用一个整机插件实例同时处理两台电机和两个舵面，不能继续使用左右两个完全独立的插件，因为完整模型包含总推力、差动推力及整机力矩耦合。

#### 11.11.2 严格处理四套坐标系

完整模型增加了零升力坐标系 `alpha`，转换链必须明确写成：

```text
Gazebo FLU
    -> PX4 FRD
    -> 绕 PX4 +y 轴旋转 +90 deg
Tailsitter body
    -> alpha_0
零升力坐标系 alpha
```

控制器内部仍固定使用：

```text
x_ts = -z_px4
y_ts =  y_px4
z_ts =  x_px4
```

对于 Gazebo 本体系速度，合并后的转换为：

```text
v_ts = [v_gz.z, -v_gz.y, v_gz.x]
```

气动力计算完成后，应先计算：

```text
f_body_ts = R_alpha_to_body * f_alpha
```

再从 TS 转回 Gazebo 本体系和世界系施加。原模型的 `m_body` 已经包含力臂产生的力矩，因此新插件应在质心施加合力和合力矩，不能再通过当前 `_cp` 产生第二次附加力矩。

#### 11.11.3 消除电机推力的重复计算

这是移植中最容易出错的部分。Tailsitter-control 的 `f_T` 已经包含电机推力，而当前 SDF 中的 `MulticopterMotorModel` 也会施加电机推力和螺旋桨反扭矩。如果两者同时保留，会把推力和反扭矩计算两次。

推荐方案是：

- 保留 `MulticopterMotorModel` 负责命令到转速的一阶动态和旋翼关节显示；
- 在新的模型变体中关闭它的力和力矩输出；
- 由新的 alpha-theory 插件根据实际关节转速统一施加全部力和力矩。

因此需要调整模型 SDF 中的：

```xml
<motorConstant>
<momentConstant>
<timeConstantUp>
<timeConstantDown>
<maxRotVelocity>
<rotorVelocitySlowdownSim>
```

需要先验证 `motorConstant=0` 时关节转速动态是否仍正常。如果当前 Gazebo 电机插件不支持这种方式，就应在新的气动插件中自行实现电机一阶动态。

#### 11.11.4 加入真实舵机动态

Tailsitter-control 中的舵机参数为：

```text
servo_time_constant = 0.03 s
servo_rate_max      = 25 rad/s
```

当前 `PhoenixAero` 会直接把舵面位置复位到命令，没有动态过程。新插件应维护每侧舵面状态：

```text
delta_dot = (delta_command - delta_actual) / tau_servo
delta_dot = clip(delta_dot, -servo_rate_max, +servo_rate_max)
```

气动力计算和关节状态发布都必须使用 `delta_actual`。当前已经建立的 Gazebo 关节实际反馈链路可以继续把真实舵面角和电机转速反馈给控制器。

#### 11.11.5 替换控制器内部气动估计

当前简化估计主要位于：

- `ros2_ws/src/phoenix_tailsitter_control/phoenix_tailsitter_control/position_control.py`
- `ros2_ws/src/phoenix_tailsitter_control/phoenix_tailsitter_control/allocator.py`

需要新增与 Gazebo C++ 插件逐项一致的 Python 气动模型，输入和输出为：

```text
输入：v_body_ts, omega_left/right_actual, delta_left/right_actual
输出：f_alpha, m_body_ts, f_delta_alpha
```

控制器中的以下简化逻辑需要被替代：

- `estimate_force_components_ts()`；
- `estimate_moment()`；
- 当前舵面气动力预补偿；
- 固定悬停分配矩阵。

控制器还必须根据 NED 速度计算 TS 机体系速度：

```text
v_body_ts = R_ts_to_ned.T * v_ned
```

如果加入风场，应使用空气相对速度而不是地速。

#### 11.11.6 移植完整速度相关控制分配

应移植 `/home/zr/Tailsitter-control/controller/actuator_ctrl.py` 中的 Eq.37--40：

1. 根据期望偏航力矩求差动推力。
2. 在单电机上下限内重新约束总推力和差动推力。
3. 由 `T_i=c_T omega_i^2` 反算电机转速。
4. 扣除推力偏心力矩和反扭矩。
5. 根据当前空速计算左右舵面效率 `nu_1`、`nu_2`。
6. 使用有界最小二乘求舵面角度。

当前分配器基本不依赖空速，不能直接用于完整前飞模型。

#### 11.11.7 补齐完整 INDI 滤波链

原项目要求下列信号使用相同的 15 Hz 二阶 Butterworth 低通：

- 线加速度；
- 角速度和角加速度；
- 电机实际转速；
- 舵面实际角度；
- 气动力和力矩估计。

此外还要对舵面信号使用 1 Hz 高通，并实现 Eq.43：

```text
a_lpf_tilde =
    a_lpf
    - (1/m) R_alpha_to_inertial f_delta_hpf
```

当前控制器尚未实现这条完整的舵面瞬态修正链。

#### 11.11.8 移植完整 differential-flatness 姿态求解

如果目标不仅是气动模型一致，而是完整参照整个 Tailsitter-control 控制框架，还需要用 `/home/zr/Tailsitter-control/controller/att_ctrl.py` 替换当前简化的 `force_to_tailsitter_attitude()`。

完整版本需要使用：

- 期望力；
- 当前和期望速度；
- `alpha_0`、`alpha_T`；
- 舵面合成偏角；
- jerk；
- yaw 和 yawspeed；
- 角速度前馈。

否则即使 Gazebo 气动力完全移植，过渡飞行中的期望姿态仍不等价。

#### 11.11.9 参数选择

Tailsitter-control 当前配置中的主要参数为：

```text
mass       = 0.7 kg
J          = diag(0.0095, 0.0030, 0.0115)
c_T        = 1.8e-6
omega_max  = 2500 rad/s
l_Ty       = 0.15 m
l_dy       = 0.12 m
l_dx       = 0.075 m
alpha_0    = -2 deg
alpha_T    = -5 deg
delta_max  = 1 rad
```

当前 PhoenixDrone 则大致为：

```text
mass       = 0.525 kg
omega_max  = 800 rad/s
c_T        = 7.864e-6
motor arm  = 0.195 m
```

如果直接照搬原参数，必须同步修改质量、惯量、几何、电机和舵面限制，否则同一个仿真中会存在相互矛盾的飞机参数。

更合理的方案是保留完整公式，但针对当前模型重新辨识：

```text
alpha_0, alpha_T
c_T, c_mu, c_mu_T
c_LV, c_DV
c_LT, c_DT
c_LV_delta, c_LT_delta
l_Ty, l_dy, l_dx
```

#### 11.11.10 在不修改现有代码约束下的推荐实现

为了保留当前已经验证过的简化模型，不应直接覆盖 `PhoenixAero` 和 `phoenixdrone`，而应并行新增：

```text
src/modules/simulation/gz_plugins/tailsitter_alpha_aero/
Tools/simulation/gz/models/phoenixdrone_alpha/
ros2_ws/src/phoenix_tailsitter_control/phoenix_tailsitter_control/alpha_aerodynamics.py
ros2_ws/src/phoenix_tailsitter_control/phoenix_tailsitter_control/alpha_allocator.py
ros2_ws/src/phoenix_tailsitter_control/launch/tailsitter_alpha_sitl.launch.py
```

这样可以在两套模型之间进行 A/B 对比：

```text
phoenixdrone       = 当前简化模型
phoenixdrone_alpha = 完整 alpha-theory 模型
```

构建系统仍需注册新的插件目标，但原气动实现和原模型内容可以保持不变。

#### 11.11.11 验证顺序

正式测试过渡轨迹前，至少需要完成：

1. 让 C++ 插件与原 Python 气动模型在随机状态网格上的力和力矩逐项一致。
2. 使用单位基向量测试锁定 GZ、PX4、TS 和 alpha 四套坐标转换。
3. 确认电机推力和反扭矩没有重复施加。
4. 验证悬停配平、电机差动和两舵面正负方向。
5. 验证前飞速度变化时舵效随 `|v| v_x_alpha` 正确变化。
6. 按悬停、小前飞、直线加速、过渡和圆轨迹逐级测试。
7. 最后才测试 knife-edge、差动转弯等复杂机动。

需要注意，当前 Tailsitter-control 仓库中的“完整模型”仍然是论文 alpha-theory 的低阶模型。源码注释已经说明 `alpha_0`、部分几何和电机参数属于估算值；它不是包含失速、动态失速、翼尖涡和 CFD 数据的全包线高保真模型。

#### 11.11.12 推荐方案的实现状态（2026-08-27）

已按 11.11.10 的并行方案完成第一版实现，保留现有 `PhoenixAero`、`phoenixdrone` 和低速控制入口不变。新增内容为：

```text
TailsitterAlphaAero                整机 Gazebo alpha-theory 插件
phoenixdrone_alpha                 独立模型，禁止电机插件重复施力
4023_gz_phoenixdrone_alpha        独立 PX4 SITL airframe
alpha_aerodynamics.py             Eq.5--14 控制器同构模型
alpha_allocator.py                Eq.37--40 速度相关分配
flatness_control.py               Eq.17--35 平坦性姿态和角速度前馈
alpha_controller_node.py          Eq.43、Eq.46 及完整滤波链
tailsitter_alpha_sitl.launch.py   独立启动入口
```

坐标链在插件和测试中固定为：

```text
[x_ts, y_ts, z_ts] = [z_gz, -y_gz, x_gz]
x_ts = -z_px4, y_ts = y_px4, z_ts = x_px4
```

当前需要重辨识或测量复核的参数及默认值为：

| 类别 | 参数 | 默认值 |
|---|---|---:|
| 质量/推进 | `mass`, `c_T`, `c_mu`, `c_mu_T` | `0.525`, `7.864e-6`, `1.80872e-7`, `0` |
| 气动角 | `alpha_0`, `alpha_T` | `-2 deg`, `0 deg` |
| 速度气动 | `c_LV`, `c_DV` | `0.29`, `0` |
| 滑流气动 | `c_LT`, `c_DT` | `2.23`, `0` |
| 舵效 | `c_LV_delta`, `c_LT_delta` | `0.18`, `1.25` |
| 几何 | `l_Ty`, `l_dy`, `l_dx` | `0.195`, `0.195`, `0.036 m` |
| 电机动态 | `tau_up`, `tau_down` | `0.016`, `0.020 s` |
| 舵机动态 | `tau_servo`, `servo_rate_max` | `0.03 s`, `25 rad/s` |

控制端默认值集中在 `PhoenixHoverConfig.alpha_identification_defaults`；仿真端对应值集中在 `phoenixdrone_alpha/model.sdf` 的 `TailsitterAlphaAero` 插件块。辨识时两处必须成对更新。当前 `c_T`、`c_mu`、质量和几何来自 PhoenixDrone 现有模型，其余气动系数主要采用 Tailsitter-control 的起始值，均不能视为当前 PhoenixDrone 的实测结果。

首轮验证结果：PX4/Gazebo 插件和 ROS 2 包构建通过；colcon 汇总为 `47 tests, 0 errors, 0 failures`；alpha 机型以 `SYS_AUTOSTART=4023` 正常启动；零输出门控、实际关节反馈和 250 Hz 模型调试话题正常；短时解锁地面测试没有发生快速翻转。该结果只确认结构、公式、坐标链和执行器链路可运行，下一阶段仍须按 11.11.11 的顺序完成气动参数辨识和各飞行包线验收。

#### 11.11.13 当前能否完成起飞、前飞和降落

目前还不能认为该机型能够可靠完成“垂直起飞 -> 姿态过渡 -> 前飞 -> 反向过渡 -> 降落”的完整自动任务。

完整移植解决的是 Gazebo 被控对象、控制器内部模型、控制分配、滤波链和平坦性公式的一致性，但当前实际验证只覆盖：

- alpha 模型和控制器可以正常启动；
- 执行器反馈、通道映射和坐标转换方向正确；
- 短时解锁没有发生由符号错误导致的快速翻转；
- 尚未完成自由飞行悬停、前飞配平、完整过渡和自动降落验收。

当前仍存在以下关键缺口：

1. `c_LV`、`c_LT`、`c_LT_delta`、`alpha_0` 等参数仍是辨识起始值，没有针对 PhoenixDrone 完成重新辨识。
2. 尚未验证垂直起飞后的高度闭环、悬停稳态误差和抗扰能力。
3. 尚未获得不同空速下的前飞配平姿态、总推力和左右舵偏。
4. 当前没有完整的起飞、过渡、巡航、反向过渡和降落任务状态机。
5. 降落阶段的接地检测、下降速度约束、推力收尾和异常中止逻辑尚未接入。
6. Tailsitter-control 使用的是低阶 alpha-theory 模型，不包含失速、动态失速等复杂效应，过渡区结果必须通过分阶段仿真重新验证。

从软件结构上看，`alpha_controller_node.py` 已经能够接收包含 position、velocity、acceleration、jerk、yaw 和 yawspeed 的连续 `TrajectorySetpoint`，因此具备实现完整任务的控制基础。但当前直接输入大幅前飞轨迹，仍可能产生推力饱和、舵面饱和、错误配平或失稳，不能作为安全测试入口。

推荐按以下顺序继续：

1. 完成垂直起飞至 `0.3--0.5 m`，验证高度闭环和稳定悬停。
2. 分别寻找 `0.5、1、2、4 m/s` 下的定速前飞配平点。
3. 从小于 `5 deg` 的姿态变化开始验证渐进式过渡。
4. 完成完整约 `90 deg` 前飞过渡以及反向过渡。
5. 加入带超时、限幅和异常中止的任务轨迹与落地状态机。
6. 最后执行短距离自动起飞、前飞和降落测试。

因此，当前结论是：完整任务所需的气动和控制框架已经具备，但系统仍处于参数辨识和分阶段飞行验证阶段，不能表述为已经能够可靠自动完成起飞、前飞和降落。
