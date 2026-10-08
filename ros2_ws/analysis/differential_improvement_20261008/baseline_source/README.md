# Phoenix Tailsitter-control 低速移植

本包包含 `study.md` 第 11.7 节的低速控制路径，以及第 11.11 节新增的完整
alpha-theory 并行路径。原 `PhoenixAero`、`phoenixdrone` 和
`tailsitter_controller` 均保留；新路径使用 `TailsitterAlphaAero`、
`phoenixdrone_alpha` 和 `alpha_tailsitter_controller`，用于 A/B 对比和后续参数辨识。

## 坐标系唯一约定

控制器内部始终使用 Tailsitter-control 机体系：

```text
x_ts = -z_px4
y_ts =  y_px4
z_ts =  x_px4

v_px4 = [v_ts.z, v_ts.y, -v_ts.x]
v_ts  = [-v_px4.z, v_px4.y, v_px4.x]
```

该映射等价于将 Tailsitter-control 基向量绕 PX4 的 `+y` 轴旋转 `+90°`。
姿态使用 `R_ts_to_ned = R_px4_to_ned C_px4_ts`，惯量使用
`J_ts = C_ts_px4 J_px4 C_px4_ts`。转换只允许出现在 `frames.py`，控制律和
分配器中不得再添加坐标补丁或临时负号。

Gazebo 桥还会先做一次 FLU→FRD：`[x_px4,y_px4,z_px4]=[x_gz,-y_gz,-z_gz]`。
合并两次变换后，`[x_ts,y_ts,z_ts]=[z_gz,-y_gz,x_gz]`。所以 SDF 中
`left/right` 名称不能直接决定 PX4/tailsitter 的左右侧，这一点已经显式写入
分配器并由单元测试锁定。

## 安全顺序

1. 构建并运行纯数学测试：

   ```bash
   cd /home/zr/PX4-PhoenixDrone-ROS2/ros2_ws
   source /opt/ros/jazzy/setup.bash
   colcon build --packages-select px4_msgs phoenix_tailsitter_control
   source install/setup.bash
   colcon test --packages-select phoenix_tailsitter_control
   colcon test-result --verbose
   ```

2. 单独启动新 launch。不要同时启动 `phoenix_sitl.launch.py`，因为它会运行旧的
   `phoenix_controller` 和 `phoenix_position_controller`。默认会在运行时挂载关节状态
   发布系统并启动单向 Gazebo→ROS 桥，不会修改已有 SDF：

   ```bash
   ros2 launch phoenix_tailsitter_control tailsitter_sitl.launch.py output_enabled:=false
   ```

3. 在不解锁状态确认以下 topic 持续且无超时：

   ```bash
   ros2 topic hz /fmu/out/vehicle_attitude
   ros2 topic hz /fmu/out/vehicle_angular_velocity
   ros2 topic echo --once /fmu/in/offboard_control_mode
   ```

4. 固定机体后，以 `controller_enabled:=false` 启动 SITL，再只运行一个
   `actuator_probe`。探针不会自动解锁或切换 offboard，持续时间默认 3 秒，电机
   最大限制为 0.15，舵机最大限制为 0.25：

   ```bash
   ros2 launch phoenix_tailsitter_control tailsitter_sitl.launch.py \
     controller_enabled:=false

   # 在另一个终端启动探针；随后再由操作者切换 offboard 和解锁。
   ros2 run phoenix_tailsitter_control actuator_probe --ros-args \
     -p enabled:=true \
     -p confirmation:=PHOENIX_SINGLE_CHANNEL_TEST \
     -p kind:=motor -p index:=0 -p motor_idle:=0.05 -p value:=0.08
   ```

   依次验证 motor 0、motor 1、servo 0、servo 1。2026-08-26 的无头 SITL
   实测结果如下：

   | PX4 通道 | Gazebo 对象 | 实测结果 |
   |---|---|---|
   | motor 0 | `rotor_right_joint`，换算后为 TS/PX4 左侧 | `+0.08 -> velocity[0]=64 rad/s` |
   | motor 1 | `rotor_left_joint`，换算后为 TS/PX4 右侧 | `+0.08 -> velocity[1]=64 rad/s` |
   | servo 0 | right elevon (`servo_0`) | `+0.20 -> -0.10472 rad (-6°)` |
   | servo 1 | left elevon (`servo_1`) | `+0.20 -> -0.20944 rad (-12°)` |

   若实际观察与表格不同，先修改新包 `allocator.py` 的边界映射并重新测试，禁止在
   姿态控制律中补负号。

5. 标定一致后，在固定/系留状态启用闭环。默认 `setpoint_source=latch` 会锁存首次
   有效姿态，推力为 `0.525*9.81 N`。节点只在 PX4 已解锁、已进入 offboard 且
   `output_enabled=true` 时输出非零值：

   ```bash
   ros2 launch phoenix_tailsitter_control tailsitter_sitl.launch.py output_enabled:=true
   ```

   2026-08-26 的无头 SITL 短时验证中，两电机稳定在约 `0.69--0.74`，机体角速度
   保持在约 `0.04 rad/s` 以内，未出现坐标符号错误导致的快速翻转或发散。PX4
   随后按 `COM_DISARM_PRFLT` 自动解除武装；这只验证了姿态内环，不代表位置定高。

6. 先做 `±0.05 rad` 的单轴小阶跃并记录 rosbag。若要从独立 topic 给定参考，设置
   `setpoint_source:=topic`，发布 `VehicleAttitudeSetpoint` 到
   `/phoenix_tailsitter/attitude_setpoint`。`setpoint_frame=px4` 时，四元数和
   `thrust_body[2]` 遵循 PX4 约定；`setpoint_frame=tailsitter` 时，四元数使用控制
   框架机体系，正的 `thrust_body[0]` 是归一化总推力。

   本包提供带确认令牌的 `attitude_step_test`。它只发布参考，不会自动切换
   Offboard 或解锁；幅值硬限制为 `0.1 rad`，推力比例限制为 `0.8--1.2`：

   ```bash
   ros2 launch phoenix_tailsitter_control tailsitter_sitl.launch.py \
     output_enabled:=true setpoint_source:=topic setpoint_frame:=tailsitter

   ros2 run phoenix_tailsitter_control attitude_step_test --ros-args \
     -p confirmation:=PHOENIX_ATTITUDE_STEP_TEST \
     -p axis:=x -p amplitude_rad:=0.05 \
     -p baseline_s:=1.2 -p step_s:=1.5 -p recovery_s:=0.5 \
     -p hover_thrust_scale:=1.05
   ```

   ROS 2 命令行会把裸写的 `y` 按 YAML 解析为布尔值，因此 y 轴参数必须写为
   `-p "axis:='y'"`。测试器在观察到 armed+offboard 前持续跟随当前姿态，只在
   测试开始瞬间冻结基准，避免把上一次落地过程误判成交叉轴响应。

   `/phoenix_tailsitter/control_debug` 使用 `Float64MultiArray` 发布姿态误差、滤波
   角速度/角加速度、角加速度命令、估计/期望/分配力矩、执行器估计和输出门控。
   稳定字段顺序定义在 `debug.py` 的 `CONTROL_DEBUG_FIELDS`，应和姿态、角速度及
   执行器 topic 一起写入 rosbag。

   2026-08-26 在 `1.05*mg` 离地条件下的最终 `±0.05 rad` 结果如下。通过判据为
   本轴峰值 `<0.15 rad`、交叉轴峰值 `<0.10 rad`、角速度峰值 `<1 rad/s`，且
   正负稳态方向均正确：

   | TS 轴 | 正/负末段响应 (rad) | 本轴峰值 (rad) | 交叉轴峰值 (rad) | 角速度峰值 (rad/s) |
   |---|---:|---:|---:|---:|
   | x | `+0.0539 / -0.0553` | `0.0556` | `0.0081` | `0.100` |
   | y | `+0.0415 / -0.0304` | `0.0683` | `0.0338` | `0.147` |
   | z | `+0.0528 / -0.0572` | `0.0575` | `0.0077` | `0.109` |

   MCAP 识别得到 x/y/z 轴约 `20/30/60 ms` 的等效延迟。y 轴静态模型增益偏低，
   z 差动推力模型增益偏高，因此当前 `config.py` 的三轴限幅是本低速阶段的实测
   保守值，不能直接外推到过渡飞行。

## 位置与线加速度 INDI 阶段

以 `setpoint_source:=trajectory` 启动后，控制器订阅 PX4 NED
`/fmu/in/trajectory_setpoint` 和 `/fmu/out/vehicle_local_position`。position、velocity、
acceleration、jerk、yaw、yawspeed 均按 PX4 的 NaN 语义解析；世界系变量一直保留
为 NED，仅机体系增益、力估计和姿态环进入 TS 坐标。节点在输出门控、状态超时、
局部位置有效性和有限数检查全部通过后才输出 direct actuator。

当前 `PhoenixAero` 的舵面升力沿 `+z_ts`、二次阻力沿 `-x_ts`。控制器对舵面气动力
做低速增量补偿，并限制水平推进力变化率，使外环参考不超过已识别的姿态响应速度。
中间量发布到 `/phoenix_tailsitter/position_debug`；稳定字段顺序见 `debug.py` 的
`POSITION_DEBUG_FIELDS`。

安全阶跃测试器只发布参考，不会自动切换 Offboard、解锁或解除武装。幅值硬限制为
`0.15 m`，并在相对起点水平位移超过 `1 m` 或速度超过 `1.5 m/s` 时中止：

```bash
ros2 launch phoenix_tailsitter_control tailsitter_sitl.launch.py \
  output_enabled:=true setpoint_source:=trajectory

ros2 run phoenix_tailsitter_control position_step_test --ros-args \
  -p confirmation:=PHOENIX_POSITION_STEP_TEST \
  -p axis:=x -p amplitude_m:=0.05 -p takeoff_height_m:=0.20
```

2026-08-26 最终无头 SITL 的 NED x `±0.05 m` 结果如下：

- 受控轴跨度 `0.357 m`，交叉轴跨度 `0.876 m`，速度峰值 `0.297 m/s`；
- 正/负末段响应为 `+0.0997/-0.0097 m`；
- 内环最大 TS 姿态误差约 `[0.0112, 0.0167, 0.0075] rad`，最大角速度约
  `[0.0177, 0.0582, 0.0088] rad/s`，未发生翻转或姿态发散；
- NED x/y/z 的期望力到实测加速度峰值相关系数约为
  `0.917/0.620/0.966`，对应采样延迟约 `0.50/0.48/0.10 s`。

该结果确认了坐标方向、力轴和完整数据链路，但未通过本包的小位置阶跃判据，主要
剩余问题是当前简化气动/执行器模型下的 NED East 慢振荡。不要把这一结果表述为
“位置闭环已验收”；下一阶段应加入实际执行器反馈或更准确的执行器/气动模型，再
重新识别 East 通道并复测，而不是继续放宽判据。

## 执行器实际反馈阶段

第 11.7 节第 8 步已加入 Gazebo 实际执行器状态。当前 PX4 DDS 配置没有输出真实
电机转速或舵面位置，因此 `tailsitter_sitl.launch.py` 在仿真启动后动态挂载
`JointStatePublisher`，再由 `ros_gz_bridge` 把下列话题转换为
`sensor_msgs/msg/JointState`：

```text
/world/stars_ts/model/phoenixdrone_0/joint_state
```

这个运行时操作没有修改 PX4、Gazebo 模型或已有 launch。当前单模型 `stars_ts`
世界中的 `phoenixdrone_0` 实体 ID 为 10；如果世界内实体构成发生变化，必须先通过
`/world/stars_ts/state` 重新确认 ID，禁止盲目把插件挂到其他实体。

反馈进入控制器前执行以下确定性换算：Gazebo 旋翼关节速度乘
`rotor_velocity_slowdown=10` 恢复物理转速并取绝对值；经过 FLU→FRD 和本文固定的
TS 坐标映射后，TS `[left,right]` 电机对应
`[rotor_right_joint,rotor_left_joint]`，舵面对应
`[left_elevon_joint,right_elevon_joint]`。该映射由单元测试锁定。

启动参数 `actuator_feedback_mode` 支持：

- `auto`（默认）：50 ms 内反馈有效时使用实测值，超时后退回一阶预测；
- `estimate`：始终使用上一周期目标和电机时间常数构成的一阶预测；
- `required`：只允许实测反馈，反馈超时会令 `output_active=false` 并输出零值。

`feedback_enabled:=false` 可关闭运行时插件和桥。预测值、实测值、两者误差、反馈
年龄及激活状态发布在 `/phoenix_tailsitter/actuator_feedback_debug`，字段顺序见
`debug.py` 的 `ACTUATOR_FEEDBACK_DEBUG_FIELDS`。

2026-08-26 无头 SITL 动态悬停验证得到 1742 个电机运行样本：反馈完整率和激活率
均为 100%，反馈年龄 99 分位为 `1.99 ms`、最大为 `4.19 ms`；左右电机实测相对
一阶预测的 RMSE 为 `3.77/3.80 rad/s`，左右舵面 RMSE 为
`0.00044/0.00035 rad`。这确认了实际反馈的时序、缩放和左右映射，但尚未重新验收
上一节未通过的 NED East 位置通道；下一阶段应使用本反馈链路重新识别并复测位置环。

## 完整 alpha-theory 并行路径

新路径逐项移植 Tailsitter-control 的 Eq.5--14、Eq.17--35、Eq.37--40、
Eq.43 和 Eq.46：

- 一个整机 Gazebo 插件同时计算推力、机翼速度项、舵面力、偏心力矩和反扭矩；
- `MulticopterMotorModel` 只保留电机转速动态，其力和力矩系数置零，避免重复施力；
- Gazebo 插件内的舵面采用一阶动态和角速度限制；
- Python 控制模型与 Gazebo 插件采用同一组公式，控制分配随当前机体系速度变化；
- 线加速度、角速度、执行器反馈使用 15 Hz 二阶低通，舵面额外使用 1 Hz 二阶高通；
- 轨迹模式使用完整微分平坦性姿态/推力解算及 jerk/yawspeed 角速度前馈。

构建和默认零输出启动命令为：

```bash
cd /home/zr/PX4-PhoenixDrone-ROS2/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select px4_msgs phoenix_tailsitter_control --symlink-install
source install/setup.bash
ros2 launch phoenix_tailsitter_control tailsitter_alpha_sitl.launch.py \
  output_enabled:=false
```

辨识参数的控制端默认值集中在 `config.py` 的
`alpha_identification_defaults`，仿真端对应值位于
`models/phoenixdrone_alpha/model.sdf` 的 `TailsitterAlphaAero` 插件块。两处必须成对
更新，禁止只修改控制器或只修改 Gazebo。2026-08-27 首轮默认值如下（历史记录，
当前模型参数以 `config.py` 和对应 SDF 为准）：

| 参数 | 默认值 | 默认值来源/处理 |
|---|---:|---|
| `mass` | `0.525 kg` | 当前 PhoenixDrone 质量，称重复核 |
| `c_T` | `7.864e-6` | 当前电机模型，拉力台重辨识 |
| `c_mu` | `1.80872e-7` | 暂取 `0.023*c_T`，扭矩台重辨识 |
| `c_mu_T` | `0` | 待辨识 |
| `alpha_0` | `-2 deg` | 原项目起始值，待辨识 |
| `alpha_T` | `0 deg` | 当前几何起始值，待测量/辨识 |
| `c_LV, c_DV` | `0.29, 0` | 原项目起始值，待风洞/飞行辨识 |
| `c_LT, c_DT` | `2.23, 0` | 原项目起始值，待辨识 |
| `c_LV_delta, c_LT_delta` | `0.18, 1.25` | 原项目起始值，待辨识 |
| `l_Ty, l_dy, l_dx` | `0.195, 0.195, 0.036 m` | 当前 SDF 几何，实机测量复核 |
| 电机上/下时间常数 | `0.016/0.020 s` | 当前电机模型，阶跃重辨识 |
| 舵机时间常数/速率 | `0.03 s, 25 rad/s` | 原项目起始值，阶跃重辨识 |

用于辨识的模型中间量发布在
`/phoenix_tailsitter/alpha_model_debug`，字段顺序由 `debug.py` 的
`ALPHA_MODEL_DEBUG_FIELDS` 固定，包括 TS 机体系速度、alpha 系合力、TS 系合力矩、
瞬态舵力以及电机/舵面的 LPF、HPF 状态。

2026-08-27 的验证结果：PX4 和 Gazebo 插件构建通过；47 项 colcon 测试无失败；
`phoenixdrone_alpha` 以 `SYS_AUTOSTART=4023` 启动；零输出时执行器保持零；alpha 调试
话题约 250 Hz；短时解锁地面测试中真实旋翼反馈达到约 `549/595 rad/s`，三轴 PX4
角速度峰值低于 `8e-4 rad/s`，没有出现坐标或通道符号错误造成的快速翻转。该测试
只验证首轮模型链路和静态方向，不代表这些默认气动参数已经辨识，也不代表过渡飞行
已经验收。

## 2026-10-04 轨迹参考与振荡修正

alpha 路径默认启用 `trajectory_prediction_enabled`、
`motion_gain_scheduling_enabled`、`use_measured_control_dt`、
`world_force_filter_enabled` 和 `transport_feedforward_rates_enabled`。参考发布仍为
20 Hz；控制器依据同钟消息时间戳，用 p/v/a/jerk 多项式以及 yaw/yawspeed
推进到当前控制时刻，最大推进 0.10 s。时间戳缺失或来自另一时钟域时，使用本地
接收时刻；原始 NaN 控制 mask 保留，位置保持与加速度单独控制的语义保持一致。

机动权重由请求的速度、加速度和偏航角速度连续决定，有限的零加速度不再自动
启用高机动增益。位置环、姿态环和反馈加速度边界使用同一个缓变权重。
NaN 加速度的起飞/降落准备使用原低速参数；有限静止轨迹使用独立的悬停参数，
以保留从运动中制动的能力。具体数值见 `config.py` 的
`trajectory_hover_*`、`maneuver_*`。实测控制周期用于滤波、差分与增益过渡，
避免将非实时 ROS 回调固定解释为 2 ms。

线加速度 INDI 使用完整瞬时模型力：用选定的实际/预测执行器状态和当前速度计算，
旋转到 NED 后再低通，截止和 dt 与测量加速度一致。瞬态舵力在相同世界系独立
滤波；在稳态力与加速度中同时扣除，保持总力平衡。旧路径仅滤波执行器输入、
再使用当前姿态旋转，会在旋转运动中产生额外相位误差。

平坦性角速度前馈属于名义参考姿态的 TS 机体系，现在通过
`R_current.T @ R_reference @ omega_reference` 转到当前 TS 机体系，再与测量角速度
比较。姿态误差反馈仍追踪位置闭环修正后的目标姿态。

`reference_debug` 的八个值依次为预测年龄 s、发布/接收是否同钟、是否启用预测、
运动权重目标、已应用权重、是否启用运动调度、是否启用世界系模型力滤波、
是否启用角速度前馈坐标转换。完整回归还记录 `timing_debug`、原始角速度、
`actuator_feedback_debug` 和原始关节反馈。五个开关可独立设为 `false`，用于
控制变量对照；alpha 路径力/加速度滤波截止仍为统一 15 Hz。

分段轨迹自身包含地面起飞，因此 `paper_trajectory_mission` 在确认 armed/offboard
后直接开始它的 t=0，避免在地面零运动参考下额外等待。原轨迹源码快照、正式
时长、速度、圈数与固定位置平移均保留。对照用同一连续解析参考计分，避免把
改变误差采样方式误判为真实改善。记录、源码快照与报告位于
`ros2_ws/analysis/tracking_improvement_20261004/`。其中 `final/` 是增益/周期修正的
中间候选；`rate_transport/` 记录上述五项全部启用的最新配置。

## 刀刃过渡与掉头的独立 SITL 配置

`tailsitter_alpha_sitl.launch.py` 新增 `maneuver_profile`，默认 `standard` 保留
本次修改前的控制器行为。可选的改善配置为 `knife-edge-transition`；
刀刃配置限制起飞补偿的工作高度，并匹配角加速度 INDI 两侧的力矩滤波。
刀刃过渡另外按机动权重提高角加速度/力矩软件限幅至原值的 4/4 倍；
执行器物理边界、悬停增益、气动参数和轨迹正式速度/时长保持原值。
掉头候选在第三次复测中失稳，因此已恢复原配置，未保留默认调参入口。

刀刃过渡任务需要同时设置 `knife_entry_straight:=true`，在正式计分前沿起点
切线加速，避免先进入上一圈的过弯姿态。该参数默认关闭，原入轨方式可复现。
正式 50 秒、8 圈的原始轨迹以及所有导数保持不变，仅世界位置平移随准备路径变化。

自动测试脚本只为刀刃过渡选择改善配置，掉头及其他轨迹使用 `standard`：

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
python3 ros2_ws/analysis/run_paper_trajectory_suite.py \
  --output-dir /tmp/phoenix_maneuver_retest \
  --trajectories knife-edge-transition differential-turn
```

加 `--baseline-controller` 可直接使用修改前的控制器和准备方式进行对照。
逐次试验的源码快照、原始 rosbag、四类结果图和回退备份位于
`ros2_ws/analysis/maneuver_improvement_20261004/`。以该目录中的报告为准，
完成整段轨迹与达到高精度跟踪分别评价；未通过的配置不作为默认配置。

## 本阶段停止条件

只有在四个单通道映射、三个小角度阶跃方向、状态超时归零、NaN/限幅和静态误差
都通过后，才进入完整气动和过渡飞行阶段。低速姿态阶段已在 2026-08-26 的无头
SITL 中通过；执行器实测反馈已接入并完成动态验证；位置环、线加速度 INDI 和
`TrajectorySetpoint` 已实现并完成方向与链路验证，但水平小阶跃验收仍未通过。
完整 alpha-theory 并行路径已经实现并完成零输出及短时地面链路验证。进入过渡飞行
前仍需按上述参数表完成 PhoenixDrone 重辨识，并依次通过悬停、小前飞、直线加速和
过渡轨迹验收；不能把原项目的默认气动系数当作当前机体的已辨识结果。
