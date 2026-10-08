#!/usr/bin/env python3
"""Derive branch diagnostics and write the Chinese report from offline runs."""
import json
import numpy as np
import matplotlib.pyplot as plt
from diagnose import OUT, ORIGINAL, CONFIG_ORIGINAL, sim, rotation, hashes

metrics=json.loads((OUT/'metrics.json').read_text())
nominal=json.loads((OUT/'nominal_metrics.json').read_text())
cfg=CONFIG_ORIGINAL()
flat=sim.FlatnessAttitudeController(cfg)
d=np.load(OUT/'baseline/flight_log.npz')
r=np.load(OUT/'baseline/diagnostics.npz')
t=d['time']
raw_thrust=[]; reconstruction_errors=[]
for i,ti in enumerate(t):
    f=r['force_command'][i];v=d['velocity'][i];ds=r['flap_sum'][i];psi=d['psi_ref'][i]
    qc,total,phi,th=flat.compute_desired_attitude_and_thrust(f,v,ds,psi,r['q_actual'][i],return_angles=True)
    reconstruction_errors.append((rotation(qc).inv()*rotation(r['q_command'][i])).magnitude())
    cp,sp=np.cos(psi),np.sin(psi);cf,sf=np.cos(phi),np.sin(phi)
    R=np.array([[1,0,0],[0,cf,sf],[0,-sf,cf]])@np.array([[cp,sp,0],[-sp,cp,0],[0,0,1]])
    fx,_,fz=R@f;vx,_,vz=R@v;n=np.linalg.norm(v)
    eta=(flat.s_a_bar*(cfg.c_LT-1)-flat.c_a_bar*cfg.c_LT_delta*ds/2)/flat.c_a_bar
    Y=eta*fx-cfg.c_LV_delta*ds*n*vx-cfg.c_LV*n*vz-fz
    X=eta*fz-cfg.c_LV_delta*ds*n*vz+cfg.c_LV*n*vx+fx
    angle=np.arctan2(Y,X)
    raw_thrust.append((np.cos(angle)*fx-np.sin(angle)*fz)/flat.c_a_bar)
raw_thrust=np.array(raw_thrust)
step=np.rad2deg((rotation(r['q_command'][:-1]).inv()*rotation(r['q_command'][1:])).magnitude())
jumps=np.where(step>90)[0]+1
assert max(reconstruction_errors)<1.e-8
branch=[dict(time_s=float(t[i]),step_deg=float(step[i-1]),
    raw_thrust_before_n=float(raw_thrust[i-1]),raw_thrust_after_n=float(raw_thrust[i])) for i in jumps]
n=np.load(OUT/'nominal_baseline.npz')
i=np.argmax(np.abs(n['moment'][:,1]))
vb=rotation(n['q'][i]).inv().apply(n['velocity'][i])
va=np.cos(cfg.alpha_0)*vb[0]+np.sin(cfg.alpha_0)*vb[2]
T=n['thrust'][i];A=cfg.c_LT_delta*np.cos(cfg.alpha_0+cfg.alpha_T)
B=cfg.c_LV_delta*np.linalg.norm(vb)*va
Tmax=cfg.c_T*cfg.omega_max**2
# Bound all motor splits and both flaps at this fixed nominal attitude/total thrust.
# This deliberately relaxes roll/yaw/force constraints; it is an optimistic bound.
pitch_bound=max(cfg.l_dx*(abs(-A*t1-B)+abs(-A*(T-t1)-B))+abs(cfg.c_mu_T*T)
                for t1 in (max(0,T-Tmax),min(T,Tmax)))
turn=(t>=.9)&(t<=2.5)
derived=dict(command_jumps=branch,
    command_reconstruction_max_error_rad=float(max(reconstruction_errors)),
    maneuver_only_position_rms_m=float(np.sqrt(np.mean(np.sum((d['position'][turn]-d['position_ref'][turn])**2,axis=1)))),
    nominal_pitch_torque=dict(time_s=float(n['time'][i]),required_abs_nm=float(abs(n['moment'][i,1])),
                              optimistic_capacity_nm=float(pitch_bound),nominal_total_thrust_n=float(T)),
    source_unchanged=hashes()==json.loads((OUT/'source_manifest.json').read_text()))
assert derived['source_unchanged']
(OUT/'derived_metrics.json').write_text(json.dumps(derived,indent=2)+'\n')
fig,ax=plt.subplots(3,1,figsize=(11,8),sharex=True,layout='constrained')
ax[0].plot(t,raw_thrust,label='Thrust before positive-thrust guard')
ax[0].plot(t,r['total_thrust_command'],'--',label='Thrust after guard')
ax[0].axhline(0,color='k',lw=.7);ax[0].set(ylabel='Thrust (N)',title='Sign crossings trigger theta += pi')
ax[1].plot(t[1:],step,label='Command quaternion step')
ax[1].set(ylabel='Rotation / sample (deg)',title='Physical orientation discontinuity, not Euler wrapping')
ax[2].plot(t,np.rad2deg(d['angular_velocity_ref'][:,1]),label='Pitch feedforward body rate')
ax[2].plot(t,np.rad2deg(d['angular_velocity'][:,1]),label='Actual pitch body rate')
ax[2].set(ylabel='Body rate (deg/s)',xlabel='Time (s)')
for a in ax:
    a.set_xlim(1.05,1.38);a.grid(alpha=.25);a.legend(fontsize=9)
    for ti in t[jumps]:a.axvline(ti,color='red',ls=':',alpha=.5)
fig.suptitle('Original numerical baseline: branch switches at 1.204 s and 1.292 s')
for ext in ('png','svg'):fig.savefig(OUT/f'branch_switches.{ext}',dpi=160)
plt.close(fig)

names={'baseline':'原始 7 m/s、500 Hz','rate_2000':'只提高到 2000 Hz',
       'instant_motors':'只取消电机延迟（诊断）','instant_servos':'只取消舵机延迟（诊断）',
       'instant_actuators':'同时取消执行器延迟（诊断）',
       'rate_transport':'只修正参考角速度的坐标系变换',
       'centered_yaw':'偏航 0.52 s 翻转移至 1.44–1.96 s（改变参考）',
       'slow_yaw':'偏航延长至 1.6 s（改变参考）','half_speed':'速度降至 3.5 m/s（改变参考）'}
rows=[]
for mode,m in metrics.items():
    rows.append(f"| {names[mode]} | {m['rms_position_error']:.3f} | {m['max_position_error']:.3f} | "
       f"{m['actual_speed_at_reference_stop']:.3f} | {m['maximum_feedforward_body_rate_deg_s']:.0f} | "
       f"{m['max_command_step_deg']:.2f} | {m['turn_motor_bound_fraction']*100:.1f}% / {m['turn_flap_bound_fraction']*100:.1f}% |")
report=f'''# differential-turn 跟踪失败诊断，2026-10-08

结论：主要问题是当前重构轨迹的偏航与速度反向时序，在平坦性反解后产生了超出执行器能力的姿态路径；闭环又触发正推力分支翻转，使姿态指令发生接近 180° 的跳变。提高频率或单独取消执行器延迟不能解决。当前数值模型可以完整运行并在后段恢复反向飞行，但不能准确跟踪转弯期间的速度、姿态及位置。此前把“能够跑完整段”表述为“能够有效跟踪”不够准确。

本次只新增离线诊断脚本、日志和图。没有修改原数值项目、其已有日志或 ROS2 控制代码；`source_unchanged.json` 校验了 1219 个 Python 源文件及原有 validation_logs。数值项目关键九个源文件与 2026-10-04 对比清单逐项一致，新基线也复现原 RMS=0.755126 m，所以没有证据表明是这几天代码退化。

## 新鲜基线：能跑完，不能准确停车和转向

- 原始条件：500 Hz，7 m/s，0.9 s 进入段 + 1.6 s 速度反向 + 0.8 s 退出段；初始位置、速度和配平均直接置于参考状态，无风。
- 全部日志约 3.3 s：位置误差 RMS **0.755 m**，峰值 **1.412 m**。仅 0.9–2.5 s 机动段 RMS **0.916 m**。前面 0.9 s 几乎零误差会拉低全程 RMS。
- 1.70 s 参考速度恰好为零，实际速度幅值 **2.841 m/s**，实际速度向量为 `[2.474, 0.883, 1.082] m/s`；它已经偏离参考直线并产生垂向速度。全程最低速度仍为 2.803 m/s。
- 姿态相对控制指令的最大四元数夹角 **133.47°**；参考机体角速度最大 **2666.85°/s**，实际最大 **856.41°/s**。
- 机动段中，至少一个电机命令触及边界的时间占 **17.7%**，至少一个舵命令触及边界占 **17.0%**。这里统计命令而非经过一阶动态的实际舵角，实际舵没有到边界不能说明命令没饱和。

![新鲜基线]({OUT}/baseline_tracking.png)

## 为什么名义 650°/s 实际要求 2600°/s

轨迹定义位于 [{ORIGINAL}/trajectory/test.py]({ORIGINAL}/trajectory/test.py:668)。位置反向安排在 0.90–2.50 s，而偏航翻转安排在 0.90–1.42 s。1.42 s 偏航已经由 90° 变为 270°，但参考速度仍沿正 Y 方向以 **4.750 m/s** 前进；速度到 1.70 s 才过零。

平坦性变换还必须满足给定的加速度、速度及竖直支撑力，因此此时的机动包含额外的快速俯仰，而不是只有偏航旋转。固定近似初始配平舵偏和 `delta_sum=-0.1107 rad`、用连续上一参考姿态选择分支，所得机体角速度峰值 **2607.57°/s**，其中机体 Y 轴分量 **44.18 rad/s**。用相邻四元数反求角速度，和公式结果的 RMS 差仅 **0.00025 rad/s**，证明这是实际姿态路径的快速转动，并非欧拉角表象或角速度公式偶然计算错。

沿上述固定配平舵假设得到的姿态路径，1.183 s 所需俯仰力矩约 **4.18 N·m**；同一姿态、速度和总推力 5.79 N 下，把左右电机推力分配和两个舵偏都放到最有利边界，俯仰力矩绝对值的乐观上界仍只有 **0.683 N·m**。这个上界甚至放松了保持滚转、偏航和合力的约束。因此这条名义姿态路径无法按时实现。该条件性检查没有证明所有可变舵偏、所有控制器下原位置/偏航轨迹都绝对不可实现。

项目内[论文文字]({ORIGINAL}/paper/full.md:776)描述的是飞行实验达到约 650°/s、半秒反向等结果；它没有给出本代码使用的 1.6 s 七次速度反向与 0.52 s 五次偏航参考的组合。把实验测得的最大机体转速等同于重构轨迹的偏航参考峰值，不能保证复现原实验的完整姿态与执行器需求。

![参考姿态负担]({OUT}/reference_demand.png)

## 接近 180° 的跳变来自哪个分支

[{ORIGINAL}/controller/att_ctrl.py:186]({ORIGINAL}/controller/att_ctrl.py:186) 在反解总推力为负时，直接执行 `theta_bar += pi`。本次完整记录了当前状态、指令合力、速度、舵偏和偏航，并重新计算每一步反解，得到的四元数与运行日志一致。

| 时刻 | 保护前推力变化 | 相邻指令的四元数转角 |
|---|---|---|
| 1.204 s | +0.332 → -0.042 N | {branch[0]['step_deg']:.2f}° |
| 1.292 s | -0.368 → +0.098 N | {branch[1]['step_deg']:.2f}° |

两次都发生在推力解穿越零点时。参考角速度由连续角度求导，不能包含这类分支瞬时跳跃，导致姿态指令与角速度前馈不再连续一致。不能简单删除正推力保护：否则会要求电机输出负推力。应通过可实现的参考规划与分支约束避免进入这种状态。

![分支跳变证据]({OUT}/branch_switches.png)

## 9 组离线对照

全部采用原数值模型，无阵风，无生产代码修改。变化只在当前 Python 进程包装器中生效并在运行结束后恢复；完整保存每组原始日志。参考变化和瞬时执行器均只用于判断因果，未作为改善方案合入。

| 试验 | 全程位置 RMS / m | 峰值 / m | 1.70 s 实际速度 / m/s | 前馈机体转速峰值 / °/s | 最大指令跳变 / ° | 机动段电机 / 舵饱和占比 |
|---|---:|---:|---:|---:|---:|---:|
{chr(10).join(rows)}

2000 Hz 和 500 Hz 的 RMS 仅相差约 0.001 m，所以主要原因不是积分步长。同时取消执行器延迟仍有 2.190 m/s 停车残速和近 180° 指令跳变；单独加快电机或舵机反而更差，表明延迟与闭环相位有关，但延迟不是唯一原因。

最有辨识力的实验是**只把 0.52 s 偏航窗口中心移到速度过零点**：保持全部原位置、速度、加速度和 jerk 样本，仍保持 649°/s 偏航参考峰值。机体前馈峰值降到 603°/s，最大指令步变降到 2.26°，两类执行器饱和都消失。RMS 只有约 6% 改善，参考停车时仍有 1.456 m/s 残速，说明消除最严重姿态问题后，位置/速度跟踪仍需进一步改善。延长偏航至 1.6 s 的 RMS 为 0.653 m；降速到 3.5 m/s 为 0.252 m，但两者改变了机动要求，不能当作原 7 m/s 快速掉头已解决。

原数值 PD 还把参考姿态机体系的角速度直接与实际机体系测量相减。在大姿态误差下，这会引入额外偏差；单独做坐标系输运后的 RMS 反而为 2.739 m。需要和分支、前馈及控制分配共同处理，不能将这一行变换作为已验证改善。

![对照实验]({OUT}/ablation_comparison.png)

## 为什么 SITL 更容易失控

纯数值仿真已有实际电机/舵机边界和动态，并不等于姿态瞬时完美执行。它从精确参考速度和配平开始，且没有 ROS2 中的内环软件角加速度/力矩限幅；当前基线的角加速度命令峰值约 `[165,478,147] rad/s²`，力矩命令峰值 `[2.182,1.478,1.483] N·m`。软件限幅会进一步阻止追上快速俯仰路径，实际入段误差也会更早改变推力分支。

此前[同源离线对照](/home/zr/PX4-PhoenixDrone-ROS2/ros2_ws/analysis/numerical_sitl_comparison_20261004/metrics.json)中，叠加 SITL 原角加速度/力矩限幅以及旧起飞力保护后，纯数值 RMS 达 7.488 m、峰值 15.339 m。它说明附加控制约束会明显放大原有问题；不能据此认定所有 SITL 差异已经解释。本次没有启动新的 SITL 飞行。

建议后续顺序：先在独立试验配置中联合规划速度反向与偏航翻转，并对反解后的姿态连续性、角速度、力矩及电机/舵余量做检查；再优化连续分支与前馈的一致性；随后调跟踪带宽、INDI 滤波和执行器延迟。对 7 m/s、0.52 s 翻转保持原要求的实验，应优先验证“居中过零”的窗口，不应仅放大增益或切掉负推力保护。数值改善达到完整位置、速度和姿态指标后，再做 SITL 复测及其他轨迹回归。

复现：
```bash
OPENBLAS_NUM_THREADS=1 MPLBACKEND=Agg /usr/bin/python3 {OUT}/diagnose.py
OPENBLAS_NUM_THREADS=1 MPLBACKEND=Agg /usr/bin/python3 {OUT}/write_report.py
```
'''
(OUT/'README.md').write_text(report)
print('Report and branch figure written; original/production source hashes verified.')
