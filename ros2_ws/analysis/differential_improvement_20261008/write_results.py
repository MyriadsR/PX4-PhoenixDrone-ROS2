#!/usr/bin/env python3
"""Write the retained/rejected experiment report from verified records."""
from pathlib import Path
import json
ROOT=Path(__file__).resolve().parent
numeric=json.loads((ROOT/'numerical_metrics.json').read_text())
sitl=json.loads((ROOT/'sitl_metrics.json').read_text())
knife=json.loads((ROOT/'knife_regression_metrics.json').read_text())
retained=('sitl_gated125','sitl_gated_repeat1','sitl_gated_repeat2','sitl_integrated')
for name in retained:
    m=sitl[name]
    assert m['mission_passed'] and m['coverage']>.999
    assert m['position_rmse_continuous_m']<.7 and m['position_peak_continuous_m']<1.
selection={'selected_case':str((ROOT/'sitl_integrated/differential-turn').resolve()),
 'reference_variant':'centered-yaw','retained_runs':list(retained),
 'rejected':{'sitl_inner150_limits16':'position RMS 4.215 m, peak 11.179 m despite mission passed',
             'sitl_inner125_limits8':'ungated bandwidth failed landing in its repeat',
             'sitl_inner125_repeat1':'height protection during landing'},
 'standard_defaults_preserved':True,'original_reference_snapshot_preserved':True,
 'original_numerical_project_preserved':True}
(ROOT/'selection.json').write_text(json.dumps(selection,indent=2)+'\n')
rows=[]
for name,m in sitl.items():
    verdict=('原配置对照，机动中止' if name=='original' else
             '保留配置的独立验证' if name in retained else
             '较保守候选，不作为最终配置' if name=='sitl_centered4' else '淘汰')
    rows.append(f"| {name} | {m['coverage']*100:.1f}% | {'是' if m['mission_passed'] else '否'} | "
       f"{m['position_rmse_continuous_m']:.3f} | {m['position_peak_continuous_m']:.3f} | "
       f"{m['common_window_position_rms_m']:.3f} | {verdict} |")
nrows=[]
for name,m in numeric.items():
    nrows.append(f"| {name} | {m['rms_position_error']:.3f} | {m['max_position_error']:.3f} | "
                f"{m['actual_speed_at_reference_stop']:.3f} | {m['max_command_step_deg']:.2f} |")
selected=sitl['sitl_integrated'];num=numeric['centered_inner125_limits8']
improvement=(1-selected['common_window_position_rms_m']/sitl['original']['common_window_position_rms_m'])*100
report=f'''# differential-turn 改善结果，2026-10-08

保留方案已在 4 个全新的 PX4/Gazebo 仿真中完成 3.3 s、7 m/s 掉头，随后正常退出和降落。
连续解析参考下，全程位置 RMS 为 0.341–0.367 m，峰值 0.631–0.803 m。
最终接入后的验证为 RMS **{selected['position_rmse_continuous_m']:.3f} m**、峰值 **{selected['position_peak_continuous_m']:.3f} m**。
与原配置共同的 0–2.049 s 窗口相比，位置 RMS 从 1.460 m 降到 {selected['common_window_position_rms_m']:.3f} m，降低约 **{improvement:.1f}%**。
原配置在 2.049 s 中止，不能把它的部分 RMS 与新方案整段 RMS 当成同一时间窗口。

本次共 18 组数值候选、8 次掉头 SITL、1 次刀刃回归。数值项目源码与原有日志不修改，
ROS2 方案以独立 profile 接入，默认 standard 和原始掉头参考仍可复现。

## 保留的实际改动

1. 位置、速度、加速度、jerk 和速度反向时长全部保持原值，只把 0.52 s 偏航窗口从
   0.90–1.42 s 移到 **1.44–1.96 s**，围绕 1.70 s 的速度过零点居中。进入速度仍为 7 m/s，
   偏航参考峰值仍约 649°/s。原参考源码快照保持原 SHA256；变体单独放在 `trajectory_variants.py`。
2. 额外姿态带宽为 1.25 倍，即机动姿态增益乘 1.25²、角速度增益乘 1.25。
   增强控制器只用于有明确加速度参考的阶段；普通保持和降落使用原控制器。
   默认 bandwidth=1.0 时直接复用原控制器。
3. 保留已验证的离地 2 m 起飞力保护范围和单次匹配力矩低通。
   机动角加速度/力矩**软件**限幅按运动权重提高到 8/8 倍；电机转速、舵偏、舵速、
   电机/舵机时间常数以及气动参数保持原值，物理分配仍执行边界约束。

控制器 launch profile 为 `differential-turn`，任务参数为 `differential_yaw_centered:=true`。
自动测试脚本同时选择两者。直接调用 `make_reference('differential-turn')` 仍返回原偏航轨迹；
`differential_yaw_centered` 默认 False，launch 默认仍为 `standard`。

## SITL 全部对照

| 试验 | 正式轨迹覆盖 | 任务 passed | 位置 RMS / m | 峰值 / m | 共同窗口 RMS / m | 采用情况 |
|---|---:|---|---:|---:|---:|---|
{chr(10).join(rows)}

1.5 倍带宽、16 倍软件限幅在纯数值模型中最好，但在 SITL 中 RMS 达 4.215 m、峰值 11.179 m，
因此淘汰。它的任务 `passed` 字段虽然为 True，不能代表精度合格。
最初的全阶段 1.25 倍带宽第一次完成，复测却在降落中触发高度保护，也没有保留。
改为只在明确加速度参考下选择增强控制器后，上表四次验证均完成并降落。
这个对照支持阶段作用范围的调整，但不等于已经证明所有降落误差的唯一原因。

![SITL 对照]({ROOT}/comparison/sitl_comparison.png)

## 最新四类曲线

图中位置和速度参考按连续解析函数推进，实际 PX4 状态不作平滑；原始 debug、rosbag、CSV/NPZ 保持完整。
姿态角比较实际姿态与控制器目标，采用 TS 原生 ZXY 顺序；欧拉角接近奇异点时要结合四元数误差评价。

| 三轴位置误差 | 速度幅值 | 姿态角跟踪 | 三维位置 |
|---|---|---|---|
| [位置]({ROOT}/selected/differential-turn/position_errors.png) | [速度]({ROOT}/selected/differential-turn/speed_magnitude.png) | [姿态]({ROOT}/selected/differential-turn/attitude_tracking.png) | [三维]({ROOT}/selected/differential-turn/trajectory_3d.png) |

![掉头结果总图]({ROOT}/selected/differential-turn/tracking_overview.png)

这仍不是零误差跟踪：最终运行在参考停车的 1.70 s，实际速度幅值还有
**{selected['actual_speed_at_stop_m_s']:.3f} m/s**；全程速度向量 RMS 为
**{selected['velocity_vector_rmse_continuous_m_s']:.3f} m/s**。
0.10–3.28 s 窗口中，实际姿态相对控制目标的四元数误差 RMS 为
**{selected['window_attitude_error_rms_deg']:.2f}°**。后续可进一步优化速度过零附近的滞后。

## 纯数值仿真

与保留配置相对应的数值包装试验为 `centered_inner125_limits8`：全程位置 RMS
**{num['rms_position_error']:.3f} m**，峰值 **{num['max_position_error']:.3f} m**，参考停车时实际速度
**{num['actual_speed_at_reference_stop']:.3f} m/s**。原数值基线对应为 0.755 m、1.412 m、2.841 m/s。
这些包装仅在当前进程修改参考/参数，没有改写 `/home/zr/Tailsitter-control` 的默认仿真。

| 数值候选 | 全程位置 RMS / m | 峰值 / m | 参考停车时实际速度 / m/s | 最大相邻指令转角 / ° |
|---|---:|---:|---:|---:|
{chr(10).join(nrows)}

新增角加速度前馈没有稳定改善，未进入 ROS2 保留方案。进一步放大增益反而产生饱和甚至分支跳变，
所以没有将纯数值模型中的最佳增益直接作为 SITL 最终参数。

![数值对照]({ROOT}/comparison/numerical_comparison.png)

## 不破坏现有代码的验证与回退

- 137 个单元测试通过，包含偏航变体的导数连续性、入轨/退出衔接及名义姿态路径检查。
- 与本次开始时备份比较，默认控制器的角加速度、力矩、位置及起飞保护输出各 500 组随机输入完全相等；
  八条默认参考 808 个样本、入轨 328 个样本完全相等。刀刃切线入轨和正式参考另做逐样本相等检查。
- 刀刃 fresh SITL 回归完成 50 s、8 圈并正常降落，连续位置 RMS
  **{knife['position_rmse_continuous_m']:.3f} m**、峰值 **{knife['position_peak_continuous_m']:.3f} m**，
  与此前保留结果约 1.009 m / 2.108 m 一致。
- 原数值项目源码与已有验证日志校验未变；ROS2 的物理 config、原始参考文件及其余运行模块未修改。
- 当前展示目录 `paper_suite_20261004_final` 只更换掉头入口和图，其他七条的图与入口保持原样。
  原掉头入口和旧汇总保存在本目录 `display_before/`。

源码改动限定在 rollback_manifest.json 的七个文件；备份在 baseline_source/。
`restore_before.py` 默认只预览，`--apply` 才恢复；会先校验全部文件，发现后续用户编辑即拒绝覆盖。
已在临时副本验证完整恢复，以及后续编辑的拒绝保护。

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
/usr/bin/python3 ros2_ws/analysis/run_paper_trajectory_suite.py \\
  --output-dir /tmp/differential_turn_retest --trajectories differential-turn
# 原偏航和控制配置对照：再加 --baseline-controller
# 回退预览：
/usr/bin/python3 {ROOT}/restore_before.py
```

原始诊断见 [问题定位报告]({ROOT.parent}/differential_diagnosis_20261008/README.md)。
数值候选可通过 numerical_candidates.py 重放；18 组结果、原始日志和诊断数组均保存在 numerical/。
'''
(ROOT/'README.md').write_text(report)
print('Retained/rejected report written; four completed runs verified.')
