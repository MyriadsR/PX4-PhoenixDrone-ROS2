#!/usr/bin/env python3
"""Document accepted, rejected and baseline SITL runs without smoothing samples."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'ros2_ws/analysis/maneuver_improvement_20261004'
NAMES = ('knife-edge-transition', 'differential-turn')


def main():
    metrics = json.loads((OUT / 'retained_comparison/metrics.json').read_text())
    regression = json.loads((OUT / 'regression_metrics.json').read_text())
    lines = ['# 两条机动轨迹的跟踪改进与回退验证', '',
             '刀刃过渡改善配置连续两次完成正式跟踪并正常落地；掉头未获得可重复的稳定改善，所有掉头调参已回退到原配置。', '',
             '只对 knife-edge-transition 和 differential-turn 调参；circular-knife-edge 本次未测试、未更新图。', '',
             '每次试验均重新启动 PX4/Gazebo。正式参考仍为原始 6 m/s、50 s、8 圈刀刃过渡和 7 m/s、3.3 s 掉头，参考发布 20 Hz。轨迹源码 SHA256 保持 b2a2b64d5316a579ef5ae263c5b8d75af7568359f4c7a554b0cec97202831786。未改变气动参数、质量/惯量、执行器物理限幅、终止条件或正式轨迹时标。', '',
             '## 相同运行环境的原配置对照', '',
             '位置指标以实际 PX4 位置减原始连续解析参考计算，并从录包重建固定世界位置平移。计分仅覆盖正式跟踪。共同窗口取原配置中止前的时间，整段指标另列；不同覆盖率的 RMS 不能直接比较。姿态误差使用实际姿态与同周期控制目标之间的四元数最短转角。', '',
             '| 轨迹 | 原配置覆盖 | 本次覆盖 | 共同窗口 s | 原配置位置 RMS m | 本次位置 RMS m（共同窗口） | 本次整段 RMS m | 已记录峰值 m |',
             '|---|---:|---:|---:|---:|---:|---:|---:|']
    for name in NAMES:
        b, n = metrics['baseline'][name], metrics['retained'][name]
        full = f'{n["position_rmse_continuous_m"]:.3f}' if n['coverage'] >= .999 else '—（未完成整段）'
        lines.append(f'| {name} | {100*b["coverage"]:.1f}% | {100*n["coverage"]:.1f}% | 0–{n["baseline_common_window_end_s"]:.3f} | {b["baseline_common_window_position_rmse_m"]:.3f} | {n["baseline_common_window_position_rmse_m"]:.3f} | {full} | {n["position_peak_continuous_m"]:.3f} |')
    lines += ['',
              '保留配置的路径见 retained/selection.json；刀刃指向第二次成功复测，掉头指向本轮原配置对照。完整覆盖、正常退出/落地与高精度跟踪分别评价，任务自带 RMS<10 m、峰值<25 m 的宽松 passed 判据不能当作高精度验收。', '',
              '## 保留的修改', '',
              '1. 起飞力补偿仅在 PX4 本地 NED 离地高度不超过 2 m 时允许工作。原逻辑在空中失高时也会把水平力归零，破坏机动所需的侧向力。',
              '2. 开启匹配滤波时，先用实际/预测的完整执行器状态计算模型力矩，再与角速度反馈一样经过一次 15 Hz 低通。撤回了仅在有限加速度阶段切换和按运动权重混合两套估计的试验。',
              '3. 刀刃过渡采用切线加速入轨，准备段之外的所有位置导数、偏航和正式时长保持原值；其机动角加速度、力矩软件限幅分别为原值 4 倍，随机动增益权重逐渐调整。实际电机、舵面和舵速仍受原物理边界限制。',
              '4. 掉头恢复原起飞补偿、力矩滤波、角加速度、力矩和外环限幅。原限幅配合力矩匹配曾两次完成，却在第三次失高；全轴放大、仅放大 TS-z 轴、外环限幅放大、前馈削弱、姿态速率治理和关闭前馈坐标转换也未得到稳定改善，全部撤回。', '',
              '标准 launch 默认仍为 maneuver_profile:=standard，起飞补偿范围和滤波路径保持修改前行为。只有刀刃有新的独立配置；自动测试脚本只对刀刃选择它，掉头仍运行原配置。knife_entry_straight 默认关闭，只有调参后的刀刃任务开启。', '',
              '## 独立重复验证', '',
              '| 配置与轨迹 | 跟踪覆盖 | 任务通过 | 控制 debug 位置 RMS m | 说明 |',
              '|---|---:|---|---:|---|']
    repeat_paths = [OUT/'knife_acceleration4/knife-edge-transition',
                    OUT/'selected/knife-edge-transition',
                    OUT/'entry_moment/differential-turn',
                    OUT/'verified/differential-turn', OUT/'selected/differential-turn',
                    OUT/'differential_z2/differential-turn']
    for folder in repeat_paths:
        c = json.loads((folder/'case_result.json').read_text())
        s = json.loads((folder/'summary.json').read_text())
        note = '刀刃保留配置复测' if folder.name == 'knife-edge-transition' else '掉头候选，最终撤回'
        lines.append(f'| {folder.parent.name}/{folder.name} | {100*s["tracking_coverage_fraction"]:.1f}% | {c["mission"]["passed"]} | {s["position_rmse_m"]:.3f} | {note} |')
    lines += ['',
              '掉头候选第三次在约 2.95 s 失高；仅放大 TS-z 轴的试验在约 2.00 s 中止。没有把前两次成功作为稳定性证明，也没有把短片段 RMS 当作全程指标。记录中存在接近 180° 的目标姿态分支切换以及执行器饱和；提高软件限幅和简单限制姿态变化均未解决。其参考在高速转向时仍需要继续处理正推力姿态分支与实际姿态/执行器可达性。原数值仿真从配平状态开始并同步迭代，本项目包含起飞、入轨、异步测量、执行器反馈和实际回调抖动。', '',
              '## 默认控制器回归', '',
              '| 轨迹 | 覆盖 | 正常完成/退出 | 此前位置 RMS m | 本次位置 RMS m |',
              '|---|---:|---|---:|---:|']
    for row in regression:
        lines.append(f'| {row["trajectory"]} | {100*row["coverage"]:.1f}% | {row["mission_passed"]} / {row["exit_code"]} | {row["baseline_controller_debug_position_rms_m"]:.3f} | {row["regression_controller_debug_position_rms_m"]:.3f} |')
    lines += ['',
              '这五条使用原默认参数，数值输出与修改前控制函数的精确比较通过；运行误差会随异步回调和初始化变化，回归表如实列出幅值差异，不声称每个统计量都改善。完整回归录包和图位于 final/ 中对应的五个目录。final/ 的两条目标轨迹则是被淘汰的滤波阶段切换试验，不是最终保留结果。', '',
              '## 所有试验与淘汰结果', '',
              '以下 RMS 为实际状态减控制 debug 参考的口径。未完成轨迹的 RMS 仅描述实际飞过的片段，不能用于全程优劣排序。所有原始数据保留，以避免只展示一次最好的运行。', '',
              '| 试验目录 | 轨迹 | 覆盖 | 任务通过 | 位置 RMS m | 选择 | 中止原因 |',
              '|---|---|---:|---|---:|---|---|']
    index = []
    for path in sorted(OUT.glob('*/*/case_result.json')):
        if path.parent.is_symlink():
            continue
        c = json.loads(path.read_text())
        s = c.get('tracking', {})
        mission = c.get('mission', {})
        group, name = path.parent.parent.name, path.parent.name
        chosen = group == 'selected' and name == 'knife-edge-transition'
        decision = ('保留' if chosen else '原配置对照' if group == 'fresh_baseline' else
                    '默认回归' if group == 'final' and name not in NAMES else
                    '重复验证' if group == 'knife_acceleration4' else '淘汰/撤回')
        coverage = s.get('tracking_coverage_fraction')
        rms = s.get('position_rmse_m')
        cv = '—' if coverage is None else f'{coverage*100:.1f}%'
        rv = '—' if rms is None else f'{rms:.3f}'
        lines.append(f'| {group} | {name} | {cv} | {mission.get("passed", False)} | {rv} | {decision} | {c.get("abort_reason", c.get("error", ""))} |')
        index.append(dict(experiment=group, trajectory=name, selected=chosen,
                          decision=decision, mission_passed=mission.get('passed'),
                          coverage=coverage, position_rms_debug_m=rms,
                          case_result=str(path.resolve()), source_snapshot=str(path.parent/'controller_source')))
    (OUT/'experiment_index.json').write_text(json.dumps(index, indent=2)+'\n')
    lines += ['', '## 结果图', '',
              '曲线来自原始 debug 样本，没有额外平滑。速度幅值同时显示参考值，三维图显示实际与参考位置。TS 姿态角采用原生 Z-X-Y 欧拉角；奇异点附近的角度跳变用四元数误差另外评价。', '',
              '| 轨迹 | 总图 | 三轴位置误差 | 速度幅值 | 姿态角跟踪 | 三维轨迹 |',
              '|---|---|---|---|---|---|']
    for name in NAMES:
        folder = OUT/'retained'/name
        links = [f'[{title}]({folder/(stem+".png")})' for stem,title in
                 [('tracking_overview','总图'),('position_errors','位置'),('speed_magnitude','速度'),
                  ('attitude_tracking','姿态'),('trajectory_3d','三维')]]
        lines.append('| '+name+' | '+' | '.join(links)+' |')
    for name in NAMES:
        lines += ['', f'![{name} 原配置对照]({OUT}/retained_comparison/{name}_comparison.png)', '']
    lines += ['', '## 验证与回退', '',
              '135 项包测试通过，colcon 构建通过，git diff --check 通过。四类默认控制函数各 500 次随机输入与保存的修改前版本逐元素精确相等；八条轨迹 808 个正式参考点与 328 个入轨点精确相等。新刀刃入轨另外检查首尾连续与正式参考导数不变。',
              '保存的 baseline_source/ 包含本次工作开始时用户已有的修改。restore_baseline.py 只恢复本次修改的文件，并在发现后续编辑时整批拒绝写入。已在临时工作目录验证完整恢复和拒绝覆盖两个分支，实际工作目录保留通过的修改。', '',
              '```bash',
              'source /opt/ros/jazzy/setup.bash', 'source ros2_ws/install/setup.bash',
              '/usr/bin/python3 ros2_ws/analysis/run_paper_trajectory_suite.py \\',
              '  --output-dir /tmp/phoenix_maneuvers_retest \\',
              '  --trajectories knife-edge-transition differential-turn',
              '# 加 --baseline-controller 可运行修改前控制器与入轨方式。',
              '# 回退预览；只有加 --apply 才会恢复源码。',
              '/usr/bin/python3 ros2_ws/analysis/maneuver_improvement_20261004/restore_baseline.py',
              '```', '',
              'validation.json、standard_equivalence.json、rollback_verification.json、regression_metrics.json、各次 case_result.json 以及 retained_comparison/metrics.json 是机器可读的验证依据。展示路径旧文件与链接保存在 display_before/；展示路径只覆盖刀刃过渡，其余轨迹保持原样。']
    (OUT/'README.md').write_text('\n'.join(lines)+'\n')
    print('Saved', OUT/'README.md')


if __name__ == '__main__':
    main()
