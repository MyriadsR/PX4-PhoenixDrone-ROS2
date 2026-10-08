#!/usr/bin/env python3
"""Compare new SITL measurements of the eight original flight profiles."""
import argparse
import csv
import json
import os
from pathlib import Path
os.environ.setdefault('MPLCONFIGDIR', '/tmp/phoenix_suite_mpl')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

NAMES = ['segments', 'lemniscate', 'knife-edge-transition', 'circle-coordinated', 'circular-knife-edge', 'transition-to-forward', 'transition-to-hover', 'differential-turn']
LABELS = ['Segments', 'Lemniscate', 'Knife-edge transition', 'Coordinated circle', 'Knife-edge circle', 'Hover to forward', 'Forward to hover', 'Differential turn']


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', type=Path, required=True)
    args=parser.parse_args()
    folder=args.directory.resolve()
    manifest = (json.loads((folder/'manifest.json').read_text())
                if (folder/'manifest.json').exists() else {})
    rows=[]
    lap_rows=[]
    for name in NAMES:
        case=folder/name
        if not (case/'case_result.json').exists():
            continue
        result=json.loads((case/'case_result.json').read_text())
        mission=result.get('mission',{})
        tracking=result.get('tracking',{})
        if (case/'summary.json').exists():
            tracking=json.loads((case/'summary.json').read_text())
        log=(case/'mission.log').read_text(errors='replace') if (case/'mission.log').exists() else ''
        aborts=[line.split('LEMNISCATE_ABORT ',1)[-1] for line in log.splitlines() if 'LEMNISCATE_ABORT ' in line]
        abort_phase=''
        phase=''
        for line in log.splitlines():
            if 'LEMNISCATE_PHASE ' in line:
                phase=line.split('LEMNISCATE_PHASE ',1)[1]
            if 'LEMNISCATE_ABORT ' in line:
                abort_phase=phase
                break
        row={'trajectory':name,'status':result['status'], 'speed_m_s':mission.get('speed_m_s'),
             'reference_duration_s':mission.get('reference_duration_s'),
             'laps':mission.get('laps'), 'coverage_fraction':tracking.get('tracking_coverage_fraction'),
             'position_rmse_m':tracking.get('position_rmse_m'),
             'position_peak_m':tracking.get('position_peak_m'),
             'position_n_rmse_m':tracking.get('position_axis_rmse_m',[None]*3)[0],
             'position_e_rmse_m':tracking.get('position_axis_rmse_m',[None]*3)[1],
             'position_d_rmse_m':tracking.get('position_axis_rmse_m',[None]*3)[2],
             'velocity_magnitude_rmse_m_s':tracking.get('velocity_magnitude_rmse_m_s'),
             'velocity_vector_rmse_m_s':tracking.get('velocity_vector_rmse_m_s'),
             'attitude_rmse_deg':tracking.get('attitude_geodesic_rmse_deg'),
             'attitude_peak_deg':tracking.get('attitude_geodesic_peak_deg'),
             'abort_reason':'; '.join(aborts) or result.get('error',''),
             'abort_phase':abort_phase,
             'plot':str(case/'tracking_overview.png') if (case/'tracking_overview.png').exists() else None}
        if row['coverage_fraction'] is None and tracking:
            row['coverage_fraction']=min(1.,tracking['tracking_duration_s']/mission['reference_duration_s'])
        rows.append(row)
        samples=case/'tracking_samples.npz'
        if samples.exists() and mission.get('lap_time_s'):
            p=np.load(samples)['position_debug']
            for lap in range(int(mission['laps'])):
                mask=(p[:,0] >= lap*mission['lap_time_s']) & (p[:,0] < (lap+1)*mission['lap_time_s'])
                if np.any(mask):
                    error=p[mask,1:4]-p[mask,4:7]
                    lap_rows.append({'trajectory':name,'lap':lap+1,'position_rmse_m':float(np.sqrt(np.mean(np.sum(error**2,axis=1)))),
                                     'position_peak_m':float(np.max(np.linalg.norm(error,axis=1))),
                                     'mean_down_error_m':float(np.mean(error[:,2])),
                                     'recorded_time_span_s':float(np.ptp(p[mask,0]))})
    (folder/'comparison.json').write_text(json.dumps(rows,indent=2)+'\n')
    if rows:
        with (folder/'comparison.csv').open('w') as f:
            writer=csv.DictWriter(f,fieldnames=list(rows[0]))
            writer.writeheader();writer.writerows(rows)
    if lap_rows:
        with (folder/'lap_metrics.csv').open('w') as f:
            writer=csv.DictWriter(f,fieldnames=list(lap_rows[0]))
            writer.writeheader();writer.writerows(lap_rows)
    x=np.arange(len(rows))
    labels=[LABELS[NAMES.index(row['trajectory'])] + (
        f"\n({100*row['coverage_fraction']:.1f}% track)"
        if row['coverage_fraction'] is not None and row['coverage_fraction'] < .999 else '')
        for row in rows]
    colors=['#007F86' if row['status']=='completed' else '#C55F4F' for row in rows]
    fig,axes=plt.subplots(2,2,figsize=(15,9),layout='constrained')
    metrics=[('position_rmse_m','Position RMSE (m)'),('position_peak_m','Position peak error (m)'),('velocity_vector_rmse_m_s','Velocity vector RMSE (m/s)'),('attitude_rmse_deg','Quaternion attitude RMSE (deg)')]
    for axis,(key,title) in zip(axes.flat,metrics):
        vals=[row.get(key) if row.get(key) is not None else 0 for row in rows]
        axis.bar(x,vals,color=colors,width=.65)
        for i,row in enumerate(rows):
            value=row.get(key)
            text='No track data' if value is None else f'{value:.2f}'
            axis.text(i,vals[i],text,ha='center',va='bottom',fontsize=8,rotation=0 if value is not None else 90)
        axis.set_xticks(x,labels,rotation=28,ha='right',fontsize=8)
        axis.set_title(title,loc='left',weight='semibold')
        axis.spines[['top','right']].set_visible(False)
        axis.grid(axis='y',alpha=.2)
        axis.set_axisbelow(True)
        axis.set_ylim(0,max([1,*vals])*1.25)
    fig.suptitle('Original Tailsitter-control trajectories in PX4/Gazebo\nOriginal reference parameters | Green: completed; red: aborted/failed (partial coverage in labels)',fontsize=14)
    fig.savefig(folder/'comparison.png',dpi=180)
    fig.savefig(folder/'comparison.svg')
    plt.close(fig)
    def number(v):return '—' if v is None else f'{v:.3f}'
    text=['# 原项目全部轨迹在 PX4/Gazebo 中的测试','',
          '源项目：`/home/zr/Tailsitter-control`，提交 `4cc11be57c3327e8779e04ac1ff542d8c22ec0a7`。',
          '轨迹源文件 SHA256：`b2a2b64d5316a579ef5ae263c5b8d75af7568359f4c7a554b0cec97202831786`。', '',
          '本轮使用当前完整 alpha/INDI 控制器，原项目的默认速度、半径、时长和圈数保持不变。源轨迹采用 NED，所有空中轨迹参考高度为 10 m。仅添加固定世界位置平移，以便从当前 SITL 起飞点进入轨迹。正式跟踪阶段从原轨迹 t=0 开始，准备与退出阶段不计入误差。', '',
          '每条轨迹使用独立的全新无头仿真。起飞/降落参考速度为 0.4 m/s，空中轨迹准备/退出各 4 s，参考发布 20 Hz，控制器标称 500 Hz。分段轨迹包含原始 6 s 起飞段，从地面直接开始正式跟踪。', '',
          '表格为高频控制 debug 的统计，位置和速度来自 PX4 估计而非 Gazebo ground truth；姿态误差是当前四元数与实际控制目标四元数之间的最短旋转角。`完成`只描述任务完成，不代表严格的精度验收。中止时的误差只覆盖实际飞过的部分，不能作为整条轨迹的完整误差。', '',
          '| 轨迹 | 运行结果 | 参考速度 m/s | 圈数 | 跟踪覆盖 | 位置 RMS m | 峰值 m | 速度向量 RMS m/s | 姿态 RMS ° |',
          '|---|---|---:|---:|---:|---:|---:|---:|---:|']
    for row in rows:
        coverage='—' if row['coverage_fraction'] is None else f"{100*row['coverage_fraction']:.1f}%"
        state={'completed':'完成','aborted':'中止','mission_timeout':'超时','failed':'失败','no_mission_result':'无完整结果'}.get(row['status'],row['status'])
        text.append(f"| [{row['trajectory']}]({folder/row['trajectory']/'tracking_overview.png'}) | {state} | {number(row['speed_m_s'])} | {row['laps'] or '—'} | {coverage} | {number(row['position_rmse_m'])} | {number(row['position_peak_m'])} | {number(row['velocity_vector_rmse_m_s'])} | {number(row['attitude_rmse_deg'])} |")
    text.extend(['',f'![全部轨迹对比]({folder}/comparison.png)','',
                 '`circle-knife-edge` 是 `knife-edge-transition` 的兼容别名，不重复测试。原项目以空中配平状态初始化部分轨迹；本轮通过实际起飞和准备段入轨，因此入口误差与原项目理想动力学结果不直接等价。', '',
                 '每个子目录保留录包、任务和仿真日志、四类曲线 PNG/SVG、采样 CSV/NPZ 与 summary.json；comparison CSV/JSON 与逐圈 lap_metrics.csv 位于本目录。姿态角图使用原生 TS Z-X-Y 欧拉角，刀锋姿态可能接近欧拉角奇异点，精度评价使用无该奇异性的四元数误差。'])
    if manifest.get('data_selection') == 'latest_controller_fixes':
        text.extend(['',
            '本目录展示本轮控制修正全部启用后的八轨迹复测结果，覆盖原展示路径。参考连续推进、悬停/机动增益调度、实测控制周期、NED 模型力滤波、角速度前馈坐标转换均启用。图表未作额外平滑；中止图明确标注跟踪覆盖率。',
            f"旧基线结果保存在 `{manifest['baseline_suite']}`；最新原始数据位于 `{manifest['plot_source_suite']}`。各案例图与统计来自同一录包，路径映射见 run_selection.json。",
            f"详细修正及统一连续参考的新旧对照见 [控制改进报告]({Path(manifest['plot_source_suite']).parent/'README.md'})。本表统计实际状态减控制器参考，与连续参考对照报告的统计口径不同。"])
    elif (folder/'run_selection.json').exists():
        text.extend(['',
            '首轮 8 类测试保存于 `../paper_suite_20261004`，入轨/退出衔接修正后的 4 类复测保存于 `../paper_suite_20261004_retest`。本目录通过相对符号链接汇总最终采用的 8 条结果，完整路径映射见 run_selection.json。首轮数据保留。控制参数与正式轨迹参考在这两轮中保持一致。',
            '协调圆周首轮正式入轨时位置误差已达约 10.6 m，改为沿原曲线加速准备后成功完成；八字首轮完成全部圈数后在退出段失高，沿曲线减速退出的复测完成正常降落。三条未完成轨迹仍按实际覆盖率报告，不以短时片段或宽松的任务 passed 字段作为验收。'])
    text.extend(['', '## 各轨迹单图', '',
                 '| 轨迹 | 三轴位置误差 | 速度幅值 | 姿态角跟踪 | 三维位置 |',
                 '|---|---|---|---|---|'])
    for row in rows:
        links = []
        for image_name, label in [('position_errors', '位置误差'), ('speed_magnitude', '速度'),
                                  ('attitude_tracking', '姿态'), ('trajectory_3d', '三维')]:
            path = folder / row['trajectory'] / f'{image_name}.png'
            links.append(f'[{label}]({path})' if path.exists() else '—')
        text.append('| ' + row['trajectory'] + ' | ' + ' | '.join(links) + ' |')
    for row in rows:
        if row['abort_reason']:
            text.extend(['',f"- `{row['trajectory']}`：{row['abort_phase']} 阶段；{row['abort_reason']}"])
    text.extend(['','复现：','', '```bash','source /opt/ros/jazzy/setup.bash','source ros2_ws/install/setup.bash',
                 '/usr/bin/python3 ros2_ws/analysis/run_paper_trajectory_suite.py ' + chr(92),
                 '  --output-dir ros2_ws/analysis/paper_suite_new_run', '```'])
    (folder/'REPORT.md').write_text('\n'.join(text)+'\n')
    print(json.dumps(rows,indent=2))


if __name__=='__main__':main()
