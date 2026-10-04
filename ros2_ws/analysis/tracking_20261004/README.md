# 2026-10-04 tracking plots

New headless PX4/Gazebo SITL run using the current full alpha/INDI controller.
Reference: 5 m/s, 7 s per lap, 3 laps, 2 m takeoff height, 20 Hz reference;
nominal controller dt retained (`use_measured_control_dt=false`).

Only the `track` phase is plotted, selected using mission phase timestamps in
`/rosout`. Position and velocity are PX4 estimates, not Gazebo ground truth.
Position errors are actual minus reference in NED: positive Down error means
the aircraft is below the reference. Vertical display scale is exaggerated in
the 3D plot. North/East coordinates there are relative to the first tracking
reference; altitude is relative to the mission takeoff origin.

Attitude comes from the new `/phoenix_tailsitter/attitude_tracking_debug`
Float64MultiArray topic. Its eight fields are current TS-to-NED quaternion
`[w,x,y,z]`, followed by desired TS-to-NED quaternion `[w,x,y,z]`, from the same
controller cycle. It adds diagnostic output only. Euler decomposition is
intrinsic Z-X-Y (`Rz(yaw) Rx(roll) Ry(pitch)`), matching the flatness controller.
Plots show roll, pitch and unwrapped yaw in degrees. This convention avoids
the pitch=90 degree hover singularity of conventional Z-Y-X Euler angles.

`summary.json` reports high-rate debug metrics separately from the mission
node's 20 Hz metrics. Their sampling differs, so their RMS/peak values differ.
`position_tracking.csv`, `attitude_tracking.csv` and `tracking_samples.npz`
preserve the plotted samples locally; `bag/` preserves the original MCAP
recording locally. Raw recordings, sample exports and runtime logs are excluded
from Git. The GitHub version includes PNG/SVG plots, this report and the JSON
summary. Replotting requires the local recording or a new recording with the
same debug topics.

## 本次结果与验证

- 测试工况：5 m/s、每圈 7 s、连续三圈、目标高度 2 m。
- 高频 debug 三轴位置 RMSE（NED）：0.409 / 0.661 / 0.253 m。
- 高频 debug 三维位置 RMS / 峰值：0.817 / 2.121 m。
- 20 Hz 任务节点三维位置 RMS / 峰值：0.753 / 1.964 m。
- 速度峰值：5.462 m/s；三圈完整结束，未触发任务中止。
- 控制器仅增加同周期当前/目标姿态四元数的诊断发布，控制律未修改。
- 上传前复查：`test_control_timing.py`、`test_alpha_theory.py` 共 17 项通过，
  `git diff --check` 通过。

图中保留了高度偏低和姿态滞后；这些结果来自本次 SITL，不代表实机验收。

Replot from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
/usr/bin/python3 ros2_ws/analysis/plot_tracking_details.py \
  --bag ros2_ws/analysis/tracking_20261004/bag \
  --output-dir ros2_ws/analysis/tracking_20261004
```

This run finished takeoff, three tracking laps, exit, landing and disarm without
an abort. All processes started for this capture were stopped afterwards.
