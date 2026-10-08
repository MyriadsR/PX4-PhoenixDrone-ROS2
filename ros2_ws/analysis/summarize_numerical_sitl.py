#!/usr/bin/env python3
"""Produce comparison figures and diagnostics from offline runs and SITL logs."""
import json
import os
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

os.environ.setdefault("MPLBACKEND", "Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "ros2_ws/analysis/numerical_sitl_comparison_20261004"
LATEST = ROOT / "ros2_ws/analysis/tracking_improvement_20261004"
NAMES = ["knife-edge-transition", "circular-knife-edge", "differential-turn"]
TITLES = ["刀刃 / 协调飞行切换", "圆形刀刃飞行", "差动推力转弯"]
sys.path.insert(0, "/home/zr/Tailsitter-control")
from trajectory.test import KnifeEdgeTransitionTrajectory, CircularTrajectory, DifferentialThrustTurnTrajectory


def summarize_model_inertia():
    path = ROOT / "Tools/simulation/gz/models/phoenixdrone_alpha/model.sdf"
    # Strip source comments containing equation ranges before strict XML parsing.
    source = re.sub(r"<!--.*?-->", "", path.read_text(), flags=re.S)
    model = ET.fromstring(source).find("model")
    links = []
    for link in model.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        pose = np.fromstring(link.findtext("pose", "0 0 0 0 0 0"), sep=" ")
        ip = np.fromstring(inertial.findtext("pose", "0 0 0 0 0 0"), sep=" ")
        r = Rotation.from_euler("xyz", pose[3:]).as_matrix()
        ri = r @ Rotation.from_euler("xyz", ip[3:]).as_matrix()
        position = pose[:3] + r @ ip[:3]
        mass = float(inertial.findtext("mass"))
        e = inertial.find("inertia")
        j = np.array([[float(e.findtext(f"i{a}{b}" if a <= b else f"i{b}{a}", "0"))
                       for b in "xyz"] for a in "xyz"])
        links.append((link.attrib["name"], mass, position, ri @ j @ ri.T))
    mass = sum(link[1] for link in links)
    com = sum(m * p for _, m, p, _ in links) / mass
    j = sum(ji + m * (np.dot(p - com, p - com) * np.eye(3) - np.outer(p - com, p - com))
            for _, m, p, ji in links)
    rotation = np.array([[0, 0, 1], [0, -1, 0], [1, 0, 0]])
    j_ts = rotation @ j @ rotation.T
    nominal = np.array([.0095, .003, .0115])
    result = {
        "links": [{"name": n, "mass_kg": m, "com_gz_m": p.tolist(),
                   "inertia_diag_gz_kgm2": np.diag(ji).tolist()} for n, m, p, ji in links],
        "total_mass_kg": mass, "com_gz_model_m": com.tolist(),
        "base_link_com_gz_model_m": links[0][2].tolist(),
        "force_application_offset_from_total_com_ts_m": (rotation @ (links[0][2] - com)).tolist(),
        "zero_joint_angle_locked_inertia_ts_kgm2": j_ts.tolist(),
        "nominal_inertia_ts_kgm2": np.diag(nominal).tolist(),
        "zero_joint_angle_locked_inertia_error_percent": ((np.diag(j_ts) / nominal - 1) * 100).tolist(),
        "caveat": "Locked-joint composite inertia at zero rotor/elevon angle; freely rotating joints have additional dynamics and angle-dependent transverse inertia.",
    }
    (OUT / "model_inertia.json").write_text(json.dumps(result, indent=2) + "\n")


def main():
    summarize_model_inertia()
    plt.rcParams.update({"font.family": "Noto Sans CJK JP", "axes.unicode_minus": False,
                         "font.size": 10, "svg.fonttype": "none"})
    numerical = json.loads((OUT / "metrics.json").read_text())
    sitl_metrics = json.loads((LATEST / "selected_comparison/metrics.json").read_text())["rate_transport"]
    trajectories = [KnifeEdgeTransitionTrajectory(), CircularTrajectory(mode="knife_edge"),
                    DifferentialThrustTurnTrajectory()]
    diagnostics = {}
    fig, axes = plt.subplots(3, 3, figsize=(17.5, 11.5), constrained_layout=True)
    for row, (name, title, trajectory) in enumerate(zip(NAMES, TITLES, trajectories)):
        summary = json.loads((LATEST / "rate_transport" / name / "summary.json").read_text())
        z = np.load(LATEST / "rate_transport" / name / "tracking_samples.npz")
        p, att = z["position_debug"], z["attitude_debug"]
        cfg_translation = np.array(sitl_metrics[name]["reconstructed_reference_translation_ned_m"])
        reference = np.array([trajectory.sample(t)[0] + cfg_translation for t in p[:, 0]])
        errors = np.linalg.norm(p[:, 1:4] - reference, axis=1)
        qa = Rotation.from_quat(att[:, 1:5][:, [1, 2, 3, 0]])
        qc = Rotation.from_quat(att[:, 5:9][:, [1, 2, 3, 0]])
        angle = np.rad2deg((qa.inv() * qc).magnitude())
        # Full masks are supplied by these three paper missions. In this log
        # their vertical reference velocity/acceleration are zero throughout.
        climbing = ((p[:, 6] < p[:, 3] - .05) | (p[:, 12] < -.05) | (p[:, 18] < -.2))
        floor = climbing & (np.maximum(0., -p[:, 24]) < .8 * .7 * 9.81)
        floor_zero = np.linalg.norm(p[:, 25:27], axis=1) < 1e-8
        raw_indi_force = p[:, 22:25] + .7 * (p[:, 19:22] - p[:, 13:16])
        extra = np.load(LATEST / "selected_comparison" / f"rate_transport_{name}_extra.npz")
        control = extra["control"]
        duration = summary["tracking_duration_s"]
        diagnostics[name] = {
            "tracking_duration_s": duration,
            "reference_duration_s": summary["mission"]["reference_duration_s"],
            "coverage": summary["tracking_coverage_fraction"],
            "position_rmse_continuous_m": sitl_metrics[name]["position_rmse_continuous_m"],
            "initial_position_error_continuous_m": float(errors[0]),
            "initial_position_error_controller_reference_m": float(np.linalg.norm(p[0, 1:4] - p[0, 4:7])),
            "initial_velocity_error_m_s": float(np.linalg.norm(p[0, 7:10] - p[0, 10:13])),
            "initial_attitude_error_to_command_deg": float(angle[0]),
            "initial_body_rate_lpf_rad_s": control[0, 4:7].tolist(),
            "initial_actual_euler_zxy_deg": qa[0].as_euler("ZXY", degrees=True).tolist(),
            "force_floor_sample_fraction": float(np.mean(floor)),
            "force_floor_first_05s_sample_fraction": float(np.mean(floor[p[:, 0] < .5])),
            "force_floor_zero_horizontal_match_fraction": float(np.mean(floor_zero[floor])) if floor.any() else None,
            "force_floor_first_trigger_s": float(p[floor, 0][0]) if floor.any() else None,
            "force_floor_raw_horizontal_force_peak_n": float(np.linalg.norm(raw_indi_force[floor, :2], axis=1).max()) if floor.any() else 0.,
            "angular_acceleration_saturation_fraction_xyz": sitl_metrics[name]["angular_acceleration_saturation_fraction"],
            "control_dt_mean_ms": sitl_metrics[name]["control_dt_mean_ms"],
            "control_dt_max_ms": sitl_metrics[name]["control_dt_max_ms"],
        }
        if floor.any():
            k = int(np.searchsorted(att[:, 0], p[floor, 0][0]))
            if 0 < k < len(att):
                diagnostics[name].update({
                    "force_floor_first_target_attitude_step_deg": float(np.rad2deg((qc[k - 1].inv() * qc[k]).magnitude())),
                    "force_floor_first_actual_attitude_step_deg": float(np.rad2deg((qa[k - 1].inv() * qa[k]).magnitude())),
                    "force_floor_first_attitude_error_before_after_deg": angle[k - 1:k + 1].tolist(),
                    "force_floor_first_attitude_step_interval_s": float(att[k, 0] - att[k - 1, 0]),
                })
        left, mid, right = axes[row]
        for mode, label, color in [
            ("baseline", "原数值模型", "#13896d"),
            ("alpha_limits", "数值模型 + 角加速度限幅", "#8b63a8"),
            ("both_limits_force_floor", "数值模型 + 双限幅 + 起飞保护", "#347bac"),
        ]:
            data = np.load(OUT / mode / name / "flight_log.npz")
            e = np.linalg.norm(data["position"] - data["position_ref"], axis=1)
            mask = data["time"] <= duration
            left.plot(data["time"][mask], e[mask], color=color, lw=1.5, label=label)
        left.plot(p[:, 0], errors, color="#d15d3a", lw=1.7, label="当前 SITL")
        left.set(title=title + "：相同时间窗", ylabel="位置误差范数 / m", xlabel="轨迹时间 / s", xlim=(0, duration))
        if row == 0:
            left.legend(fontsize=8, loc="upper left")

        original = np.load(OUT / "baseline" / name / "flight_log.npz")
        raw = Rotation.from_euler("ZXY", original["euler_deg"], degrees=True)
        cmd = Rotation.from_euler("ZXY", original["euler_command_deg"], degrees=True)
        numerical_angle = np.rad2deg((raw.inv() * cmd).magnitude())
        common = original["time"] <= duration
        mid.plot(original["time"][common], numerical_angle[common], color="#13896d", lw=1.5, label="原数值模型")
        mid.plot(att[:, 0], angle, color="#d15d3a", lw=1.6, label="当前 SITL")
        mid.fill_between(p[:, 0], 0, 180, where=floor, color="#e1a526", alpha=.18, label="空中触发起飞保护")
        mid.set(title="实际姿态与控制指令的旋转角误差", ylabel="姿态误差 / °", xlabel="轨迹时间 / s", ylim=(0, 180), xlim=(0, duration))
        if row == 0:
            mid.legend(fontsize=8, loc="upper left")

        acceleration = original["angular_acceleration_command"]
        for k, color in enumerate(("#3274a1", "#e1812c", "#3a923a")):
            right.plot(original["time"], acceleration[:, k] / np.array([20., 15., 24.])[k],
                       color=color, lw=.9, label=f"TS {'xyz'[k]} 轴")
        right.axhline(1, color="black", ls="--", lw=1)
        right.axhline(-1, color="black", ls="--", lw=1)
        right.set(title="原数值模型的角加速度指令（全程）", ylabel="指令 / 当前对应轴限值", xlabel="轨迹时间 / s", xlim=(0, original["time"][-1]))
        if row == 0:
            right.legend(fontsize=8, loc="upper right")
        for ax in (left, mid, right):
            ax.grid(alpha=.2)
    fig.suptitle("原数值模型与当前 SITL：三条失败轨迹的差异\n左、中列截取相同时间窗；右列覆盖数值轨迹全程；曲线未平滑", fontsize=17)
    fig.savefig(OUT / "failure_diagnosis.png", dpi=190)
    fig.savefig(OUT / "failure_diagnosis.svg")
    plt.close(fig)
    (OUT / "sitl_diagnostics.json").write_text(json.dumps(diagnostics, ensure_ascii=False, indent=2) + "\n")

    modes = ["baseline", "alpha_limits", "moment_limits", "both_limits", "extra_moment_lpf", "force_floor", "both_limits_force_floor"]
    labels = ["原数值模型", "+角加速度限幅", "+力矩限幅", "+双限幅", "+额外力矩低通", "+起飞保护逻辑", "+双限幅与起飞保护"]
    lines = ["# 数值模型与 SITL 三条失败轨迹对照", "", "2026-10-04：原项目当前源码重新运行；SITL 使用当天 rate_transport 批次的录包。所有实验只在独立进程里包装控制器方法，不修改原项目/生产控制器，不启动新的 SITL。", "", "## 核心结论", "", "三条失败轨迹的原因并不相同。刀刃切换对额外角加速度限幅高度敏感，已通过单变量离线对照复现。差动转弯的限幅与空中误触发起飞保护共同扩大误差。圆形刀刃原数值模型本身也有明显高度/位置漂移；它在 SITL 中的进一步退化尚未通过实时单变量实验完全隔离。", "", "## 相同时间窗比较", "", "下表三列均截取各轨迹从 t=0 到 SITL 中止时刻，避免把完整数值轨迹与残缺 SITL 的 RMS 直接比较。", "", "|轨迹|时间窗 / s|原数值模型 RMS / m|数值模型加入双限幅与起飞保护 / m|当前 SITL RMS / m|SITL 覆盖率|", "|---|---:|---:|---:|---:|---:|"]
    for name in NAMES:
        d = diagnostics[name]
        lines.append(f"|{name}|0–{d['tracking_duration_s']:.3f}|{numerical['baseline'][name]['common_window_position_rmse_m']:.3f}|{numerical['both_limits_force_floor'][name]['common_window_position_rmse_m']:.3f}|{d['position_rmse_continuous_m']:.3f}|{100*d['coverage']:.1f}%|")
    lines += ["", "## 数值模型单变量对照（完整轨迹）", "", "每种条件都保持原来的 500 Hz 同步闭环、配平初值、电机 40 ms、舵机 30 ms/25 rad/s、转速/舵偏约束及气动模型。双限幅 = α=[20,15,24] rad/s²，M=[0.25,0.08,0.35] N·m。起飞保护使用当前项目同一函数。", "", "|条件|刀刃切换 RMS / m|圆形刀刃 RMS / m|差动转弯 RMS / m|", "|---|---:|---:|---:|"]
    for mode, label in zip(modes, labels):
        lines.append("|" + label + "|" + "|".join(f"{numerical[mode][name]['rms_position_error']:.3f}" for name in NAMES) + "|")
    lines += ["", "这些是原模型内部的因果对照，不能当成 Gazebo 的预测结果。失败实验仍按原 main_sim 运行完整程，保留原有简化地面约束；后期几十米 RMS 的具体数值不代表可实现飞行。数值大小不具可加性。", "", "## 逐条分析", "", "### knife-edge-transition", "", "- 原模型全程 RMS=0.506 m，峰值=1.258 m。只加角加速度限幅，RMS=25.960 m；只加力矩限幅为 0.815 m。角加速度限幅是已经证明的主要差异。", "- 原控制指令三轴峰值约 [250.2,244.7,144.5] rad/s²，而当前限值为 [20,15,24]。原实际角加速度也明显超过当前指令限值，证明不仅是分配器丢弃了大指令。", "- SITL 进入计分轨迹第一帧的姿态相对控制目标误差为 86.0°，第一帧相对控制参考的位置误差约 0.98 m（相对同一解析时间点约 1.26 m）。数值模型则直接以目标位置、速度、姿态、角速度及预装载执行器启动。当前 entry 仅保证参考曲线导数相接，没有等待实际姿态/速度/执行器收敛后才计分。", "", "### circular-knife-edge", "", "- 原数值模型虽运行完 10.86 s，但 RMS=2.941 m、峰值=5.112 m；位置各轴 RMS 约 [0.896,0.891,2.656] m，主要是高度漂移。因此不能把它称为高精度成功跟踪。", "- 原数值模型三轴角加速度指令峰值仅约 [5.07,5.23,2.35] rad/s²。独立加角加速度/力矩限幅均没有改变数值结果，限幅不能解释该轨迹最初的退化。", "- SITL 第一帧相对控制参考的位置误差约 1.34 m（同一解析时间点约 1.54 m）、速度向量误差约 1.43 m/s；姿态相对含反馈的控制目标只差 3.6°，但这不等于已达到数值模型的配平姿态。实际俯仰约 15.3°，原数值配平俯仰约 36.1°；当前初始滤波角速度含约 1.01 rad/s 的俯仰角速度，原初值该轴约为 0。", "- 起飞保护约从 t=0.754 s 开始在空中触发，清零水平力；首次触发时目标姿态在约 8.65 ms 内跳变 64.4°，实际姿态只变化约 2.8°，误差从 10.2° 跳到 74.2°。之后俯仰角加速度限幅占约 63%。后期饱和是闭环进入大误差后的现象，不能倒推出圆形刀刃在理想配平附近必然需要这些大指令。", "- 额外退化与非配平入口、空中起飞保护、状态/执行器时序以及模型差异一致；各自贡献仍需 Gazebo 单变量实验验证。", "", "### differential-turn", "", "- 原数值模型 RMS=0.755 m。只加角加速度限幅变为 1.288 m；只加起飞保护变为 2.654 m；双限幅与保护共同加入变为 7.488 m。", "- 翻转期间原角加速度指令峰值约 [165,478,147] rad/s²，原实际约 [136,150,61] rad/s²；当前限值显著压低姿态机动能力。参考 yaw 在 0.52 s 内转 π，峰值约 649°/s；完整 TS 前馈角速度峰值约 46.5 rad/s，不能只按 yaw rate 判断需求。", "- 初始姿态误差仅约 0.5°，问题主要发生在快速翻转而非第一帧初始化。SITL 约从 t=1.414 s 开始空中触发起飞保护，随后水平力被取消；首次触发时目标姿态约 6.19 ms 内跳变 177.4°，实际姿态只变化约 3.7°。该机动在此之前已存在大姿态误差，保护逻辑进一步改变了控制目标。", "", "## 共同代码差异及证据", "", "1. **空中误触发起飞保护。** `apply_takeoff_force_floor` 只检查爬升需求和估计向上力是否低于 0.8mg，没有起飞阶段/离地高度门槛。轨迹掉高时同样满足条件，函数第 286 行将水平力清零。alpha_controller_node 在所有轨迹控制里调用它。三条失败轨迹中的触发采样比例分别为 21.5%、30.8%、23.8%，与水平力清零一致率均为 100%。这是直接日志和代码证据。", "2. **控制器额外限制。** 原项目只在执行器分配和动态上限制物理量；当前在 PD 和 INDI 之间还先截断角加速度、再截断力矩，位置/速度反馈也额外限幅。相同高机动参考不等于相同闭环。", "3. **初始化不同。** 原 main_sim 先迭代配平，再直接初始化 p/v/q/ω、电机、舵面及滤波器。SITL 从地面起飞并用 4 s 时间缩放入口接入；当前记录显示刀刃入口明显未收敛。", "4. **测量时序不同。** 原模型所有状态和执行器同步、500 Hz、固定 2 ms。当前 PX4 EKF 的位置/速度/加速度、姿态、角速度及 Gazebo 关节反馈异步到达。三条失败轨迹控制回调平均间隔约 5.13、5.73、4.64 ms，且存在 12.63、14.94、22.37 ms 的最大间隔。状态年龄只是消息到达年龄，不能据此推断 EKF 的完整估计延迟。参考仍是同一解析轨迹，20 Hz 发布后已做时间预测。", "5. **角 INDI 额外模型力矩低通。** 当前先低通电机/舵面计算模型力矩，再对结果额外低通；原模型省略后一个低通。独立加入该滤波后 RMS 分别变为 0.580、2.944、0.829 m，说明它有影响，但离线证据不支持将它作为三条失效的单一主因。", "6. **Gazebo 多刚体与单刚体差异。** 数值模型 J 为 diag[0.0095,0.0030,0.0115]。SDF 在 base_link 上已赋相同 J 的对应轴，却又加入有质量/惯量的 IMU、旋转电机和舵面 link。关节零角度、锁定时汇总 J_TS≈diag[0.010227,0.003468,0.012017]，比 nominal 高约 [7.7%,15.6%,4.5%]；总 COM 比施力所用 base_link COM 沿 TS-x 偏约 1.59 mm。这只是几何汇总值，自由旋转的多刚体还存在角度相关惯量与关节动力学，不能直接拿汇总值替代 INDI J。", "7. **两套电机状态。** 气动插件接收电机指令后用自己的电机状态算推力，MulticopterMotorModel 同时维护用于关节反馈的另一套电机状态；两者名义 τ 都是 40 ms，但积分与采样路径不同。这是并行两套状态，不是串联两个 40 ms 延迟。应以同一实际状态计算气动力及提供控制反馈。", "", "## 下一步改善顺序", "", "1. 将起飞保护作用域限制在地面启动/建立升力阶段；空中掉高保留所需水平加速度。先分别重复三条失败轨迹。", "2. 刀刃切换增加真实状态的入口收敛判据，核对姿态分支，并将角加速度约束按已验证的执行器能力和机动需求调度。数值实验不能证明可以直接在 Gazebo 去掉所有限幅。", "3. 差动转弯核对前馈角速度与姿态分支连续性；在相同原参考下逐项验证限幅。若实际执行器能力确实不足，再明确采用时间伸缩并作为另一套轨迹评分。", "4. 圆形刀刃先以原数值配平状态启动 Gazebo 作对照，再统一气动物理/反馈电机状态、校核总惯量/总 COM 和角 INDI 两侧延迟。", "5. 继续同时记录完成率、共同时间窗误差、完整轨迹误差和饱和比例；取消中止逻辑本身不会改善跟踪。", "", "## 文件与复现", "", "- `failure_diagnosis.png/.svg`：相同时间窗位置/姿态误差与原数值模型角加速度需求。", "- `metrics.json`：21 次原模型实验的全程/共同时间窗误差和指令统计。", "- `sitl_diagnostics.json`：空中起飞保护触发、入口误差与时序。", "- `model_inertia.json`：SDF 零角度惯量/COM 汇总及计算假设。", "- `source_manifest.json`：本次原项目各源码 SHA256。", "", "运行 `/usr/bin/python3 ros2_ws/analysis/compare_numerical_sitl.py` 可重复 21 个离线实验（输出目录参数 `--out`）；运行 `/usr/bin/python3 ros2_ws/analysis/summarize_numerical_sitl.py` 可更新本目录报告与图。", "", "本轮完成分析和离线验证，尚未修改生产控制器，也没有重新进行实时 SITL 的因果隔离。"]
    (OUT / "README.md").write_text("\n".join(lines) + "\n")
    print(json.dumps(diagnostics, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
