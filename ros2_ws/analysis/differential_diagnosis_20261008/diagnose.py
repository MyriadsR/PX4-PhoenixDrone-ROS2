#!/usr/bin/env python3
"""Offline ablations only. Original sources, logs and production code are read-only.

Run: OPENBLAS_NUM_THREADS=1 MPLBACKEND=Agg /usr/bin/python3 diagnose.py
All comparisons use the complete 3.3 s numerical mission. Reference changes and
instantaneous actuators are diagnostic experiments, not accepted controller fixes.
"""
import contextlib
import hashlib
import json
import os
from pathlib import Path
import sys

os.environ.setdefault('MPLBACKEND', 'Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

OUT = Path(__file__).resolve().parent
ORIGINAL = Path('/home/zr/Tailsitter-control')
PRODUCTION = Path('/home/zr/PX4-PhoenixDrone-ROS2/ros2_ws/src/phoenix_tailsitter_control')
sys.path.insert(0, str(ORIGINAL))
import main_sim as sim
from utils.math_help import zxy_euler_to_quaternion


def rotation(q):
    return Rotation.from_quat(np.asarray(q)[..., [1, 2, 3, 0]])


def hashes():
    files = list(ORIGINAL.rglob('*.py')) + list(PRODUCTION.rglob('*.py'))
    files += list(ORIGINAL.glob('validation_logs/*.npz'))
    return {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in files
            if '__pycache__' not in str(p)}


def trajectory(mode):
    tr = TRAJECTORY_ORIGINAL(speed=3.5 if mode == 'half_speed' else 7.,
                             yaw_duration=1.6 if mode == 'slow_yaw' else .52)
    if mode == 'centered_yaw':
        sample = tr.sample

        def centered(t):
            p, v, a, j, _, _ = sample(t)
            # Preserve every translational sample; center yaw on v_y=0 at 1.70 s.
            u = np.clip((t - 1.44) / .52, 0., 1.)
            s = 10*u**3 - 15*u**4 + 6*u**5
            ds = 30*u**2 * (1-u)**2 / .52
            return p, v, a, j, np.pi/2 + np.pi*s, np.pi*ds
        tr.sample = centered
    return tr


CONFIG_ORIGINAL = sim.TailsitterConfig
TRAJECTORY_ORIGINAL = sim.DifferentialThrustTurnTrajectory


def run(mode):
    folder = OUT / mode
    folder.mkdir(exist_ok=True)
    pd_original = sim.PDAttitudeController.compute_angular_acceleration_command
    flat_original = sim.FlatnessAttitudeController.compute_desired_attitude_and_thrust
    alloc_original = sim.ControlAllocator.allocate_controls
    last = {}
    records = {k: [] for k in ('q_actual', 'q_command', 'q_nominal', 'nominal_thrust',
        'flap_sum', 'force_command', 'total_thrust_command', 'moment_command',
        'allocated_moment', 'motor_command', 'flap_command', 'rate_frame_difference')}
    cfg = CONFIG_ORIGINAL()
    if mode == 'rate_2000':
        cfg.control_rate_hz = 2000.
    if mode in ('instant_actuators', 'instant_motors'):
        cfg.motor_time_constant = 1.e-6
    if mode in ('instant_actuators', 'instant_servos'):
        cfg.servo_time_constant = 1.e-6
        cfg.servo_rate_max = 1.e9
    aero = sim.Aerodynamics(cfg)

    def flat(self, f, v, ds, psi, q, return_angles=False):
        qc, total, phi, th = flat_original(self, f, v, ds, psi, q, return_angles=True)
        if return_angles:
            last.update(nominal=qc.copy(), thrust=float(total), ds=float(ds))
        else:
            last['force'] = f.copy()
        return (qc, total, phi, th) if return_angles else (qc, total)

    def pd(self, q, qc, w, wr):
        rn, ra = rotation(last['nominal']), rotation(q)
        transported = (ra.inv() * rn).apply(wr)
        for k, val in (('q_actual', q), ('q_command', qc),
                       ('q_nominal', last['nominal']), ('nominal_thrust', last['thrust']),
                       ('flap_sum', last['ds']), ('force_command', last['force']),
                       ('rate_frame_difference', transported-wr)):
            records[k].append(np.array(val).copy())
        return pd_original(self, q, qc, w, transported if mode == 'rate_transport' else wr)

    def alloc(self, total, moment, vb):
        motor, flap = alloc_original(self, total, moment, vb)
        _, realized, _ = aero.compute_forces_and_moments(vb, *motor, *flap)
        for k, val in (('total_thrust_command', total), ('moment_command', moment),
                       ('allocated_moment', realized), ('motor_command', motor),
                       ('flap_command', flap)):
            records[k].append(np.array(val).copy())
        return motor, flap

    sim.TailsitterConfig = lambda: cfg
    sim.DifferentialThrustTurnTrajectory = lambda **kw: trajectory(mode)
    sim.PDAttitudeController.compute_angular_acceleration_command = pd
    sim.FlatnessAttitudeController.compute_desired_attitude_and_thrust = flat
    sim.ControlAllocator.allocate_controls = alloc
    try:
        with (folder/'simulation.log').open('w') as stream, contextlib.redirect_stdout(stream):
            metrics = sim.main('differential-turn', log_path=str(folder/'flight_log.npz'),
                               show_plot=False)
    finally:
        sim.TailsitterConfig = CONFIG_ORIGINAL
        sim.DifferentialThrustTurnTrajectory = TRAJECTORY_ORIGINAL
        sim.PDAttitudeController.compute_angular_acceleration_command = pd_original
        sim.FlatnessAttitudeController.compute_desired_attitude_and_thrust = flat_original
        sim.ControlAllocator.allocate_controls = alloc_original
    records = {k: np.array(v) for k, v in records.items()}
    np.savez_compressed(folder/'diagnostics.npz', **records)
    d = np.load(folder/'flight_log.npz')
    t = d['time']
    attitude_error = (rotation(records['q_actual']).inv()*rotation(records['q_command'])).magnitude()
    command_step = (rotation(records['q_command'][:-1]).inv()*rotation(records['q_command'][1:])).magnitude()
    mr = records['moment_command'] - records['allocated_moment']
    motors, flaps = records['motor_command'], records['flap_command']
    turn = (t >= .9) & (t <= 2.5)
    metrics.update(
        rms_velocity_error=float(np.sqrt(np.mean(np.sum((d['velocity']-d['velocity_ref'])**2,axis=1)))),
        actual_speed_at_reference_stop=float(np.linalg.norm(d['velocity'][np.argmin(abs(t-1.7))])),
        max_attitude_command_error_deg=float(np.rad2deg(attitude_error.max())),
        max_command_step_deg=float(np.rad2deg(command_step.max())),
        max_command_step_time_s=float(t[np.argmax(command_step)+1]),
        maximum_feedforward_body_rate_deg_s=float(np.rad2deg(np.linalg.norm(d['angular_velocity_ref'],axis=1).max())),
        peak_body_rate_time_s=float(t[np.argmax(np.linalg.norm(d['angular_velocity_ref'],axis=1))]),
        maximum_actual_body_rate_deg_s=float(np.rad2deg(np.linalg.norm(d['angular_velocity'],axis=1).max())),
        max_abs_angular_acceleration_command=np.abs(d['angular_acceleration_command']).max(axis=0).tolist(),
        max_moment_residual_nm=np.abs(mr).max(axis=0).tolist(),
        peak_moment_demand_nm=np.abs(records['moment_command']).max(axis=0).tolist(),
        turn_motor_bound_fraction=float(np.mean(np.any((motors[turn]<1e-4)|(motors[turn]>=2499.9999),axis=1))),
        turn_flap_bound_fraction=float(np.mean(np.any(np.abs(flaps[turn])>.9999,axis=1))),
        maximum_rate_frame_difference_rad_s=float(np.linalg.norm(records['rate_frame_difference'],axis=1).max()),
        final_position_error_m=float(np.linalg.norm(d['position'][-1]-d['position_ref'][-1])),
        reference_changed=mode in ('centered_yaw', 'slow_yaw', 'half_speed'),
        actuator_dynamics_changed=mode.startswith('instant'),
    )
    (folder/'metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
    print(mode, json.dumps({k:metrics[k] for k in ('rms_position_error','max_position_error',
            'actual_speed_at_reference_stop','maximum_feedforward_body_rate_deg_s',
            'max_command_step_deg','turn_motor_bound_fraction','turn_flap_bound_fraction')}),flush=True)
    return metrics


def nominal(mode, delta_sum):
    cfg = CONFIG_ORIGINAL()
    flat = sim.FlatnessAttitudeController(cfg)
    tr = trajectory(mode)
    t = np.arange(0,3.3001,.0005)
    q = zxy_euler_to_quaternion(np.pi/2,0,0)
    qs, ws, thrust, vel, yaw, yawdot = [],[],[],[],[],[]
    for ti in t:
        _, v, a, j, psi, psid = tr.sample(ti)
        f = cfg.mass*(a-[0,0,cfg.g])
        q, total, phi, th = flat.compute_desired_attitude_and_thrust(f,v,delta_sum,psi,q,return_angles=True)
        w = flat.compute_feedforward_angular_velocity(v,a,j,psi,psid,f,phi,th,delta_sum)
        qs.append(q);ws.append(w);thrust.append(total);vel.append(v);yaw.append(psi);yawdot.append(psid)
    qs, ws, thrust = np.array(qs), np.array(ws), np.array(thrust)
    dr = rotation(qs[:-1]).inv()*rotation(qs[1:])
    actual_geometric_rate = dr.as_rotvec()/.0005
    omega_dot = np.gradient(ws,.0005,axis=0)
    J = np.array(cfg.inertia_matrix)
    moment = omega_dot@J.T+np.cross(ws,ws@J.T)
    omega_norm = np.linalg.norm(ws,axis=1)
    wn = np.argmax(omega_norm)
    allocator = sim.ControlAllocator(cfg)
    term = allocator.c_a0*allocator.c_at*(1-cfg.c_DT)-allocator.s_a0*allocator.s_at*(cfg.c_LT-1)
    denom = cfg.l_Ty*term-allocator.s_aT*cfg.c_mu/cfg.c_T
    delta_demand = moment[:,2]/denom
    capacity = np.minimum(thrust,2*cfg.c_T*cfg.omega_max**2-thrust)
    data = dict(time=t,q=qs,body_rate=ws,thrust=thrust,velocity=np.array(vel),
                yaw=np.array(yaw),yaw_rate=np.array(yawdot),moment=moment,
                differential_demand=delta_demand,differential_capacity=capacity)
    # Fixed flap sum is a geometric diagnostic, not a proof of full four-input feasibility.
    metric = dict(delta_sum_rad=delta_sum,peak_body_rate_deg_s=float(np.rad2deg(omega_norm[wn])),
        peak_body_rate_time_s=float(t[wn]),peak_body_rate_components_rad_s=ws[wn].tolist(),
        peak_yaw_rate_deg_s=float(np.rad2deg(np.max(yawdot))),
        min_thrust_n=float(thrust.min()),max_thrust_n=float(thrust.max()),
        max_quaternion_step_deg=float(np.rad2deg(dr.magnitude().max())),
        peak_moment_nm=np.abs(moment).max(axis=0).tolist(),
        differential_infeasible_time_fraction=float(np.mean(np.abs(delta_demand)>capacity)),
        rate_formula_vs_geometry_rms_rad_s=float(np.sqrt(np.mean(np.sum((actual_geometric_rate-(ws[:-1]+ws[1:])/2)**2,axis=1)))))
    return data,metric


def plots(metrics, nominal_data):
    plt.rcParams.update({'font.size':10,'svg.fonttype':'none'})
    d=np.load(OUT/'baseline/flight_log.npz');r=np.load(OUT/'baseline/diagnostics.npz');t=d['time']
    fig,ax=plt.subplots(3,2,figsize=(13,10),layout='constrained')
    for k,c in enumerate(('tab:blue','tab:orange','tab:green')):
        ax[0,0].plot(t,d['position'][:,k]-d['position_ref'][:,k],label='xyz'[k],color=c)
    ax[0,0].set(title='Position tracking error (NED)',ylabel='Error (m)')
    ax[0,1].plot(t,d['velocity'][:,1],label='Actual v_y')
    ax[0,1].plot(t,d['velocity_ref'][:,1],'--',label='Reference v_y')
    ax[0,1].plot(t,np.linalg.norm(d['velocity'],axis=1),label='Actual speed',alpha=.65)
    ax[0,1].set(title='Velocity reversal',ylabel='Velocity (m/s)')
    ax[1,0].plot(t,np.rad2deg(np.unwrap(np.deg2rad(d['euler_deg'][:,0]))),label='Actual yaw')
    ax[1,0].plot(t,np.rad2deg(d['psi_ref']),'--',label='Reference yaw')
    ax[1,0].set(title='Yaw tracking (ZXY)',ylabel='Angle (deg)')
    err=np.rad2deg((rotation(r['q_actual']).inv()*rotation(r['q_command'])).magnitude())
    ax[1,1].plot(t,err,label='Quaternion angle error')
    ax[1,1].set(title='Full attitude command tracking',ylabel='Angle error (deg)')
    ax[2,0].plot(t,r['moment_command'][:,2],label='Requested M_z')
    ax[2,0].plot(t,r['allocated_moment'][:,2],label='At allocated command')
    ax[2,0].set(title='Moment lost at control allocation (before actuator lag)',ylabel='Moment (Nm)')
    for k in range(2):ax[2,1].plot(t,np.rad2deg(r['flap_command'][:,k]),label=f'Flap {k+1} command')
    ax[2,1].set(title='Flap saturation',ylabel='Deflection (deg)')
    for a in ax.flat:
        a.axvspan(.9,1.42,alpha=.08,color='red');a.axvline(1.7,color='gray',ls=':');a.grid(alpha=.25);a.legend(fontsize=9);a.set_xlabel('Time (s)')
    fig.suptitle('Fresh original numerical differential-turn, 7 m/s, 500 Hz, full 3.3 s')
    for ext in ('png','svg'):fig.savefig(OUT/f'baseline_tracking.{ext}',dpi=160)
    plt.close(fig)
    fig,ax=plt.subplots(2,2,figsize=(13,8),layout='constrained')
    colors={'baseline':'tab:red','centered_yaw':'tab:blue','slow_yaw':'tab:green'}
    for name,data in nominal_data.items():
        c=colors[name]
        ax[0,0].plot(data['time'],np.rad2deg(data['yaw']),color=c,label=name)
        ax[0,1].plot(data['time'],np.rad2deg(np.linalg.norm(data['body_rate'],axis=1)),color=c,label=name)
        ax[1,0].plot(data['time'],data['thrust'],color=c,label=name)
        ax[1,1].plot(data['time'],data['differential_demand'],color=c,label=name)
    data=nominal_data['baseline'];ax[1,1].plot(data['time'],data['differential_capacity'],'k--',label='Baseline +capacity')
    ax[1,1].plot(data['time'],-data['differential_capacity'],'k--',label='Baseline -capacity')
    twin=ax[0,0].twinx();twin.plot(data['time'],data['velocity'][:,1],'k--',alpha=.5);twin.set_ylabel('Common reference v_y (m/s)')
    ax[0,0].set(title='Yaw / velocity scheduling',ylabel='Yaw reference (deg)')
    ax[0,1].set(title='Body rotation demand exceeds yaw-rate reference',ylabel='Body rate norm (deg/s)')
    ax[1,0].set(title='Nominal thrust',ylabel='Thrust (N)')
    ax[1,1].set(title='Nominal differential demand / positive-motor capacity',ylabel='Differential thrust (N)')
    for a in ax.flat:a.grid(alpha=.25);a.legend(fontsize=8);a.set_xlabel('Time (s)')
    fig.suptitle('Geometric flatness diagnostic: fixed flap sum -0.1107 rad, dt=0.5 ms\nReference changes are experiments; not a full actuator feasibility proof')
    for ext in ('png','svg'):fig.savefig(OUT/f'reference_demand.{ext}',dpi=160)
    plt.close(fig)
    fig,ax=plt.subplots(2,1,figsize=(12,8),layout='constrained',sharex=True)
    for name in metrics:
        dd=np.load(OUT/name/'flight_log.npz')
        ax[0].plot(dd['time'],np.linalg.norm(dd['position']-dd['position_ref'],axis=1),label=name)
        if name in ('baseline','centered_yaw','slow_yaw','instant_actuators'):
            ax[1].plot(dd['time'],dd['velocity'][:,1],label=name)
    ax[1].plot(t,d['velocity_ref'][:,1],'k--',label='7 m/s reference v_y')
    ax[0].set(ylabel='Position error norm (m)',title='Controlled ablations, complete 3.3 s interval')
    ax[1].set(ylabel='v_y (m/s)',xlabel='Time (s)',title='Selected reversal responses')
    for a in ax:a.legend(ncol=3,fontsize=9);a.grid(alpha=.25)
    for ext in ('png','svg'):fig.savefig(OUT/f'ablation_comparison.{ext}',dpi=160)
    plt.close(fig)


if __name__=='__main__':
    before=hashes()
    (OUT/'source_manifest.json').write_text(json.dumps(before,indent=2)+'\n')
    modes=('baseline','rate_2000','instant_motors','instant_servos','instant_actuators',
           'rate_transport','centered_yaw','slow_yaw','half_speed')
    metrics={mode:run(mode) for mode in modes}
    nom,nom_metrics={},{}
    for mode in ('baseline','centered_yaw','slow_yaw'):
        data,metric=nominal(mode,-.1107)
        np.savez_compressed(OUT/f'nominal_{mode}.npz',**data)
        nom[mode]=data;nom_metrics[mode]=metric
    for ds in (0.,-.54,.54):
        _,metric=nominal('baseline',ds);nom_metrics[f'baseline_flap_sum_{ds}']=metric
    (OUT/'metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
    (OUT/'nominal_metrics.json').write_text(json.dumps(nom_metrics,indent=2)+'\n')
    plots(metrics,nom)
    after=hashes()
    assert before==after,'Original or production files changed during offline diagnosis'
    (OUT/'source_unchanged.json').write_text(json.dumps({'verified':True,'files':len(before)},indent=2)+'\n')
    print('Original sources/logs and production sources unchanged.',len(before),'files')
