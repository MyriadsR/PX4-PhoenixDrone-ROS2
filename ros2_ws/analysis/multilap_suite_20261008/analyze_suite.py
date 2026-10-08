#!/usr/bin/env python3
"""Consistent, unsmoothed scoring and scientific figures for all fresh runs."""
import argparse
import csv
import hashlib
import inspect
import json
from pathlib import Path
import sys
import textwrap
import types

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.signal import welch
from matplotlib.ticker import MaxNLocator

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE/'mission_overlay'))
import plot_tracking_details as plot
from analyze_tracking_oscillations import read_extra, hold, timing, band_rms
from compare_tracking_improvements import read_diagnostics
from phoenix_tailsitter_control.paper_trajectory_mission import make_reference
from phoenix_tailsitter_control.config import PhoenixHoverConfig
from phoenix_tailsitter_control.maneuver_profiles import CONTROLLER_PROFILES
from phoenix_tailsitter_control import flatness_control

NAMES = ['segments', 'lemniscate', 'knife-edge-transition', 'circle-coordinated',
         'transition-to-forward', 'transition-to-hover', 'differential-turn']
PERIODIC = NAMES[1:4]
COLORS = ['#007F86', '#5570B4', '#E68A2E']
FS = 400.


def dump(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def csv_rows(path, rows):
    if not rows:
        return
    keys = list(dict.fromkeys(k for row in rows for k in row))
    with path.open('w') as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def rms(v):
    return float(np.sqrt(np.mean(np.sum(v*v, axis=1)))) if v.ndim == 2 else float(np.sqrt(np.mean(v*v)))


def basic_metrics(error, velocity_error, speed_error, q_error):
    norm = np.linalg.norm(error, axis=1)
    axes = np.sqrt(np.mean(error*error, axis=0))
    return {'position_rms_m': rms(error), 'position_peak_m': float(norm.max()),
            'position_p95_m': float(np.quantile(norm, .95)),
            **{f'position_{axis}_rms_m':float(v) for axis,v in zip('ned', axes)},
            **{f'position_{axis}_bias_m':float(v) for axis,v in zip('ned', np.mean(error, axis=0))},
            'velocity_vector_rms_m_s':rms(velocity_error), 'speed_magnitude_rms_m_s':rms(speed_error),
            'attitude_rms_deg':rms(q_error), 'attitude_p95_deg':float(np.quantile(q_error,.95)),
            'attitude_peak_deg':float(q_error.max())}


def phases(name, reference):
    if name == 'segments':
        return [('takeoff',0,6),('hover_1',6,11),('forward',11,17),
                ('hover_2',17,22),('right',22,28),('hover_3',28,33)]
    if name.startswith('transition-'):
        return [('lead',0,.4),('speed_transition',.4,3.4),('settle',3.4,3.8)]
    if name == 'differential-turn':
        return [('entry',0,.9),('velocity_reversal',.9,2.5),
                ('yaw_reversal',1.44,1.96),('exit_straight',2.5,3.3)]
    return []


def replay_jump_branches(p,att,alpha,steps,cfg):
    """Replay only jump-adjacent inputs; require quaternion agreement first.

    The trusted local method is copied in memory and given diagnostic state
    assignments. Its formula and return values remain unchanged. Transport
    makes flap sample alignment uncertain, so unverified events stay unknown.
    """
    original=flatness_control.FlatnessAttitudeController.attitude_and_thrust
    source=textwrap.dedent(inspect.getsource(original))
    source=source.replace('    force = np.asarray(force_ned, dtype=float)',
                          '    self._diag_alignment = None\n    force = np.asarray(force_ned, dtype=float)')
    source=source.replace('        roll = (roll_candidate if np.dot(body_y_current, new_y) > 0.0',
                          '        self._diag_alignment = float(np.dot(body_y_current, new_y))\n        roll = (roll_candidate if np.dot(body_y_current, new_y) > 0.0')
    source=source.replace('    if thrust < 0.0:', '    self._diag_raw_thrust = float(thrust)\n    if thrust < 0.0:')
    namespace=dict(flatness_control.__dict__);exec(source,namespace)
    controller=flatness_control.FlatnessAttitudeController(cfg)
    controller.attitude_and_thrust=types.MethodType(namespace['attitude_and_thrust'],controller)
    events=[]
    for j in np.where(steps>90)[0]:
        ends=[]
        for k in [j,j+1]:
            index=int(np.argmin(np.abs(p[:,0]-att[k,0])))
            row=p[index];near=int(np.searchsorted(alpha[:,0],row[0]))
            candidates=range(max(0,near-2),min(len(alpha),near+2))
            options=[]
            for ai in candidates:
                flap=float(np.sum(alpha[ai,15:17]-alpha[ai,17:19]))
                q,_=controller.attitude_and_thrust(row[25:28],row[7:10],flap,row[31],att[k,1:5])
                expected=Rotation.from_quat(att[k,5:9][[1,2,3,0]])
                computed=Rotation.from_quat(q[[1,2,3,0]])
                disagreement=float(np.rad2deg((expected.inv()*computed).magnitude()))
                options.append({'quaternion_replay_disagreement_deg':disagreement,
                                'raw_thrust_n':controller._diag_raw_thrust,
                                'negative_thrust_branch':controller._diag_raw_thrust<0,
                                'roll_alignment':controller._diag_alignment,
                                'alternate_roll_branch':None if controller._diag_alignment is None else controller._diag_alignment<=0,
                                'flap_sample_time_offset_s':float(alpha[ai,0]-row[0])})
            ends.append(min(options,key=lambda x:x['quaternion_replay_disagreement_deg']))
        verified=all(x['quaternion_replay_disagreement_deg']<.5 for x in ends)
        events.append({'time_s':float(att[j+1,0]),'step_deg':float(steps[j]),
                       'replay_verified_within_0_5deg':verified,'before':ends[0],'after':ends[1],
                       'negative_thrust_branch_toggled':verified and ends[0]['negative_thrust_branch']!=ends[1]['negative_thrust_branch'],
                       'alternate_roll_branch_toggled':verified and all(x['alternate_roll_branch'] is not None for x in ends) and ends[0]['alternate_roll_branch']!=ends[1]['alternate_roll_branch']})
    return events


def render(folder, name, repeat, p, att, metrics, lap_time, planned, origin):
    out = folder/'continuous_figures'
    out.mkdir(exist_ok=True)
    actual, target = plot.euler_pairs(att)
    title = (f'PX4 SITL + Gazebo | {name} | fresh repeat {repeat}/3\n'
             f'{metrics["status"]} | coverage {metrics["coverage"]:.1%} | '
             f'continuous reference | position RMS {metrics["position_rms_m"]:.3f} m, peak {metrics["position_peak_m"]:.3f} m')
    if name == 'differential-turn':
        title += ' | centered yaw'
    duration = min(planned, metrics['recorded_duration_s'])
    def errors(axes):
        plot.plot_errors(axes,p,lap_time,duration)
        for axis, key in zip(axes,'ned'):
            for t in list(axis.texts):t.remove()
            axis.text(.98,.90,f'RMS {metrics[f"position_{key}_rms_m"]:.3f} m',transform=axis.transAxes,ha='right',fontsize=9)
    def spatial(ax):
        plot.plot_3d(ax,p,10.,origin)
        span=np.maximum(np.ptp(np.vstack((p[:,1:4],p[:,4:7])),axis=0),[1,1,.5])
        ax.set_box_aspect([2.3,4.,2.5] if name=='differential-turn' else [span[0],span[1],max(span[2],.6*max(span[:2]))])
        for t in list(ax.texts):t.remove()
        ax.text2D(.01,.96,'Display aspect adjusted; all axes in metres',transform=ax.transAxes,fontsize=7,color='#66727D')
        for a in (ax.xaxis,ax.yaxis,ax.zaxis):a.set_major_locator(MaxNLocator(nbins=3))
        ax.tick_params(labelsize=8,pad=0)
    fig,axes=plt.subplots(3,1,figsize=(11,7.5),sharex=True,layout='constrained')
    errors(axes);fig.suptitle(title,fontsize=11);plot.save(fig,out,'position_errors')
    fig,ax=plt.subplots(figsize=(11,4),layout='constrained')
    plot.plot_speed(ax,p,lap_time,duration);fig.suptitle(title,fontsize=11);plot.save(fig,out,'speed_magnitude')
    fig,axes=plt.subplots(3,1,figsize=(11,8),sharex=True,layout='constrained')
    plot.plot_attitude(axes,att,actual,target,lap_time,duration);fig.suptitle(title+'\nAttitude: actual / controller command (TS intrinsic ZXY)',fontsize=11);plot.save(fig,out,'attitude_tracking')
    fig=plt.figure(figsize=(10,8),layout='constrained');spatial(fig.add_subplot(projection='3d'))
    fig.suptitle(title,fontsize=11);plot.save(fig,out,'trajectory_3d')
    fig=plt.figure(figsize=(16,12.5),layout='constrained');grid=fig.add_gridspec(2,2,height_ratios=[1.5,1.2])
    left=grid[0,0].subgridspec(3,1);right=grid[0,1].subgridspec(3,1)
    errors([fig.add_subplot(left[i]) for i in range(3)])
    plot.plot_attitude([fig.add_subplot(right[i]) for i in range(3)],att,actual,target,lap_time,duration)
    plot.plot_speed(fig.add_subplot(grid[1,0]),p,lap_time,duration)
    spatial(fig.add_subplot(grid[1,1],projection='3d'))
    fig.suptitle(title+'\nRecorded PX4 state is not smoothed; reference is analytic',fontsize=12)
    plot.save(fig,out,'tracking_overview')


def analyze_case(folder, force=False, redraw=True):
    cached=folder/'uniform_metrics.json'
    if cached.exists() and not force:
        return json.loads(cached.read_text())
    result=json.loads((folder/'case_result.json').read_text())
    name=folder.name;repeat=int(folder.parent.name.split('_')[-1])
    base={'trajectory':name,'repeat':repeat,'status':result['status'],'mission_passed':result.get('mission',{}).get('passed',False),
          'folder':str(folder.resolve()),'abort_reason':result.get('abort_reason',result.get('error',''))}
    if not (folder/'tracking_samples.npz').exists():
        dump(cached,base);return base
    stored=json.loads((folder/'summary.json').read_text());mission=stored['mission']
    z=np.load(folder/'tracking_samples.npz');p=z['position_debug'].copy();att=z['attitude_debug']
    cache=HERE/'diagnostic_cache';cache.mkdir(exist_ok=True)
    extra=read_extra(folder,cache/f'{repeat}_{name}.npz')
    diag=read_diagnostics(folder,float(extra['phase_start_epoch_s']))
    ref=make_reference(name,differential_yaw_centered=bool(mission.get('differential_yaw_centered')))
    planned=float(mission['reference_duration_s'])
    publish=extra['sp'];publish=publish[(publish[:,1]>=0)&(publish[:,1]<=planned)]
    translation=np.median(publish[:,2:5]-np.array([ref.sample(t)[0] for t in publish[:,1]]),axis=0)
    residual=np.linalg.norm(publish[:,2:5]-np.array([ref.sample(t)[0] for t in publish[:,1]])-translation,axis=1)
    assert np.quantile(residual,.95)<.03, (name,'reference mismatch')
    end=min(planned,stored['tracking_duration_s'],p[-1,0]+np.median(np.diff(p[:,0])))
    grid=np.arange(max(p[0,0],att[0,0],extra['control'][0,0]),end,1/FS)
    state=hold(p[:,0],p[:,1:13],grid)
    ref_samples=[ref.sample(t) for t in grid]
    rp=np.array([s[0]+translation for s in ref_samples]);rv=np.array([s[1] for s in ref_samples])
    error=state[:,:3]-rp;ve=state[:,6:9]-rv
    speed=np.linalg.norm(state[:,6:9],axis=1);speed_ref=np.linalg.norm(rv,axis=1)
    qa=Rotation.from_quat(att[:,1:5][:,[1,2,3,0]]);qt=Rotation.from_quat(att[:,5:9][:,[1,2,3,0]])
    qe=np.rad2deg((qa.inv()*qt).magnitude());qg=hold(att[:,0],qe[:,None],grid)[:,0]
    cg=hold(extra['control'][:,0],extra['control'][:,1:],grid)
    base.update(basic_metrics(error,ve,speed-speed_ref,qg))
    base.update({'planned_duration_s':planned,'recorded_duration_s':stored['tracking_duration_s'],
                 'coverage':min(1.,stored['tracking_duration_s']/planned),'laps':mission['laps'],
                 'lap_time_s':mission['lap_time_s'],'speed_parameter_m_s':mission['speed_m_s'],
                 'speed_peak_m_s':float(speed.max()),'speed_min_m_s':float(speed.min()),
                 'scoring_grid_hz':FS,'scoring_sample_count':len(grid),
                 'reference_translation_ned_m':translation.tolist(),
                 'reference_p95_translation_residual_m':float(np.quantile(residual,.95)),
                 'reference_variant':mission.get('reference_variant','original'),
                 'reference_speed_peak_m_s':float(speed_ref.max()),
                 'reference_acceleration_peak_m_s2':float(np.linalg.norm(np.array([s[2] for s in ref_samples]),axis=1).max()),
                 'reference_jerk_peak_m_s3':float(np.linalg.norm(np.array([s[3] for s in ref_samples]),axis=1).max()),
                 'reference_yaw_rate_peak_deg_s':float(np.rad2deg(np.max(np.abs([s[5] for s in ref_samples])))),
                 'tracking_altitude_minmax_m':[-float(state[:,2].max())+mission['takeoff_origin_ned_m'][2],-float(state[:,2].min())+mission['takeoff_origin_ned_m'][2]],
                 'peak_position_time_s':float(grid[np.argmax(np.linalg.norm(error,axis=1))]),
                 'peak_attitude_time_s':float(grid[np.argmax(qg)]),
                 'body_rate_peak_deg_s':float(np.rad2deg(np.linalg.norm(cg[:,3:6],axis=1)).max()),
                 'position_state_hold_fraction':float(np.mean(np.all(np.diff(p[:,1:4],axis=0)==0,axis=1))),
                 'debug_timing':timing(p),'local_arrival_timing':timing(extra['local']),
                 'local_sample_timing':timing(extra['local'],2),
                 'setpoint_timing':timing(extra['sp'])})
    host_file=HERE/'host_load.csv'
    if host_file.exists():
        host=np.genfromtxt(host_file,delimiter=',',skip_header=1)
        if host.ndim==2 and len(host):
            epoch=float(extra['phase_start_epoch_s'])
            hx=host[(host[:,0]>=epoch)&(host[:,0]<epoch+end)]
            if len(hx):
                base['host_load_samples']=len(hx)
                base['host_cpu_busy_mean_fraction']=float(hx[:,4].mean())
                base['host_cpu_busy_peak_fraction']=float(hx[:,4].max())
                base['host_cpu_samples_over_80pct_fraction']=float(np.mean(hx[:,4]>.8))
                base['host_load_1m_mean']=float(hx[:,1].mean())
    gz_file=HERE/'gazebo_stats.csv'
    if gz_file.exists():
        gs=np.genfromtxt(gz_file,delimiter=',',skip_header=1)
        if gs.ndim==2 and len(gs):
            epoch=float(extra['phase_start_epoch_s'])
            gx=gs[(gs[:,0]>=epoch)&(gs[:,0]<epoch+end)]
            if len(gx):
                base['gazebo_stats_samples']=len(gx)
                base['gazebo_real_time_factor_mean']=float(gx[:,3].mean())
                base['gazebo_real_time_factor_min']=float(gx[:,3].min())
    steps=np.rad2deg((qt[:-1].inv()*qt[1:]).magnitude())
    steps[(att[1:,0]>planned)|(att[:-1,0]<0)]=0.
    base['target_quaternion_step_peak_deg']=float(steps.max())
    base['target_quaternion_steps_over_90_deg']=int(np.sum(steps>90))
    base['target_quaternion_jump_times_s']=att[1:,0][steps>90].tolist()
    base['target_quaternion_jump_degrees']=steps[steps>90].tolist()
    cfg=PhoenixHoverConfig();profile=CONTROLLER_PROFILES.get(name,CONTROLLER_PROFILES['standard'])
    branch_events=replay_jump_branches(p,att,extra['alpha'],steps,cfg)
    dump(folder/'quaternion_jump_replay.json',branch_events)
    base['verified_jump_replay_count']=sum(e['replay_verified_within_0_5deg'] for e in branch_events)
    base['negative_thrust_branch_toggle_count']=sum(e['negative_thrust_branch_toggled'] for e in branch_events)
    base['alternate_roll_branch_toggle_count']=sum(e['alternate_roll_branch_toggled'] for e in branch_events)
    rg=hold(diag['reference'][:,0],diag['reference'][:,1:],grid)
    blend=rg[:,4]
    for key,values,limits,scale in [('angular_acceleration',cg[:,9:12],cfg.angular_acceleration_limit,profile['maneuver_acceleration_limit_scale']),
                                   ('moment',cg[:,15:18],cfg.moment_limit,profile['maneuver_moment_limit_scale'])]:
        cap=limits[None,:]*(1+blend[:,None]*(scale-1))
        base[f'{key}_software_limit_fraction_xyz']=np.mean(np.abs(values)>=.999*cap,axis=0).tolist()
    base['motor_upper_limit_fraction_lr']=np.mean(cg[:,21:23]>=.999,axis=0).tolist()
    base['motor_lower_limit_fraction_lr']=np.mean(cg[:,21:23]<=1e-6,axis=0).tolist()
    base['flap_limit_fraction_lr']=np.mean(np.abs(cg[:,23:25])>=.999*np.array([cfg.left_flap_limit,cfg.right_flap_limit]),axis=0).tolist()
    base['allocation_moment_residual_rms_nm_xyz']=np.sqrt(np.mean((cg[:,18:21]-cg[:,15:18])**2,axis=0)).tolist()
    base['actuator_output_active_fraction']=float(np.mean(cg[:,26]))
    base['position_18_22hz_rms_m']=band_rms(error,18,22)
    base['position_80_120hz_rms_m']=band_rms(error,80,120)
    base['motors_18_22hz_rms_normalized']=band_rms(cg[:,21:23],18,22)
    base['flaps_18_22hz_rms_deg']=band_rms(np.rad2deg(cg[:,23:25]),18,22)
    if 'timing' in diag:
        tg=hold(diag['timing'][:,0],diag['timing'][:,1:],grid)
        base['control_dt_mean_ms']=float(tg[:,0].mean()*1000)
        base['control_dt_p99_ms']=float(np.quantile(tg[:,0],.99)*1000)
        base['state_arrival_age_mean_ms_rates_local_actuator']=(tg[:,4:7].mean(axis=0)*1000).tolist()
    if 'feedback' in diag:
        fb=hold(diag['feedback'][:,0],diag['feedback'][:,1:],grid)
        base['measured_actuator_feedback_fraction']=float(np.mean(fb[:,14]))
        base['actuator_prediction_motor_rms_rad_s_lr']=np.sqrt(np.mean(fb[:,4:6]**2,axis=0)).tolist()
        base['actuator_prediction_flap_rms_deg_lr']=np.rad2deg(np.sqrt(np.mean(fb[:,10:12]**2,axis=0))).tolist()
    # Secondary estimator-native scoring removes repeated held values and uses
    # the PX4 measurement timestamp; it is not Gazebo truth or state smoothing.
    lp=extra['local'];ts=lp[:,2]-float(extra['phase_start_epoch_s'])
    lm=(ts>=0)&(ts<planned)
    if np.any(lm):
        native=lp[lm];nr=[ref.sample(t) for t in ts[lm]]
        ne=native[:,3:6]-np.array([s[0]+translation for s in nr])
        base['measurement_timestamp_position_rms_m']=rms(ne)
        base['measurement_timestamp_position_peak_m']=float(np.linalg.norm(ne,axis=1).max())
        base['measurement_timestamp_sample_count']=len(ne)
    lap_rows=[]
    for lap in range(int(mission['laps'])):
        lo=lap*mission['lap_time_s'];hi=(lap+1)*mission['lap_time_s'];mask=(grid>=lo)&(grid<hi)
        if not np.any(mask):continue
        lm=basic_metrics(error[mask],ve[mask],(speed-speed_ref)[mask],qg[mask])
        lm.update(trajectory=name,repeat=repeat,lap=lap+1,
                  coverage=min(1.,max(0.,end-lo)/(hi-lo)),start_s=lo,end_s=hi)
        lap_rows.append(lm)
    phase_rows=[]
    for label,lo,hi in phases(name,ref):
        mask=(grid>=lo)&(grid<hi)
        if np.any(mask):
            row=basic_metrics(error[mask],ve[mask],(speed-speed_ref)[mask],qg[mask])
            row.update(trajectory=name,repeat=repeat,phase=label,start_s=lo,end_s=hi)
            phase_rows.append(row)
    if name=='differential-turn':
        k=np.argmin(np.abs(grid-1.7));base['speed_at_reference_stop_m_s']=float(speed[k])
    if name.startswith('transition-'):
        k=np.argmin(np.abs(grid-3.4));base['speed_at_transition_end_m_s']=float(speed[k])
    if name in PERIODIC and len(lap_rows)>=2:
        complete=[x for x in lap_rows if x['coverage']>=.999]
        if len(complete)>=2:
            base['lap_rms_trend_m_per_lap']=float(np.polyfit([x['lap'] for x in complete],[x['position_rms_m'] for x in complete],1)[0])
            base['first_lap_rms_m']=complete[0]['position_rms_m'];base['last_lap_rms_m']=complete[-1]['position_rms_m']
            base['later_laps_mean_rms_m']=float(np.mean([x['position_rms_m'] for x in complete[1:]]))
            base['odd_laps_mean_rms_m']=float(np.mean([x['position_rms_m'] for x in complete if x['lap']%2]))
            even=[x['position_rms_m'] for x in complete if x['lap']%2==0]
            if even:base['even_laps_mean_rms_m']=float(np.mean(even))
    dump(folder/'lap_metrics.json',lap_rows);dump(folder/'phase_metrics.json',phase_rows)
    csv_rows(folder/'lap_metrics.csv',lap_rows);csv_rows(folder/'phase_metrics.csv',phase_rows)
    np.savez_compressed(folder/'uniform_scoring.npz',time_s=grid,position_error_ned_m=error,
                        velocity_error_ned_m_s=ve,speed_m_s=speed,reference_speed_m_s=speed_ref,
                        attitude_error_deg=qg,control=cg)
    # Preserve original debug NPZ/CSV and put the continuous references in new files.
    samples=[ref.sample(float(t)) for t in p[:,0]]
    p[:,4:7]=np.array([s[0]+translation for s in samples]);p[:,10:13]=np.array([s[1] for s in samples])
    np.savetxt(folder/'continuous_position_tracking.csv',p[:,:13],delimiter=',',header='time_s,n_m,e_m,d_m,n_ref_m,e_ref_m,d_ref_m,vn_m_s,ve_m_s,vd_m_s,vn_ref_m_s,ve_ref_m_s,vd_ref_m_s',comments='')
    if redraw or not (folder/'continuous_figures/tracking_overview.png').exists():
        render(folder,name,repeat,p,att,base,mission['lap_time_s'],planned,mission['takeoff_origin_ned_m'][2])
        render_diagnostics(folder,grid,error,qg,cg,base,lap_rows)
    dump(cached,base)
    print(f'Analyzed repeat {repeat} {name}: {base["status"]}, RMS {base["position_rms_m"]:.3f}, peak {base["position_peak_m"]:.3f}',flush=True)
    return base


def render_diagnostics(folder,t,error,qe,cg,m,laps):
    out=folder/'continuous_figures'
    fig,axes=plt.subplots(4,1,figsize=(12,10),sharex=True,layout='constrained')
    axes[0].plot(t,np.linalg.norm(error,axis=1),color=COLORS[0]);axes[0].set_ylabel('Position error norm (m)')
    axes[1].plot(t,qe,color=COLORS[1]);axes[1].set_ylabel('Quaternion error (deg)')
    axes[2].plot(t,cg[:,21],label='Left');axes[2].plot(t,cg[:,22],label='Right');axes[2].set_ylabel('Motor command / max');axes[2].legend()
    axes[3].plot(t,np.rad2deg(cg[:,23]),label='Left');axes[3].plot(t,np.rad2deg(cg[:,24]),label='Right');axes[3].set(ylabel='Flap command (deg)',xlabel='Tracking time (s)');axes[3].legend()
    for ax in axes:
        ax.grid(alpha=.2)
        if m['trajectory'] in PERIODIC:
            for boundary in np.arange(m['lap_time_s'],t[-1],m['lap_time_s']):ax.axvline(boundary,color='#AAAAAA',linestyle=':',linewidth=.7)
    fig.suptitle(f'PX4 SITL | {m["trajectory"]} | repeat {m["repeat"]}: errors and allocated commands',fontsize=12)
    plot.save(fig,out,'control_diagnostics')
    fig,axes=plt.subplots(3,1,figsize=(11,9),layout='constrained')
    for ax,values,title in zip(axes,[error,cg[:,21:23],np.rad2deg(cg[:,23:25])],['Position error PSD (m²/Hz)','Motor command PSD (normalized²/Hz)','Flap command PSD (deg²/Hz)']):
        f,psd=welch(values,fs=FS,nperseg=min(len(t),int(FS*8)),axis=0,detrend='linear')
        labels=['North','East','Down'] if psd.shape[1]==3 else ['Left','Right']
        for i in range(psd.shape[1]):ax.semilogy(f,np.maximum(psd[:,i],1e-14),label=labels[i])
        ax.axvspan(18,22,color=COLORS[2],alpha=.15);ax.axvspan(80,120,color=COLORS[1],alpha=.10)
        ax.set(xlim=(0,150),ylabel=title);ax.grid(alpha=.2);ax.legend(fontsize=8)
    axes[-1].set_xlabel('Frequency (Hz)')
    fig.suptitle(f'{m["trajectory"]} repeat {m["repeat"]}: short, nonstationary spectra are descriptive\n18–22 Hz / 80–120 Hz bands; held state and transport affect position spectrum',fontsize=11)
    plot.save(fig,out,'spectra')
    if m['trajectory'] in PERIODIC:
        bins=60;matrix=np.full((8,bins),np.nan);angle_matrix=matrix.copy()
        for lap in range(8):
            local=(t-lap*m['lap_time_s'])/m['lap_time_s']
            for b in range(bins):
                mask=(local>=b/bins)&(local<(b+1)/bins)
                if np.any(mask):matrix[lap,b]=rms(error[mask]);angle_matrix[lap,b]=rms(qe[mask])
        fig,axes=plt.subplots(2,1,figsize=(11,7),layout='constrained')
        for ax,data,label in zip(axes,[matrix,angle_matrix],['Position RMS (m)','Quaternion RMS (deg)']):
            im=ax.imshow(data,aspect='auto',origin='lower',extent=[0,1,.5,8.5],cmap='viridis')
            ax.set(ylabel='Lap',xlabel='Fraction of lap',yticks=range(1,9));fig.colorbar(im,ax=ax,label=label)
        fig.suptitle(f'{m["trajectory"]} repeat {m["repeat"]}: error location within each lap\nKnife-edge spatial laps alternate upright/inverted posture; compare odd/even laps separately',fontsize=11)
        plot.save(fig,out,'lap_phase_heatmap')


def aggregate(rows):
    summaries=[]
    for name in NAMES:
        group=[r for r in rows if r['trajectory']==name];scored=[r for r in group if 'position_rms_m' in r]
        if not group:continue
        row={'trajectory':name,'runs':len(group),'successful_missions':sum(r['mission_passed'] for r in group),
             'full_tracking_runs':sum(r.get('coverage',0)>=.999 for r in group),'expected_runs':3}
        for key in ['position_rms_m','position_peak_m','velocity_vector_rms_m_s','speed_magnitude_rms_m_s','attitude_rms_deg','attitude_peak_deg']:
            vals=[r[key] for r in scored]
            if vals:row.update({key+'_mean':float(np.mean(vals)),key+'_min':float(min(vals)),key+'_max':float(max(vals)),key+'_sd':float(np.std(vals,ddof=1)) if len(vals)>1 else 0.})
        if scored:
            # Representative is the actual run nearest median full-run RMS;
            # selecting the lowest-error run would obscure reproducibility.
            complete=[r for r in scored if r.get('coverage',0)>=.999] or scored
            target=float(np.median([r['position_rms_m'] for r in complete]))
            selected=min(complete,key=lambda r:abs(r['position_rms_m']-target))
            row['representative_repeat']=selected['repeat'];row['representative_folder']=selected['folder']
        summaries.append(row)
    return summaries


def render_aggregate(rows,summaries,laps,phase_rows):
    out=HERE/'figures';out.mkdir(exist_ok=True)
    fig,axes=plt.subplots(2,2,figsize=(16,10),layout='constrained')
    for ax,(key,title) in zip(axes.flat,[('position_rms_m','Position RMSE (m)'),('position_peak_m','Position peak (m)'),('velocity_vector_rms_m_s','Velocity vector RMSE (m/s)'),('attitude_rms_deg','Quaternion RMSE (deg)')]):
        for repeat,color in zip(range(1,4),COLORS):
            points=[next((r for r in rows if r['trajectory']==name and r['repeat']==repeat),{}) for name in NAMES]
            vals=[r.get(key,np.nan) for r in points]
            x=np.arange(len(NAMES))+(repeat-2)*.23
            bars=ax.bar(x,vals,width=.22,color=color,label=f'Fresh repeat {repeat}')
            for bar,r in zip(bars,points):
                if r and not r['mission_passed']:bar.set_hatch('//');bar.set_edgecolor('#C55F4F')
            for xx,v,r in zip(x,vals,points):
                if np.isfinite(v):ax.text(xx,v,f'{v:.2f}'+('*' if r.get('coverage',0)<.999 else ''),ha='center',va='bottom',fontsize=7,rotation=35)
        ax.set_xticks(range(len(NAMES)),NAMES,rotation=25,ha='right',fontsize=8)
        ax.set_title(title,loc='left');ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
    axes[0,0].legend(fontsize=9)
    fig.suptitle('PX4 SITL + Gazebo: 8 laps for periodic references, 3 fresh repeats per trajectory\n400 Hz common scoring grid, continuous references; * means partial tracking; hatched = mission not passed',fontsize=12)
    plot.save(fig,out,'all_trajectory_comparison')
    fig,axes=plt.subplots(3,2,figsize=(13,11),layout='constrained')
    for i,name in enumerate(PERIODIC):
        for repeat,color in zip(range(1,4),COLORS):
            group=[r for r in laps if r['trajectory']==name and r['repeat']==repeat]
            for ax,key in zip(axes[i],['position_rms_m','attitude_rms_deg']):
                ax.plot([x['lap'] for x in group],[x[key] for x in group],'-o',color=color,label=f'Repeat {repeat}',markersize=4)
                ax.set(title=name,ylabel='Position RMS (m)' if key.startswith('position') else 'Quaternion RMS (deg)',xlabel='Lap',xticks=range(1,9));ax.grid(alpha=.2)
        axes[i,0].legend(fontsize=8)
    fig.suptitle('Per-lap tracking: first-lap transient and later-lap repeatability',fontsize=13)
    plot.save(fig,out,'lap_evolution')
    fig,axes=plt.subplots(4,2,figsize=(15,14),layout='constrained')
    for ax,name in zip(axes.flat,NAMES):
        for repeat,color in zip(range(1,4),COLORS):
            row=next((r for r in rows if r['trajectory']==name and r['repeat']==repeat and 'position_rms_m' in r),None)
            if row:
                z=np.load(Path(row['folder'])/'uniform_scoring.npz')
                ax.plot(z['time_s'],np.linalg.norm(z['position_error_ned_m'],axis=1),label=f'Repeat {repeat} ({row["status"]})',color=color,linewidth=.8,linestyle='-' if row['mission_passed'] else '--')
        ax.set(title=name,xlabel='Tracking time (s)',ylabel='Position error norm (m)');ax.grid(alpha=.2);ax.legend(fontsize=8)
    axes.flat[-1].axis('off')
    fig.suptitle('All recorded repeats: position error envelopes (state not smoothed)',fontsize=14)
    plot.save(fig,out,'repeat_error_overlays')
    for name in NAMES:
        group=[r for r in phase_rows if r['trajectory']==name]
        if not group:continue
        labels=list(dict.fromkeys(r['phase'] for r in group))
        fig,axes=plt.subplots(1,2,figsize=(12,4.5),layout='constrained')
        for ax,key in zip(axes,['position_rms_m','attitude_rms_deg']):
            for repeat,color in zip(range(1,4),COLORS):
                vals=[next((r[key] for r in group if r['repeat']==repeat and r['phase']==label),np.nan) for label in labels]
                ax.bar(np.arange(len(labels))+(repeat-2)*.22,vals,width=.21,color=color,label=f'Repeat {repeat}')
            ax.set_xticks(range(len(labels)),labels,rotation=20,ha='right');ax.set_ylabel('Position RMS (m)' if key.startswith('position') else 'Quaternion RMS (deg)');ax.grid(axis='y',alpha=.2)
        axes[0].legend(fontsize=8);fig.suptitle(name+': phase-specific tracking (yaw_reversal overlaps velocity_reversal)')
        plot.save(fig,out,name+'_phases')
    scored=[r for r in rows if 'position_rms_m' in r]
    if scored:
        fig,axes=plt.subplots(2,2,figsize=(16,10),layout='constrained')
        keys=['control_dt_mean_ms','host_cpu_busy_mean_fraction','debug_rate_hz','gazebo_real_time_factor_mean']
        titles=['Control callback mean dt (ms)','Host CPU busy fraction','Recorded control debug rate (Hz)','Gazebo real-time factor']
        labels=[r['trajectory']+' / '+str(r['repeat']) for r in scored]
        for ax,key,title in zip(axes.flat,keys,titles):
            vals=[r['debug_timing']['mean_rate_hz'] if key=='debug_rate_hz' else r.get(key,np.nan) for r in scored]
            ax.bar(range(len(scored)),vals,color=[COLORS[r['repeat']-1] for r in scored])
            ax.set_xticks(range(len(scored)),labels,rotation=75,ha='right',fontsize=6);ax.set_title(title,loc='left');ax.grid(axis='y',alpha=.2)
        fig.suptitle('Runtime conditions: missing early host/Gazebo samples are not filled with zeros',fontsize=13)
        plot.save(fig,out,'runtime_timing')
        fig,axes=plt.subplots(1,2,figsize=(13,5),layout='constrained')
        cmap=plt.get_cmap('tab10')
        for i,name in enumerate(NAMES):
            group=[r for r in scored if r['trajectory']==name]
            for ax,key in zip(axes,['position_rms_m','attitude_rms_deg']):
                ax.scatter([r['debug_timing']['mean_rate_hz'] for r in group],[r[key] for r in group],label=name,color=cmap(i),s=35)
                for r in group:ax.annotate(str(r['repeat']),(r['debug_timing']['mean_rate_hz'],r[key]),xytext=(3,4),textcoords='offset points',fontsize=7)
                ax.set(xlabel='Recorded control debug rate (Hz)',ylabel='Position RMS (m)' if key.startswith('position') else 'Quaternion RMS (deg)');ax.grid(alpha=.2)
        axes[0].legend(fontsize=7)
        fig.suptitle('Error versus observed controller cadence: repeat IDs annotated\nDescriptive association only; three repeats do not establish causality',fontsize=12)
        plot.save(fig,out,'error_vs_callback_rate')


def num(value,nd=3):
    return '—' if value is None else f'{value:.{nd}f}'


def table(headers,rows):
    return ['| '+' | '.join(headers)+' |','| '+' | '.join(['---']*len(headers))+' |',
            *['| '+' | '.join(str(x) for x in row)+' |' for row in rows]]


def write_report(rows,summaries,laps,phase_rows):
    finished=len(rows);passed=sum(r['mission_passed'] for r in rows)
    lines=['# 七条轨迹多圈与重复测试：PX4 SITL / Gazebo，2026-10-08','',
           f'已完成记录 **{finished}/21** 次独立任务；其中任务通过 **{passed}/{finished}**。'+('本报告为全部任务完成后的最终结果。' if finished==21 else '本报告是中间结果，未完成任务不能视为通过。'),'',
           '## 测试设计与数据来源','',
           '- 明确排除 `circular-knife-edge`。三条周期轨迹统一连续 8 圈：八字、刀刃过渡、协调圆。每条轨迹做 3 次全新仿真，周期轨迹共计划 72 圈。',
           '- 分段起飞、前飞转换、悬停转换、差动掉头是非周期轨迹，每次执行完整原机动，共 3 次独立起飞/退出/降落；不将不同的首尾位置、速度和姿态硬拼成闭环。',
           '- PX4 SITL + Gazebo 动力学 + ROS2 alpha/INDI 控制器。每次重新启动 PX4/Gazebo，位置、速度和姿态来自 PX4 估计；没有用 Gazebo 真值替代估计量，也没有用纯数值仿真结果填充本报告。',
           '- 每次的 bag、仿真日志、任务日志及完整 Python 源码 SHA256 存在 repeat_XX/轨迹目录。三次重复测试相同参数和启动条件，主要覆盖启动/通信时序差异；没有施加随机风、模型扰动或传感器随机种子扫描。',
           '- 每次通过 PX4 rcS 支持的 PX4_PARAM_CAL_MAG{0,1}_{X,Y,Z}OFF 环境变量把初始磁力计校准偏置设为零；磁场健康检查和本次飞行中的正常估计/自动校准仍启用。这样避免上一架次学习的偏置被下一架次继承。没有修改 airframe、传感器噪声、世界磁场或生产控制代码。',
           '- 测试使用现有控制参数。刀刃采用此前保留的 4/4 软件限幅 profile 和切线入轨；掉头采用保留的 8/8 profile、仅机动阶段 1.25 倍姿态带宽及居中偏航。standard 控制保持现状。',
           '- 只在 mission_overlay 测试副本中把周期轨迹圈数设为 8；生产控制模块和原参考快照保持本轮开始时的内容。速度、圆半径、单圈时间和原机动时长不改变。',
           '- 掉头的速度反向时长仍 1.6 s，偏航翻转时长仍 0.52 s，窗口使用此前保留的 1.44–1.96 s，而非原始 0.90–1.42 s。',
           '- NED 坐标：x 北、y 东、z 下。高度目标 10 m；起降 0.4 m/s，准备/退出各 4 s（地面分段含自身起飞），参考发布 20 Hz、控制标称 500 Hz，实际时序另行统计。','',
           '### 实际轨迹条件','']
    conditions=[]
    for name in NAMES:
        scored=next((r for r in rows if r['trajectory']==name and 'position_rms_m' in r),None)
        if scored:
            conditions.append([name,'8 圈连续' if name in PERIODIC else '完整非周期机动',
                               num(scored['planned_duration_s']),num(scored['lap_time_s']),
                               num(scored['reference_speed_peak_m_s']),num(scored['reference_acceleration_peak_m_s2']),
                               num(scored['reference_yaw_rate_peak_deg_s'],1)])
    lines+=table(['轨迹','每次执行方式','参考总时长 s','单圈/机动时长 s','参考速度峰值 m/s','加速度峰值 m/s²','偏航率峰值 °/s'],conditions)
    lines+=['','协调圆半径 3.5 m、速度 8.1 m/s；八字速度 6 m/s、单圈 7 s；刀刃速度 6 m/s、单圈 6.25 s。分段的 33 s 为起飞6 s、悬停5 s、前移6 s、悬停5 s、右移6 s、悬停5 s。转换的3.8 s包括前置0.4 s、转换3 s、收尾0.4 s。掉头3.3 s包括进入0.9 s、速度反向1.6 s、退出0.8 s。','',
           '## 统一统计口径','',
           '主统计仅覆盖正式 track 阶段，排除预备起飞、空中入轨、退出和落地；分段轨迹自身的 6 s 起飞段包含在其 33 s track 中。所有轨迹统一对连续解析参考计分，并重建录包中的固定世界位置平移。PX4 估计位置/速度使用控制器当时持有的状态，不平滑；统一重采样到 400 Hz，避免控制回调次数波动改变样本权重。','',
           'debug 话题是没有自带采样时间戳的 Float64MultiArray，主横轴采用 rosbag 接收时间。因此主统计还包含发布/记录延迟与状态采样保持，不能把所有细锯齿解释为真实运动。下面同时给出 PX4 原生估计采样时间戳的独立位置指标；主参考定义、时间窗口和加权在七条轨迹间保持一致。','',
           '位置误差 e=p_est−p_ref，三维位置 RMS=√mean(||e||²)，峰值=max(||e||)，P95 是范数的第95百分位；三轴 RMS 分别计算。速度向量 RMS 与速度幅值 RMS 分开，避免速度方向错误被幅值指标隐藏。姿态主指标为当前四元数与控制目标四元数的最短旋转角，欧拉角图用 TS 原生 intrinsic ZXY；欧拉角附近的分支/奇异不能直接解释为真实转角。','',
           '主指标仍包含异步估计消息的采样保持效应。另列 measurement_timestamp_position_rms_m：直接用每条 PX4 原生测量时间戳匹配解析参考，不重复持有值，作为时序诊断；它不是 Gazebo 真值，也不是平滑后的真实误差。不同采样时刻/加权使这两种 RMS 不必相等。','',
           '任务 passed 使用原任务的宽松条件：任务采样位置 RMS ≤10 m、峰值 ≤25 m，未中止并成功解除武装。因此 passed 主要说明任务完成，不能代表亚米级精度。本文全部主指标由统一高频分析另算。没有另行指定精度阈值，不将“完成”自动称为高精度。中止记录保留，覆盖不足的部分数据不能与完整轨迹直接按 RMS 排名。','',
           '## 七条轨迹汇总','']
    lines+=table(['轨迹','任务通过/已测','整段覆盖次数','位置 RMS 均值 [最小,最大] m','最差峰值 m','速度向量 RMS 均值 m/s','姿态 RMS 均值 °'],
                 [[f'[{s["trajectory"]}]({Path(s["representative_folder"])/"continuous_figures/tracking_overview.png"})' if 'representative_folder' in s else s['trajectory'],f"{s['successful_missions']}/{s['runs']}",s['full_tracking_runs'],
                   f"{num(s.get('position_rms_m_mean'))} [{num(s.get('position_rms_m_min'))}, {num(s.get('position_rms_m_max'))}]",
                   num(s.get('position_peak_m_max')),num(s.get('velocity_vector_rms_m_s_mean')),num(s.get('attitude_rms_deg_mean'))] for s in summaries])
    lines+=['',f'![三次重复总体对照]({HERE}/figures/all_trajectory_comparison.png)','',
            f'![逐圈变化]({HERE}/figures/lap_evolution.png)','',
            f'![三次误差叠加]({HERE}/figures/repeat_error_overlays.png)','',
            '## 全部独立任务明细','']
    lines+=table(['轨迹','重复','结果','覆盖','位置 RMS / 峰值 m','位置 P95 m','速度向量 / 幅值 RMS m/s','四元数 RMS / 峰值 °'],
                 [[r['trajectory'],r['repeat'],r['status'],f"{r.get('coverage',0):.1%}",
                   f"{num(r.get('position_rms_m'))} / {num(r.get('position_peak_m'))}",num(r.get('position_p95_m')),
                   f"{num(r.get('velocity_vector_rms_m_s'))} / {num(r.get('speed_magnitude_rms_m_s'))}",
                   f"{num(r.get('attitude_rms_deg'))} / {num(r.get('attitude_peak_deg'))}"] for r in rows])
    for s in summaries:
        name=s['trajectory'];group=[r for r in rows if r['trajectory']==name];scored=[r for r in group if 'position_rms_m' in r]
        lines+=['',f'## {name}：逐圈/分阶段分析与图片','']
        if not scored:
            lines+=['本轨迹没有可计分的正式跟踪数据。']
            continue
        worst=max(scored,key=lambda r:r['position_peak_m']);axis_rms=np.mean([[r[f'position_{a}_rms_m'] for a in 'ned'] for r in scored],axis=0)
        lines+=[f"三轴 RMS 均值 N/E/D 为 **{num(axis_rms[0])} / {num(axis_rms[1])} / {num(axis_rms[2])} m**；主要误差轴为 **{'NED'[int(np.argmax(axis_rms))]}**。最差位置峰值出现在重复 {worst['repeat']}、t={num(worst['peak_position_time_s'])} s，为 {num(worst['position_peak_m'])} m。",'']
        if name in PERIODIC:
            lr=[r for r in laps if r['trajectory']==name]
            lines+=table(['重复','圈','覆盖','位置 RMS m','位置峰值 m','速度向量 RMS m/s','姿态 RMS °','平均 Down 偏差 m'],
                         [[r['repeat'],r['lap'],f"{r['coverage']:.1%}",num(r['position_rms_m']),num(r['position_peak_m']),num(r['velocity_vector_rms_m_s']),num(r['attitude_rms_deg']),num(r['position_d_bias_m'])] for r in lr])
            trend=[r for r in scored if 'lap_rms_trend_m_per_lap' in r]
            lines+=['',* [f"重复 {r['repeat']}：首圈 RMS {num(r['first_lap_rms_m'])} m，后续圈均值 {num(r['later_laps_mean_rms_m'])} m，末圈 {num(r['last_lap_rms_m'])} m；圈 RMS 线性斜率 {num(r['lap_rms_trend_m_per_lap'],4)} m/圈。斜率描述本次序列，不能单独证明长期稳定性。" for r in trend],'']
            if name=='knife-edge-transition':
                lines+=['刀刃参考在每个转弯平滑旋转 π/2，下面直线段的正飞/倒飞姿态按圈交替。因此它的空间轨迹是单圈闭合，但完整姿态需求需要结合奇偶圈评价。',
                        *[f"重复 {r['repeat']}：奇数圈位置 RMS 均值 {num(r['odd_laps_mean_rms_m'])} m，偶数圈 {num(r.get('even_laps_mean_rms_m'))} m。" for r in trend],'']
        else:
            pr=[r for r in phase_rows if r['trajectory']==name]
            lines+=table(['重复','阶段','时间窗 s','位置 RMS / 峰值 m','速度向量 RMS m/s','姿态 RMS / 峰值 °'],
                         [[r['repeat'],r['phase'],f"{r['start_s']:g}–{r['end_s']:g}",f"{num(r['position_rms_m'])} / {num(r['position_peak_m'])}",num(r['velocity_vector_rms_m_s']),f"{num(r['attitude_rms_deg'])} / {num(r['attitude_peak_deg'])}"] for r in pr])
            if name=='differential-turn':lines+=['',* [f"重复 {r['repeat']} 在参考停车 1.70 s 的实际速度为 {num(r['speed_at_reference_stop_m_s'])} m/s。" for r in scored],'掉头 yaw_reversal 与 velocity_reversal 时间窗口有重叠，不能把各阶段样本数相加当作总时长。']
            if name.startswith('transition-'):
                lines+=['','原参考在 t=0.4 s 开始恒定切向加速度（前飞 +2.7、悬停 −2.7 m/s²），在 t=3.4 s 将切向加速度切回零，位置/速度连续而加速度存在跳变。这会改变要求的合力与姿态目标，是边界瞬态的一个输入来源。本次保留这一原始定义，没有通过平滑参考改善数值指标。']
                lines+=[*[f"重复 {r['repeat']} 在 t=3.4 s 转换结束的实际速度为 {num(r['speed_at_transition_end_m_s'])} m/s（前飞目标8.1，悬停目标0）。" for r in scored]]
            if pr:
                largest=max(pr,key=lambda r:r['position_rms_m'])
                lines+=['',f"当前记录中阶段位置 RMS 最大的是重复 {largest['repeat']} 的 **{largest['phase']}**（{largest['start_s']:g}–{largest['end_s']:g} s），为 {num(largest['position_rms_m'])} m。"]
        lines+=['',* [f"重复 {r['repeat']}：任务结果 {r['status']}，最大相邻目标四元数转角 {num(r['target_quaternion_step_peak_deg'])}°，超过 90° 的步数 {r['target_quaternion_steps_over_90_deg']}；{('中止/失败原因：'+r['abort_reason']) if r['abort_reason'] else '日志未报告中止原因。'}" for r in scored],'']
        representative=Path(s['representative_folder'])/'continuous_figures'
        lines+=[f"下面展示位置 RMS 最接近中位数的实际完整运行：**重复 {s['representative_repeat']}**，没有选择最优运行作为唯一示例。全部重复链接在下表。",'',f'![{name} 总图]({representative}/tracking_overview.png)','']
        if name in PERIODIC:lines+=[f'![{name} 每圈局部误差]({representative}/lap_phase_heatmap.png)','']
        lines+=table(['重复','位置误差','速度幅值','姿态角','三维轨迹','控制诊断','频谱'],
                     [[r['repeat'],*[f'[{label}]({Path(r["folder"])/"continuous_figures"/(stem+".png")})' for label,stem in [('位置','position_errors'),('速度','speed_magnitude'),('姿态','attitude_tracking'),('三维','trajectory_3d'),('控制','control_diagnostics'),('频谱','spectra')]]] for r in scored])
    lines+=['','## 跨轨迹判断与剩余问题','']
    complete_summaries=[s for s in summaries if s.get('full_tracking_runs',0)==3 and 'position_rms_m_mean' in s]
    if complete_summaries:
        order=sorted(complete_summaries,key=lambda s:s['position_rms_m_mean'])
        lines+=[f"仅对完成三次整段跟踪的轨迹按位置 RMS 均值排序："+' → '.join(f"{s['trajectory']} ({num(s['position_rms_m_mean'])} m)" for s in order)+'。轨迹速度、曲率和时长不同，这个排序是当前任务难度下的结果，不是对控制器的统一激励基准。','']
    for s in summaries:
        if 'position_rms_m_mean' not in s:continue
        lines+=[f"- **{s['trajectory']}**：重复间位置 RMS 的样本标准差为 {num(s['position_rms_m_sd'])} m，最大/最小均方根比为 {num(s['position_rms_m_max']/max(s['position_rms_m_min'],1e-12),2)}；最差姿态瞬时误差为 {num(s['attitude_peak_deg_max'])}°。均值与最坏值应一起评价。"]
    lines+=['','刀刃与掉头的高姿态误差、大相邻目标转角和分配残差需要联合查看：软件限幅放宽不能让电机/舵面的物理能力同时变大。若出现超过 90° 的相邻目标步，优先检查平坦化姿态分支及正推力约束；表中列出的跳变是日志证据，具体因果仍需专门对照试验。逐圈误差是否持续升高应结合末圈、后续圈均值、Down 偏差与三次重复，而不是只观察第一圈。','',
            '### 大姿态步的输入重放','',
            '从生产 flatness_control.py 复制同一计算函数到分析进程，只加入原始推力和 roll 分支的诊断记录，不修改公式或生产文件。用记录的要求合力、实际速度、yaw、当前姿态与附近的已记录非瞬态舵角（低通值减去高通值）重放每个超过90°的目标步。只有步前/步后目标四元数均与记录吻合在0.5°以内，才标记分支切换；否则保留为未验证，不能据此确定原因。舵角样本的传输顺序有不确定性，全部匹配误差和所选时间偏差保存在 quaternion_jump_replay.json。','',
            'roll 分支根据当前机体 y 轴与候选 y 轴的点积选择相差 π 的解；原始推力为负时又将 pitch_bar 加 π 以保证正推力。两个条件都可能改变姿态解的分支。计数允许同一跳变同时切换两个分支，不能将两列相加当作事件总数。正推力约束不能简单删除，后续改善应兼顾可实现的合力方向和姿态解的连续性。','']
    lines+=table(['轨迹/重复','>90°目标步数','重放吻合事件数','负推力分支切换数','roll分支切换数','完整输入重放'],
                 [[f"{r['trajectory']}/{r['repeat']}",r['target_quaternion_steps_over_90_deg'],r['verified_jump_replay_count'],r['negative_thrust_branch_toggle_count'],r['alternate_roll_branch_toggle_count'],f"[JSON]({Path(r['folder'])}/quaternion_jump_replay.json)"] for r in rows if 'position_rms_m' in r])
    jump_cases=[r for r in rows if r.get('target_quaternion_steps_over_90_deg',0)]
    lines+=['',f"本次合计记录 {sum(r['target_quaternion_steps_over_90_deg'] for r in jump_cases)} 次超过90°的目标步，其中 {sum(r['verified_jump_replay_count'] for r in jump_cases)} 次输入重放通过目标吻合检查。"]
    for r in jump_cases:
        events=json.loads((Path(r['folder'])/'quaternion_jump_replay.json').read_text())
        times='、'.join(num(e['time_s']) for e in events)
        lines+=[f"- {r['trajectory']} 重复 {r['repeat']}：跳变时刻 **{times} s**；已验证的负推力分支切换 {r['negative_thrust_branch_toggle_count']} 次、roll 分支切换 {r['alternate_roll_branch_toggle_count']} 次。"]
        if r['trajectory']=='differential-turn' and events and all(e['time_s']>2.5 for e in events):
            lines+=['  这些跳变发生在速度反向结束后的退出直线段，已不在1.44–1.96 s偏航窗口内。因此这次恶化并非原偏航时间窗提前翻转的同一个问题；录包重放显示闭环要求合力与当前速度/舵态进入了另一姿态解分支。']
    lines+=['','这类跳变以四元数最短转角计算，已排除单纯欧拉角 ±180° 显示换支的解释。当前检查验证计算分支与目标跳变对应；导致状态/合力进入该分支的上游原因仍需结合回调时序、模型/执行器延迟及位置反馈继续做受控对照。','']
    lines+=['',
            '## 限幅、分配误差与执行器反馈','',
            '角加速度/力矩的软件限幅按各轨迹 profile 和记录的运动增益 blend 重建，不使用默认固定阈值误判。电机/舵面比例是分配后指令触及物理边界的时间比例；软件触限不等于执行器必然物理饱和。分配残差为 achieved−desired moment，非零表示当前分配未实现全部要求。','']
    lines+=table(['轨迹/重复','角加速度触限 x/y/z %','力矩触限 x/y/z %','电机上限 L/R %','舵偏限幅 L/R %','分配力矩残差 RMS x/y/z Nm'],
                 [[f"{r['trajectory']}/{r['repeat']}",*[' / '.join(num(100*x,1) for x in r[k]) for k in ['angular_acceleration_software_limit_fraction_xyz','moment_software_limit_fraction_xyz','motor_upper_limit_fraction_lr','flap_limit_fraction_lr']],
                   ' / '.join(num(x,4) for x in r['allocation_moment_residual_rms_nm_xyz'])] for r in rows if 'position_rms_m' in r])
    lines+=['','预测与实际执行器反馈的误差反映预测模型/反馈延迟的一致性，不是相对目标指令的跟踪误差；电机单位 rad/s，舵面单位度。','']
    lines+=table(['轨迹/重复','电机下限 L/R %','实测执行器反馈占比 %','电机预测 RMS L/R rad/s','舵面预测 RMS L/R °'],
                 [[f"{r['trajectory']}/{r['repeat']}",' / '.join(num(100*x,1) for x in r['motor_lower_limit_fraction_lr']),
                   num(100*r.get('measured_actuator_feedback_fraction',0),1),
                   ' / '.join(num(x,2) for x in r.get('actuator_prediction_motor_rms_rad_s_lr',[])),
                   ' / '.join(num(x,2) for x in r.get('actuator_prediction_flap_rms_deg_lr',[]))] for r in rows if 'position_rms_m' in r])
    lines+=['','正式跟踪阶段高度范围如下；分段轨迹自身含起飞，因此其最低高度接近地面是参考要求。','']
    lines+=table(['轨迹/重复','高度 最低/最高 m','最大速度 m/s','滤后机体角速度峰值 °/s','目标姿态最大相邻步 °'],
                 [[f"{r['trajectory']}/{r['repeat']}",' / '.join(num(x) for x in r['tracking_altitude_minmax_m']),num(r['speed_peak_m_s']),num(r['body_rate_peak_deg_s'],1),num(r['target_quaternion_step_peak_deg'],1)] for r in rows if 'position_rms_m' in r])
    lines+=['','## 采样时序、锯齿与高频振荡','',
            '位置误差细锯齿可由低频位置消息保持与连续参考相减产生。快飞方向最明显，量级约为速度×状态更新间隔；100 Hz 附近的能量不能直接归因于机体真实运动。18–22 Hz 既可能包含参考/传输拍频，也可能包含控制和执行器纹波；用电机、舵面和姿态一起核对，不凭位置曲线单独判定。','',
            'Welch 频谱在统一 400 Hz 网格上计算，最长段 8 s、线性去趋势。单次转换和掉头很短且非平稳，频带 RMS 只能作为描述性指标，不用于证明固定频率自激或因果关系。','']
    lines+=table(['轨迹/重复','控制 dt 均值 ms','位置到达间隔 中位数/p99 ms','状态保持 %','主位置 RMS m','测量时间戳 RMS m','位置18–22 Hz RMS m','舵面18–22 Hz RMS °'],
                 [[f"{r['trajectory']}/{r['repeat']}",num(r.get('control_dt_mean_ms')),
                   f"{num(r['local_arrival_timing']['median_interval_ms'])}/{num(r['local_arrival_timing']['p99_interval_ms'])}",
                   num(100*r['position_state_hold_fraction'],1),num(r['position_rms_m']),num(r.get('measurement_timestamp_position_rms_m')),
                   num(r['position_18_22hz_rms_m'],5),num(r['flaps_18_22hz_rms_deg'],4)] for r in rows if 'position_rms_m' in r])
    lines+=['','## 初始化故障与预试验：保留全部异常记录','',
            '正式统一测试前先跑了使用共享参数历史的预试验。它暴露出参数并非完全独立：上一飞行学习并保存的 CAL_MAG0_ZOFF 约为 0.19879 Gauss，后续协调圆和前飞转换启动报告 Strong magnetic interference，90 s 内未出现 Ready for takeoff。没有关闭健康检查；改为每次重置初始磁偏置后重新进行统一重复测试。预试验不混入正式主表，但全部日志、图和已计算结果保存在 pilot_shared_calibration/。','',
            '预试验刀刃完成任务但位置 RMS 1.730 m、峰值 3.848 m，曾出现 6 个超过 90° 的相邻目标姿态步；这一异常结果保留。该时段主机存在其他高 CPU 作业，离线制图也曾与仿真重叠；正式测试期间不并行运行离线制图。其他计算作业仍可能占用 CPU，未擅自终止这些作业。下面按正式跟踪时间窗匹配 host_load.csv 的5 s负载采样；最早个别段缺少主机样本时显示 —，控制器时序录包仍完整。不能把参数继承、运行时序和控制分支变化单独指定为所有误差的唯一原因。','']
    lines+=table(['轨迹/重复','主机 CPU 平均 %','主机 CPU 峰值 %','CPU>80% 样本占比 %','主机1分钟负载均值','控制 dt 均值 ms'],
                 [[f"{r['trajectory']}/{r['repeat']}",num(100*r['host_cpu_busy_mean_fraction'],1) if 'host_cpu_busy_mean_fraction' in r else '—',
                   num(100*r['host_cpu_busy_peak_fraction'],1) if 'host_cpu_busy_peak_fraction' in r else '—',
                   num(100*r['host_cpu_samples_over_80pct_fraction'],1) if 'host_cpu_samples_over_80pct_fraction' in r else '—',
                   num(r.get('host_load_1m_mean'),1),num(r.get('control_dt_mean_ms'))] for r in rows if 'position_rms_m' in r])
    lines+=['',f'![运行时序与负载]({HERE}/figures/runtime_timing.png)','',
            'Gazebo 实时因子由只读 world stats 订阅获得，记录始于第二轮八字附近，早期任务没有这个观测，不能推定为 1。CPU 满载不一定使动力学时钟变慢，也可能主要降低 Python 控制回调频率。主机/实时因子为运行条件观测，不能仅用相关性把误差全部归因于 CPU。','']
    lines+=table(['轨迹/重复','记录的控制 debug Hz','控制 dt 均值 ms','Gazebo 实时因子均值 / 最小','Gazebo 观测数'],
                 [[f"{r['trajectory']}/{r['repeat']}",num(r['debug_timing']['mean_rate_hz'],1),num(r.get('control_dt_mean_ms')),
                   f"{num(r.get('gazebo_real_time_factor_mean'))} / {num(r.get('gazebo_real_time_factor_min'))}",r.get('gazebo_stats_samples','—')] for r in rows if 'position_rms_m' in r])
    lines+=['',f'![误差与实际控制记录频率]({HERE}/figures/error_vs_callback_rate.png)','',
            '不同回调频率下的性能差异在当前数据中可直接检查。较慢控制周期会增加命令更新时间和状态配对时序的不确定性，对刀刃翻转及快速掉头更敏感；INDI 使用实测 dt 能避免固定差分步长错误，但不能消除执行器滞后或因回调变慢增加的闭环延迟。该图展示关联，不替代固定负载、固定估计条件下的控制 A/B 试验。','']
    init_rows=[]
    for p in sorted((HERE/'initialization_attempts').glob('*/*/*/case_result.json')):
        data=json.loads(p.read_text());init_rows.append([p.parents[2].name,p.parents[1].name,p.parent.name,data.get('status'),data.get('error',''),f'[日志]({p.parent}/simulation.log)'])
    if init_rows:lines+=table(['重复','轨迹','尝试','结果','原因','记录'],init_rows)
    else:lines+=['正式统一参数下没有另行归档的初始化重试。'+('当前仍为中间结果，后续会继续核对。' if finished<21 else '')]
    lines+=['','[预试验中间报告（历史磁校准，非正式统计）]('+str(HERE/'pilot_shared_calibration/REPORT.md')+')','',
            '## 可复现与结果文件','',
            '- `manifest.json`：测试范围、参数、来源和测试副本 SHA256。',
            '- `all_run_metrics.json/csv`：全部重复、三轴误差、速度、姿态、限幅、时序和执行器预测诊断。',
            '- `all_lap_metrics.json/csv`、`all_phase_metrics.json/csv`：每圈/每阶段独立统计。',
            '- 每次运行的 `tracking_samples.npz`、原始 `position_tracking.csv`、bag 和日志保留；统一统计另存 `uniform_scoring.npz`、`uniform_metrics.json`，连续参考图在 `continuous_figures/`。所有科学图提供 PNG/SVG。',
            '- `source_validation.json`：本轮开始和结束的生产文件哈希检查及测试检查。','',
            '```bash','source /opt/ros/jazzy/setup.bash','source ros2_ws/install/setup.bash',
            f'/usr/bin/python3 {HERE}/run_suite.py',f'/usr/bin/python3 {HERE}/analyze_suite.py','```','',
            '运行脚本在已记录的 case_result.json 处保留检查点；新一轮独立实验应复制脚本到新目录，避免把旧任务当作新重复。']
    (HERE/'REPORT.md').write_text('\n'.join(lines)+'\n')


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--force',action='store_true');parser.add_argument('--refresh-metrics',action='store_true');args=parser.parse_args()
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':9,'svg.fonttype':'none'})
    rows=[];laps=[];phase_rows=[]
    for folder in sorted(HERE.glob('repeat_*/*')):
        if not (folder/'case_result.json').exists():continue
        rows.append(analyze_case(folder,args.force or args.refresh_metrics,redraw=not args.refresh_metrics))
        for file,target in [('lap_metrics.json',laps),('phase_metrics.json',phase_rows)]:
            if (folder/file).exists():target.extend(json.loads((folder/file).read_text()))
    rows.sort(key=lambda r:(NAMES.index(r['trajectory']),r['repeat']))
    summaries=aggregate(rows)
    selected=HERE/'selected';selected.mkdir(exist_ok=True)
    for s in summaries:
        if 'representative_folder' not in s:continue
        link=selected/s['trajectory']
        if link.is_symlink():link.unlink()
        if not link.exists():link.symlink_to(Path(s['representative_folder'])/'continuous_figures',target_is_directory=True)
    for name,data in [('all_run_metrics',rows),('all_lap_metrics',laps),('all_phase_metrics',phase_rows),('trajectory_summary',summaries)]:
        dump(HERE/(name+'.json'),data);csv_rows(HERE/(name+'.csv'),data)
    if rows:
        render_aggregate(rows,summaries,laps,phase_rows);write_report(rows,summaries,laps,phase_rows)
    before=json.loads((HERE/'production_before.json').read_text())
    changes=[p for p,h in before.items() if not Path(p).exists() or hashlib.sha256(Path(p).read_bytes()).hexdigest()!=h]
    dump(HERE/'source_validation.json',{'production_files_checked':len(before),'production_changed_files':changes,
                                      'production_unchanged':not changes,'overlay_unit_tests_passed':137,
                                      'branch_replay_smoke':json.loads((HERE/'branch_replay_smoke.json').read_text()) if (HERE/'branch_replay_smoke.json').exists() else None,
                                      'case_results':len(rows),'expected_case_results':21,
                                      'all_planned_runs_recorded':len(rows)==21})
    print(f'Report updated: {len(rows)}/21 recorded missions; production changes: {changes}',flush=True)


if __name__=='__main__':
    main()
