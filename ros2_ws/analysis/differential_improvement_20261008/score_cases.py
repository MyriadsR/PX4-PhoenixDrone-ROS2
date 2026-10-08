#!/usr/bin/env python3
"""Score full tracking and a shared position window; label changed yaw explicitly."""
import json,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parent))
from compare_tracking_improvements import load_case, hold

cache=ROOT/'comparison';cache.mkdir(exist_ok=True)
roots={'original':ROOT.parent/'maneuver_improvement_20261004/fresh_baseline'}
roots.update({p.name:p for p in sorted(ROOT.glob('sitl_*')) if (p/'differential-turn/summary.json').exists()})
metrics={};data={}
for label,root in roots.items():
    folder=root/'differential-turn'
    m,d=load_case(folder,cache,label)
    result=json.loads((folder/'case_result.json').read_text())
    scales={key:1. for key in ('maneuver_acceleration_limit_scale','maneuver_moment_limit_scale')}
    for argument in result.get('launch_arguments',[]):
        key,value=argument.split(':=',1)
        if key in scales:scales[key]=float(value)
    blend=hold(d['diagnostics']['reference'][:,0],d['diagnostics']['reference'][:,5:6],d['grid'])[:,0]
    limit=np.array([20.,15.,24.])*(1+blend[:,None]*(scales['maneuver_acceleration_limit_scale']-1))
    m['angular_acceleration_saturation_fraction']=np.mean(np.abs(d['control'][:,9:12])>=limit*.999,axis=0).tolist()
    m.update(mission_passed=result['mission']['passed'],
             reference_variant='centered-yaw' if result['mission'].get('differential_yaw_centered',False) else 'original',
             actual_speed_at_stop_m_s=float(np.linalg.norm([np.interp(1.7,d['p'][:,0],d['p'][:,7+i]) for i in range(3)])),
             comparison_to_original_end_s=2.049,
             full_original_position_trace_preserved=True)
    mask=d['p'][:,0]<=2.049
    m['common_window_position_rms_m']=float(np.sqrt(np.mean(np.sum(d['error'][mask]**2,axis=1))))
    metrics[label]=m;data[label]=d
    print(label,'full continuous RMS',round(m['position_rmse_continuous_m'],3),'peak',round(m['position_peak_continuous_m'],3),
           'common',round(m['common_window_position_rms_m'],3),'pass',m['mission_passed'],flush=True)
(ROOT/'sitl_metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
fig,axes=plt.subplots(3,1,figsize=(12,9),layout='constrained')
for label,d in data.items():
    if label in ('sitl_inner125_limits8','sitl_inner125_repeat1','sitl_inner150_limits16'):
        continue
    axes[0].plot(d['p'][:,0],np.linalg.norm(d['error'],axis=1),label=label)
    axes[1].plot(d['p'][:,0],np.linalg.norm(d['p'][:,7:10],axis=1),label=label)
    axes[2].plot(d['grid'],np.linalg.norm(d['control'][:,:3],axis=1)*180/np.pi,label=label)
axes[0].set(ylabel='Position error (m)',title='Full trajectory; original baseline aborts at 2.049 s')
axes[1].set(ylabel='Speed (m/s)')
axes[2].set(ylabel='Attitude command error (deg)')
for ax in axes:ax.grid(alpha=.2);ax.legend(fontsize=8);ax.set_xlabel('Tracking time (s)');ax.set_xlim(0,3.3)
fig.suptitle('Differential-turn: translation unchanged, candidate yaw centered at velocity reversal')
for ext in ('png','svg'):fig.savefig(cache/f'sitl_comparison.{ext}',dpi=170)
plt.close(fig)
