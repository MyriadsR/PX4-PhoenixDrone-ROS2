#!/usr/bin/env python3
"""Render continuous-reference scientific plots, keeping recorded samples intact."""
import argparse,json,sys
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parent))
import plot_tracking_details as plot
from compare_tracking_improvements import load_case

parser=argparse.ArgumentParser();parser.add_argument('folder',type=Path)
args=parser.parse_args();folder=args.folder.resolve()
cache=ROOT/'comparison';cache.mkdir(exist_ok=True)
metrics,data=load_case(folder,cache,'figure_'+folder.parent.name)
metrics.pop('angular_acceleration_saturation_fraction',None)
stored=json.loads((folder/'summary.json').read_text());mission=stored['mission']
z=np.load(folder/'tracking_samples.npz');p=z['position_debug'].copy();att=z['attitude_debug']
p[:,4:7]=p[:,1:4]-data['error']
from phoenix_tailsitter_control.paper_trajectory_mission import make_reference
tr=make_reference('differential-turn')
p[:,10:13]=np.array([tr.sample(float(t))[1] for t in p[:,0]])
actual,target=plot.euler_pairs(att);duration=min(stored['tracking_duration_s'],3.3)
lap=mission['lap_time_s'];origin=mission['takeoff_origin_ned_m'][2]
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'svg.fonttype':'none'})
title=('Differential-turn | 7 m/s | yaw reversal centered at velocity zero\n'
       f'Continuous position reference | RMS {metrics["position_rmse_continuous_m"]:.3f} m | '
       f'peak {metrics["position_peak_continuous_m"]:.3f} m')

def trajectory3d(ax):
    plot.plot_3d(ax,p,10.,origin)
    ax.set_box_aspect([2.3,4.,2.5])
    for axis in (ax.xaxis,ax.yaxis,ax.zaxis):axis.set_major_locator(MaxNLocator(nbins=3))
    ax.tick_params(labelsize=8,pad=0)
    for text in list(ax.texts):text.remove()
    ax.text2D(.01,.94,'Display aspect adjusted; all axes in metres',transform=ax.transAxes,fontsize=8,color='#66727D')

fig,axes=plt.subplots(3,1,figsize=(11,7.5),sharex=True,layout='constrained')
plot.plot_errors(axes,p,lap,duration);fig.suptitle(title);plot.save(fig,folder,'position_errors')
fig,ax=plt.subplots(figsize=(11,4),layout='constrained');plot.plot_speed(ax,p,lap,duration)
fig.suptitle(title);plot.save(fig,folder,'speed_magnitude')
fig,axes=plt.subplots(3,1,figsize=(11,8),sharex=True,layout='constrained')
plot.plot_attitude(axes,att,actual,target,lap,duration);fig.suptitle(title+'\nAttitude: actual / controller command, intrinsic ZXY')
plot.save(fig,folder,'attitude_tracking')
fig=plt.figure(figsize=(10,8),layout='constrained');trajectory3d(fig.add_subplot(projection='3d'))
fig.suptitle(title);plot.save(fig,folder,'trajectory_3d')
fig=plt.figure(figsize=(16,12.5),layout='constrained');grid=fig.add_gridspec(2,2,height_ratios=[1.5,1.2])
left=grid[0,0].subgridspec(3,1);right=grid[0,1].subgridspec(3,1)
plot.plot_errors([fig.add_subplot(left[i]) for i in range(3)],p,lap,duration)
plot.plot_attitude([fig.add_subplot(right[i]) for i in range(3)],att,actual,target,lap,duration)
plot.plot_speed(fig.add_subplot(grid[1,0]),p,lap,duration);trajectory3d(fig.add_subplot(grid[1,1],projection='3d'))
fig.suptitle(title+'\nAttitude: actual / controller command | physical actuator limits retained',fontsize=14)
plot.save(fig,folder,'tracking_overview')
full=p[:,0]<=3.3
metrics['position_axis_rmse_continuous_m']=np.sqrt(np.mean((p[full,1:4]-p[full,4:7])**2,axis=0)).tolist()
metrics['velocity_magnitude_rmse_continuous_m_s']=float(np.sqrt(np.mean((np.linalg.norm(p[full,7:10],axis=1)-np.linalg.norm(p[full,10:13],axis=1))**2)))
(folder/'continuous_figure_metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
np.savetxt(folder/'continuous_position_tracking.csv',p[:,:13],delimiter=',',
 header='time_s,n_m,e_m,d_m,n_ref_m,e_ref_m,d_ref_m,vn_m_s,ve_m_s,vd_m_s,vn_ref_m_s,ve_ref_m_s,vd_ref_m_s',comments='')
print(title)
