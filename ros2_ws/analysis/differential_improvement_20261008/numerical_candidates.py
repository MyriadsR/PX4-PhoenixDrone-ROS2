#!/usr/bin/env python3
"""Process-local numerical candidates; preserve the original project and logs."""
from pathlib import Path
import sys,json
import numpy as np
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parent/'differential_diagnosis_20261008'))
import diagnose as d

CONFIG=d.CONFIG_ORIGINAL
TRAJECTORY=d.TRAJECTORY_ORIGINAL
PD=d.sim.PDAttitudeController.compute_angular_acceleration_command
FLAT=d.sim.FlatnessAttitudeController.compute_desired_attitude_and_thrust
MOMENT=d.sim.INDIAngularController.compute_moment_command
d.OUT=ROOT/'numerical'
d.OUT.mkdir(exist_ok=True)

def run(name, *, center=1.7, inner=1., outer=1., transport=True, alpha_ff=0., limit_scale=None):
    last={}
    def config():
        cfg=CONFIG()
        for axis in ('roll','pitch','yaw'):
            setattr(cfg,'k_xi_'+axis,getattr(cfg,'k_xi_'+axis)*inner**2)
            setattr(cfg,'k_omega_'+axis,getattr(cfg,'k_omega_'+axis)*inner)
        for axis in ('x','y','z'):
            setattr(cfg,'kx_'+axis,getattr(cfg,'kx_'+axis)*outer**2)
            setattr(cfg,'kv_'+axis,getattr(cfg,'kv_'+axis)*outer)
        return cfg
    def trajectory(**kwargs):
        tr=TRAJECTORY(**kwargs);sample=tr.sample
        def centered(t):
            p,v,a,j,_,_=sample(t)
            u=np.clip((t-(center-.26))/.52,0.,1.)
            s=10*u**3-15*u**4+6*u**5
            ds=30*u**2*(1-u)**2/.52
            return p,v,a,j,np.pi/2+np.pi*s,np.pi*ds
        tr.sample=centered
        return tr
    def flat(self,*args,**kwargs):
        result=FLAT(self,*args,**kwargs)
        if kwargs.get('return_angles',False):last['qref']=result[0]
        return result
    def pd(self,q,qc,w,wr):
        transform=d.rotation(q).inv()*d.rotation(last['qref'])
        target=transform.apply(wr) if transport else wr
        command=PD(self,q,qc,w,target)
        if alpha_ff:
            # Test only: derivative in the reference body frame, then transport.
            dt=1./self.cfg.control_rate_hz
            previous=last.get('previous_wr',wr)
            dw=(wr-previous)/dt
            last['previous_wr']=wr.copy()
            command+=alpha_ff*(transform.apply(dw)-np.cross(w,target))
        if limit_scale is not None:
            command=np.clip(command,-np.array([20.,15.,24.])*limit_scale,
                            np.array([20.,15.,24.])*limit_scale)
        return command
    def moment(self,*args):
        value=MOMENT(self,*args)
        if limit_scale is not None:
            value=np.clip(value,-np.array([.25,.08,.35])*limit_scale,
                          np.array([.25,.08,.35])*limit_scale)
        return value
    d.CONFIG_ORIGINAL=config;d.TRAJECTORY_ORIGINAL=trajectory
    d.sim.PDAttitudeController.compute_angular_acceleration_command=pd
    d.sim.FlatnessAttitudeController.compute_desired_attitude_and_thrust=flat
    d.sim.INDIAngularController.compute_moment_command=moment
    try:
        m=d.run(name)
    finally:
        d.CONFIG_ORIGINAL=CONFIG;d.TRAJECTORY_ORIGINAL=TRAJECTORY
        d.sim.PDAttitudeController.compute_angular_acceleration_command=PD
        d.sim.FlatnessAttitudeController.compute_desired_attitude_and_thrust=FLAT
        d.sim.INDIAngularController.compute_moment_command=MOMENT
    m['candidate_parameters']=dict(center_s=center,inner_bandwidth_scale=inner,
         outer_bandwidth_scale=outer,transport_rates=transport,angular_acceleration_ff_scale=alpha_ff,
         software_limit_scale=limit_scale)
    m['reference_changed']=True
    (d.OUT/name/'metrics.json').write_text(json.dumps(m,indent=2)+'\n')
    return m

if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('--limit-cases',action='store_true')
    args=parser.parse_args()
    candidates={
        'centered_legacy_rate':dict(transport=False),
        'centered_transport':{},
        'centered_inner125':dict(inner=1.25),
        'centered_inner150':dict(inner=1.5),
        'centered_inner200':dict(inner=2.),
        'centered_outer150':dict(outer=1.5),
        'centered_outer200':dict(outer=2.),
        'centered_alpha_ff':dict(alpha_ff=1.),
        'centered_alpha_ff_half':dict(alpha_ff=.5),
        'centered_inner150_alpha_ff':dict(inner=1.5,alpha_ff=1.),
        'centered_inner150_outer150':dict(inner=1.5,outer=1.5),
        'center_160':dict(center=1.6),
        'center_165':dict(center=1.65),
        'center_175':dict(center=1.75),
    }
    if args.limit_cases:
        candidates={'centered_limits4':dict(limit_scale=4.),
                    'centered_limits8':dict(limit_scale=8.),
                    'centered_inner125_limits8':dict(inner=1.25,limit_scale=8.),
                    'centered_inner150_limits16':dict(inner=1.5,limit_scale=16.)}
    path=ROOT/'numerical_metrics.json'
    results=json.loads(path.read_text()) if path.exists() else {}
    for name,params in candidates.items():
        results[name]=run(name,**params)
        (ROOT/'numerical_metrics.json').write_text(json.dumps(results,indent=2)+'\n')
