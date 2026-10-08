"""Physical consistency and isolation of the opt-in differential-turn variant."""
import numpy as np
from scipy.spatial.transform import Rotation
from phoenix_tailsitter_control.paper_trajectory_mission import make_reference, PaperTrajectoryAdapter
from phoenix_tailsitter_control.config import PhoenixHoverConfig
from phoenix_tailsitter_control.flatness_control import FlatnessAttitudeController


def test_centering_preserves_every_translational_derivative_and_join():
    original=make_reference('differential-turn')
    variant=make_reference('differential-turn',differential_yaw_centered=True)
    for t in np.linspace(0,original.total_duration,601):
        before,after=original.sample(t),variant.sample(t)
        for i in range(4):np.testing.assert_array_equal(before[i],after[i])
    assert np.linalg.norm(variant.sample(1.7)[1])<1.e-10
    assert abs(variant.sample(1.7)[4]-np.pi)<1.e-10
    for t in (1.44,1.7,1.96):
        h=1.e-5
        left,mid,right=[variant.sample(t+delta) for delta in (-h,0,h)]
        np.testing.assert_allclose((right[4]-left[4])/(2*h),mid[5],atol=1.e-6)
    adapter=PaperTrajectoryAdapter('differential-turn',differential_yaw_centered=True)
    for sample,target in ((adapter.entry_sample(4,4),adapter.sample(0)),
                          (adapter.exit_sample(adapter.total_duration,0,4),adapter.sample(adapter.total_duration))):
        for field in ('position','velocity','acceleration','jerk','yaw','yawspeed'):
            np.testing.assert_allclose(getattr(sample,field),getattr(target,field),atol=1.e-8)


def test_centered_nominal_path_has_finite_continuous_attitude_and_rate():
    cfg=PhoenixHoverConfig();flat=FlatnessAttitudeController(cfg)
    tr=make_reference('differential-turn',differential_yaw_centered=True)
    previous=np.array([1.,0.,0.,0.]);qs=[];rates=[]
    for t in np.arange(0,3.3001,.002):
        _,v,a,j,yaw,rate=tr.sample(t)
        force=cfg.mass*(a-[0,0,cfg.gravity])
        q,total,roll,pitch=flat.attitude_and_thrust(force,v,-.1107,yaw,previous,return_angles=True)
        assert total>0
        qs.append(q)
        rates.append(flat.feedforward_rates(v,a,j,yaw,rate,force,roll,pitch,-.1107))
        previous=q
    qs=np.array(qs);r=Rotation.from_quat(qs[:,[1,2,3,0]])
    assert np.rad2deg((r[:-1].inv()*r[1:]).magnitude().max())<2.
    assert np.rad2deg(np.linalg.norm(rates,axis=1).max())<750.
