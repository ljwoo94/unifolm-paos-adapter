import numpy as np
from scipy.spatial.transform import Rotation
from unifolm_paos_adapter.inverse_kinematics import ArmIK
from unifolm_paos_adapter.kinematics import G1Kinematics


def test_placo_matches_original_urdf_with_measured_waist_and_tool_offset():
    solver=ArmIK();reference=G1Kinematics();rng=np.random.default_rng(5)
    for _ in range(12):
        q=rng.uniform(-.15,.15,29)
        q[15:]=rng.uniform(solver.lower*.25,solver.upper*.25)
        reference.model.update_cfg(dict(zip(reference.names,q)))
        for side in ['left','right']:
            position,rotation,jac=solver._arm_state(q,side)
            expected=reference.model.get_transform(frame_to=f'{side}_wrist_yaw_link',frame_from='pelvis')
            np.testing.assert_allclose(position,(expected@reference.offset)[:3],atol=1e-12)
            np.testing.assert_allclose(rotation,expected[:3,:3],atol=1e-12)
            assert jac.shape==(6,7)


def test_geometric_jacobian_matches_independent_finite_difference():
    solver=ArmIK();reference=G1Kinematics();q=np.zeros(29);q[12:15]=[.2,-.1,.15]
    q[15:]=[.2,.15,-.1,.6,.1,.15,-.1,.2,-.15,.1,.6,-.1,.15,.1]
    for side,start in [('left',15),('right',22)]:
        _,_,jac=solver._arm_state(q,side)
        def pose(values):
            reference.model.update_cfg(dict(zip(reference.names,values)))
            t=reference.model.get_transform(frame_to=f'{side}_wrist_yaw_link',frame_from='pelvis')
            return (t@reference.offset)[:3],t[:3,:3]
        for i in range(7):
            plus=q.copy();minus=q.copy();plus[start+i]+=1e-6;minus[start+i]-=1e-6
            p1,r1=pose(plus);p0,r0=pose(minus)
            numerical=np.r_[(p1-p0)/2e-6,Rotation.from_matrix(r1@r0.T).as_rotvec()/2e-6]
            np.testing.assert_allclose(jac[:,i],numerical,atol=2e-9)
