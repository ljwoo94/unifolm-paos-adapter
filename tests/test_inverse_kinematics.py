import numpy as np
from scipy.spatial.transform import Rotation
from unifolm_paos_adapter.kinematics import G1Kinematics
from unifolm_paos_adapter.inverse_kinematics import ArmIK


def test_measured_full_chain_reachable_tool_pose_and_joint_limits():
    k=G1Kinematics();target_q=np.zeros(29)
    target_q[12:15]=[.15,-.05,.08]
    target_q[15:22]=[.1,.15,-.1,.8,.1,.05,-.1]
    target_q[22:29]=[.1,-.15,.1,.8,-.1,.05,.1]
    ee=k.ee9(target_q)
    targets={side:np.r_[v[:3],Rotation.from_matrix(np.column_stack([v[3:6],v[6:9],np.cross(v[3:6],v[6:9])])).as_euler('xyz')]for side,v in ee.items()}
    start=target_q.copy();start[15:]+=np.tile([.08,-.05,.02,-.1,.05,.02,-.04],2)
    solver=ArmIK()
    result=solver.solve(start,targets)
    assert result['joints'].shape==(14,)
    assert max(v['position_error_m']for v in result['residuals'].values())<.005
    assert max(v['rotation_error_rad']for v in result['residuals'].values())<.05
    assert result['within_limits']
    assert result['backend']=='placo'
    # The model-side arm solve must not move the measured lower body or waist.
    for name,value in zip(k.names[:15],start[:15]):
        assert solver.robot.get_joint(name)==value
    # Infeasible target remains bounded and is reported rather than fabricated as solved.
    targets['left'][:3]=[5.,5.,5.]
    failed=ArmIK().solve(start,targets)
    assert not failed['converged'] and failed['within_limits']
    assert np.isfinite(failed['joints']).all()


def test_reused_solver_resets_all_measured_joints_and_reports_shared_qp_count():
    solver=ArmIK();kin=G1Kinematics()
    for waist in [[.2,-.1,.15],[-.1,.15,-.2]]:
        q=np.zeros(29);q[12:15]=waist;q[[18,25]]=.8
        ee=kin.ee9(q)
        targets={side:np.r_[v[:3],Rotation.from_matrix(np.column_stack([v[3:6],v[6:9],np.cross(v[3:6],v[6:9])])).as_euler('xyz')]for side,v in ee.items()}
        result=solver.solve(q,targets)
        assert result['converged'] and result['qp_solves']==0
        np.testing.assert_allclose(result['joints'],q[15:],atol=1e-12)
        for name,value in zip(kin.names,q):assert solver.robot.get_joint(name)==value
        assert all(v['iterations']==result['qp_solves']for v in result['residuals'].values())


def test_invalid_model_target_rejected_before_any_qp_step():
    import pytest
    solver=ArmIK();targets={'left':np.zeros(6),'right':np.zeros(6)}
    targets['right'][2]=np.nan
    with pytest.raises(ValueError,match='finite xyz/rpy6'):
        solver.solve(np.zeros(29),targets)


def test_measured_waist_outside_nominal_limit_does_not_block_arm_only_solve():
    solver=ArmIK();kin=G1Kinematics()
    q=np.zeros(29);q[14]=.53;q[[18,25]]=.8
    truth=q.copy();truth[15]+=.05
    ee=kin.ee9(truth)
    targets={side:np.r_[v[:3],Rotation.from_matrix(np.column_stack([v[3:6],v[6:9],np.cross(v[3:6],v[6:9])])).as_euler('xyz')]for side,v in ee.items()}
    result=solver.solve(q,targets)
    assert result['converged'] and result['within_limits']
    for name,value in zip(kin.names[:15],q[:15]):assert solver.robot.get_joint(name)==value
    # Arm constraints remain the original URDF limits.
    for i,name in enumerate(solver.names):
        np.testing.assert_array_equal(solver.robot.get_joint_limits(name),[solver.lower[i],solver.upper[i]])
