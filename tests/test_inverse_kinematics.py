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
    result=ArmIK().solve(start,targets)
    assert result['joints'].shape==(14,)
    assert max(v['position_error_m']for v in result['residuals'].values())<.005
    assert max(v['rotation_error_rad']for v in result['residuals'].values())<.05
    assert result['within_limits']
    # Infeasible target remains bounded and is reported rather than fabricated as solved.
    targets['left'][:3]=[5.,5.,5.]
    failed=ArmIK().solve(start,targets)
    assert not failed['converged'] and failed['within_limits']
    assert np.isfinite(failed['joints']).all()
