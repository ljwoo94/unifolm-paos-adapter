import numpy as np
import pytest
from unifolm_paos_adapter.projection import ArmGripperProjection
from unifolm_paos_adapter.chunk import ModelActionChunk
from unifolm_paos_adapter.kinematics import G1Kinematics
from scipy.spatial.transform import Rotation
from test_adapter import raw_chunk


def test_projection_uses_current_chunk_ee_and_explicit_gripper_endpoints():
    q=np.zeros(29);q[[18,25]]=.8
    k=G1Kinematics();ee=k.ee9(q);raw=raw_chunk()
    for side,v in ee.items():
        r=np.column_stack([v[3:6],v[6:9],np.cross(v[3:6],v[6:9])])
        raw[f'action.{side}_ee_rpy'][0,:,:]=np.r_[v[:3],Rotation.from_matrix(r).as_euler('xyz')]
    raw['action.left_gripper'][0,:,:]=2.7;raw['action.right_gripper'][0,:,:]=5.5
    chunk=ModelActionChunk.from_response(raw,received_at=0.,period_s=1/30)
    p=ArmGripperProjection({'left':[0.,5.4],'right':[0.,5.4]})
    value=p.targets(chunk,10,q)
    assert len(value['positions'])==14
    assert value['grippers']==pytest.approx({'left':.5,'right':1.})
    assert value['gripper_saturated']==['right']
    assert value['ik']['converged']
    assert value['ik']['backend']=='placo'
    assert chunk.actions['action.right_gripper'][0,10,0]==5.5
