"""Experimental model tool-pose/gripper conversion to public PAOS goals."""
import numpy as np
from .inverse_kinematics import ArmIK


class ArmGripperProjection:
    def __init__(self,endpoints):
        self.solver=ArmIK();self.endpoints=endpoints
        for side in ['left','right']:
            lo,hi=endpoints[side]
            if not np.isfinite([lo,hi]).all()or not lo<hi:raise ValueError('explicit native-rad device endpoints required')

    def targets(self,chunk,index,q):
        targets={side:chunk.actions[f'action.{side}_ee_rpy'][0,index]for side in ['left','right']}
        ik=self.solver.solve(q,targets)
        positions=dict(zip(self.solver.names,map(float,ik.pop('joints'))))
        grippers={};saturated=[]
        for side in ['left','right']:
            native=float(chunk.actions[f'action.{side}_gripper'][0,index,0])
            low,high=self.endpoints[side];opening=(native-low)/(high-low)
            if not 0<=opening<=1:saturated.append(side)
            grippers[side]=float(np.clip(opening,0,1))
        return dict(positions=positions,grippers=grippers,gripper_saturated=saturated,ik=ik)
