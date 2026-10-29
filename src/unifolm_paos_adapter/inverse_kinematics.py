"""Experimental bounded arm IK over the unchanged official G1 URDF.

Targets are tool xyz + xyz-Euler radians in pelvis coordinates. This conversion
is not presented as the official UniFoLM deployment IK or a collision planner.
"""
import time
import numpy as np
from scipy.spatial.transform import Rotation
from .kinematics import G1Kinematics


class ArmIK:
    def __init__(self, *, iterations=45, position_tolerance_m=.003, rotation_tolerance_rad=.03):
        self.kinematics=G1Kinematics()
        self.names=self.kinematics.names[15:]
        self.iterations=iterations
        self.position_tolerance=position_tolerance_m
        self.rotation_tolerance=rotation_tolerance_rad
        joints=self.kinematics.model.joint_map
        self.lower=np.array([joints[n].limit.lower for n in self.names])
        self.upper=np.array([joints[n].limit.upper for n in self.names])

    def _state(self,q,side):
        k=self.kinematics;k.model.update_cfg(dict(zip(k.names,map(float,q))))
        transform=k.model.get_transform(frame_to=f'{side}_wrist_yaw_link',frame_from='pelvis')
        return (transform@k.offset)[:3],transform[:3,:3]

    def _jacobian(self,side,position):
        k=self.kinematics;columns=[]
        offset=0 if side=='left'else 7
        for name in self.names[offset:offset+7]:
            joint=k.model.joint_map[name]
            frame=k.model.get_transform(frame_to=joint.parent,frame_from='pelvis')@joint.origin
            axis=frame[:3,:3]@joint.axis
            columns.append(np.r_[np.cross(axis,position-frame[:3,3]),axis])
        return np.stack(columns,axis=1)

    def solve(self,q,targets):
        started=time.perf_counter();q=np.asarray(q,dtype=float).copy()
        if q.shape!=(29,)or not np.isfinite(q).all():raise ValueError('finite measured body29 required')
        if set(targets)!={'left','right'}:raise ValueError('both tool targets required')
        q[15:]=np.clip(q[15:],self.lower,self.upper)
        residuals={};weight=np.array([1.,1.,1.,.12,.12,.12])
        for side,offset in [('left',0),('right',7)]:
            target=np.asarray(targets[side],dtype=float)
            if target.shape!=(6,)or not np.isfinite(target).all():raise ValueError('finite xyz/rpy6 target required')
            rotation=Rotation.from_euler('xyz',target[3:]).as_matrix()
            region=slice(15+offset,22+offset)
            low,high=self.lower[offset:offset+7],self.upper[offset:offset+7]
            count=0
            for count in range(self.iterations):
                position,current=self._state(q,side)
                error=np.r_[target[:3]-position,Rotation.from_matrix(rotation@current.T).as_rotvec()]
                if np.linalg.norm(error[:3])<=self.position_tolerance and np.linalg.norm(error[3:])<=self.rotation_tolerance:break
                jac=self._jacobian(side,position)*weight[:,None]
                delta=jac.T@np.linalg.solve(jac@jac.T+np.eye(6)*.015**2,error*weight)
                delta*=min(1.,.15/max(np.max(np.abs(delta)),1e-12))
                cost=np.linalg.norm(error*weight);previous=q[region].copy();accepted=False
                for alpha in [1.,.5,.25]:
                    q[region]=np.clip(previous+alpha*delta,low,high)
                    p,r=self._state(q,side)
                    candidate=np.r_[target[:3]-p,Rotation.from_matrix(rotation@r.T).as_rotvec()]
                    if np.linalg.norm(candidate*weight)<cost-1e-10:
                        accepted=True;break
                if not accepted:
                    q[region]=previous;break
            p,r=self._state(q,side)
            residuals[side]=dict(position_error_m=float(np.linalg.norm(target[:3]-p)),
                rotation_error_rad=float(np.linalg.norm(Rotation.from_matrix(rotation@r.T).as_rotvec())),iterations=count+1)
        return dict(joints=q[15:].copy(),residuals=residuals,
            within_limits=bool(np.all(q[15:]>=self.lower)&np.all(q[15:]<=self.upper)),
            converged=all(v['position_error_m']<=self.position_tolerance and v['rotation_error_rad']<=self.rotation_tolerance for v in residuals.values()),
            compute_s=time.perf_counter()-started)
