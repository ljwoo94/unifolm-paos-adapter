"""URDF-derived numeric arm chain; no scene-graph updates in the IK loop."""
import math
import numpy as np


class ArmChain:
    def __init__(self,kinematics,side):
        by_child={joint.child:joint for joint in kinematics.model.joint_map.values()}
        joints=[];link=f'{side}_wrist_yaw_link'
        while link!='pelvis':
            joint=by_child[link];joints.append(joint);link=joint.parent
        joints.reverse()
        indices={name:i for i,name in enumerate(kinematics.names)}
        self.entries=[]
        for joint in joints:
            if joint.type not in ('fixed','revolute') or joint.mimic is not None:
                raise ValueError('expected original fixed/revolute G1 chain without mimic joints')
            origin=np.eye(4) if joint.origin is None else np.asarray(joint.origin)
            index=indices[joint.name] if joint.type=='revolute' else None
            axis=np.asarray(joint.axis,dtype=float)
            axis=axis/np.linalg.norm(axis) if index is not None else np.zeros(3)
            x,y,z=axis;skew=np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
            self.entries.append((origin[:3,:3].copy(),origin[:3,3].copy(),index,axis,skew,skew@skew))
        self.split=next(i for i,e in enumerate(self.entries)if e[2] is not None and e[2]>=15)
        self.prefix_indices=tuple(e[2] for e in self.entries[:self.split]if e[2] is not None)
        arm_indices=[e[2]for e in self.entries[self.split:]if e[2] is not None]
        expected=list(range(15 if side=='left'else 22,22 if side=='left'else 29))
        if arm_indices!=expected:raise ValueError('chain must contain the declared seven arm joints in order')
        self.tool=kinematics.offset[:3].copy();self.identity=np.eye(3)
        self.prefix_key=None

    def _advance(self,rotation,position,entry,q):
        origin_r,origin_p,index,axis,skew,squared=entry
        position=position+rotation@origin_p
        rotation=rotation@origin_r
        world_axis=rotation@axis
        if index is not None:
            angle=float(q[index])
            rotation=rotation@(self.identity+math.sin(angle)*skew+(1-math.cos(angle))*squared)
        return rotation,position,world_axis

    def evaluate(self,q):
        key=tuple(float(q[i])for i in self.prefix_indices)
        if key!=self.prefix_key:
            r,p=self.identity,np.zeros(3)
            for entry in self.entries[:self.split]:r,p,_=self._advance(r,p,entry,q)
            self.prefix_rotation,self.prefix_position,self.prefix_key=r,p,key
        r,p=self.prefix_rotation,self.prefix_position
        origins=[];axes=[]
        for entry in self.entries[self.split:]:
            r,p,axis=self._advance(r,p,entry,q)
            if entry[2] is not None:origins.append(p);axes.append(axis)
        position=p+r@self.tool
        axes=np.asarray(axes);origins=np.asarray(origins)
        jacobian=np.concatenate([np.cross(axes,position-origins).T,axes.T],axis=0)
        return position,r,jacobian
