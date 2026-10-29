"""PlaCo arm IK: measured waist, pelvis-relative tool poses, bounded joint goals.

Only arm14 is optimized. This does not replace PAOS WBC or plan collisions.
"""
import time
import xml.etree.ElementTree as ET
import numpy as np
import placo
from scipy.spatial.transform import Rotation
from .kinematics import G1Kinematics


class ArmIK:
    def __init__(self, *, iterations=45, position_tolerance_m=.003, rotation_tolerance_rad=.03):
        if (type(iterations)is not int or iterations<1 or
                not np.isfinite([position_tolerance_m,rotation_tolerance_rad]).all() or
                min(position_tolerance_m,rotation_tolerance_rad)<=0):
            raise ValueError('positive IK iteration/tolerance settings required')
        self.kinematics=G1Kinematics();self.names=self.kinematics.names[15:]
        self.iterations=iterations;self.position_tolerance=position_tolerance_m
        self.rotation_tolerance=rotation_tolerance_rad
        joints=self.kinematics.model.joint_map
        self.lower=np.array([joints[n].limit.lower for n in self.names])
        self.upper=np.array([joints[n].limit.upper for n in self.names])
        tree=ET.parse(self.kinematics.path);root=tree.getroot()
        # Geometry is unused by this kinematic solve; original file is unchanged.
        # The source package does not contain the URDF's visual/collision meshes.
        for link in root.findall('link'):
            for tag in ('visual','collision'):
                for node in link.findall(tag):link.remove(node)
        for side in ('left','right'):
            ET.SubElement(root,'link',name=side+'_paos_tool')
            joint=ET.SubElement(root,'joint',name=side+'_paos_tool_fixed',type='fixed')
            ET.SubElement(joint,'parent',link=side+'_wrist_yaw_link')
            ET.SubElement(joint,'child',link=side+'_paos_tool')
            ET.SubElement(joint,'origin',xyz='0.105 0 0',rpy='0 0 0')
        self.robot=placo.RobotWrapper(str(self.kinematics.path.parent),placo.Flags.ignore_collisions,
                                      ET.tostring(root,encoding='unicode'))
        self.solver=placo.KinematicsSolver(self.robot);self.solver.mask_fbase(True)
        for name in self.kinematics.names[:15]:self.solver.mask_dof(name)
        self.solver.enable_joint_limits(True);self.solver.enable_velocity_limits(False)
        self.solver.add_regularization_task(.015**2)
        self.tasks={}
        for side in ('left','right'):
            task=self.solver.add_frame_task(side+'_paos_tool',np.eye(4))
            task.configure(side,'soft',1.,.12**2);self.tasks[side]=task
        self.velocity_indices={side:[self.robot.get_joint_v_offset(n)for n in self.names[start:start+7]]
                               for side,start in [('left',0),('right',7)]}

    def _reset(self,q):
        self.robot.set_T_world_fbase(np.eye(4))
        # Masked lower15 are measured inputs, not decision variables. A sensor
        # overshoot beyond nominal URDF bounds must not make the arm QP infeasible.
        # Expand only their in-memory bounds to include this exact measurement.
        for name,value in zip(self.kinematics.names[:15],q[:15]):
            limit=self.kinematics.model.joint_map[name].limit
            self.robot.set_joint_limits(name,min(limit.lower,float(value)),max(limit.upper,float(value)))
        for name,value in zip(self.kinematics.names,q):self.robot.set_joint(name,float(value))
        self.robot.update_kinematics()

    def _arm_state(self,q,side):
        self._reset(q)
        transform=self.robot.get_T_world_frame(side+'_paos_tool')
        jac=self.robot.frame_jacobian(side+'_paos_tool','local_world_aligned')
        return transform[:3,3].copy(),transform[:3,:3].copy(),jac[:,self.velocity_indices[side]].copy()

    def solve(self,q,targets):
        started=time.perf_counter();q=np.asarray(q,dtype=float).copy()
        if q.shape!=(29,)or not np.isfinite(q).all():raise ValueError('finite measured body29 required')
        if set(targets)!={'left','right'}:raise ValueError('both tool targets required')
        desired={}
        for side in ('left','right'):
            target=np.asarray(targets[side],dtype=float)
            if target.shape!=(6,)or not np.isfinite(target).all():raise ValueError('finite xyz/rpy6 target required')
            transform=np.eye(4);transform[:3,3]=target[:3]
            transform[:3,:3]=Rotation.from_euler('xyz',target[3:]).as_matrix()
            desired[side]=transform;self.tasks[side].T_world_frame=transform
        q[15:]=np.clip(q[15:],self.lower,self.upper);self._reset(q)
        def residuals():
            values={}
            for side,target in desired.items():
                pose=self.robot.get_T_world_frame(side+'_paos_tool')
                values[side]=dict(position_error_m=float(np.linalg.norm(target[:3,3]-pose[:3,3])),
                    rotation_error_rad=float(np.linalg.norm(Rotation.from_matrix(target[:3,:3]@pose[:3,:3].T).as_rotvec())))
            return values
        def converged(values):
            return all(v['position_error_m']<=self.position_tolerance and
                       v['rotation_error_rad']<=self.rotation_tolerance for v in values.values())
        steps=0;errors=residuals()
        indices=self.velocity_indices['left']+self.velocity_indices['right']
        while steps<self.iterations and not converged(errors):
            delta=np.asarray(self.solver.solve(False),dtype=float)
            if delta.ndim!=1 or not np.isfinite(delta).all():raise RuntimeError('PlaCo returned invalid joint step')
            arm_delta=delta[indices];scale=min(1.,.15/max(float(np.max(np.abs(arm_delta))),1e-12))
            values=np.array([self.robot.get_joint(n)for n in self.names])+arm_delta*scale
            # QP enforces limits; clamp numeric roundoff before publishing a goal.
            values=np.clip(values,self.lower,self.upper)
            for name,value in zip(self.names,values):self.robot.set_joint(name,float(value))
            self.robot.update_kinematics();steps+=1;errors=residuals()
        result=np.array([self.robot.get_joint(n)for n in self.names])
        if not np.isfinite(result).all():raise RuntimeError('PlaCo returned nonfinite joints')
        for value in errors.values():value['iterations']=steps
        return dict(joints=result,residuals=errors,within_limits=bool(np.all(result>=self.lower)&np.all(result<=self.upper)),
                    converged=converged(errors),backend='placo',qp_solves=steps,compute_s=time.perf_counter()-started)
