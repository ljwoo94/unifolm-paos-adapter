"""Official G1 URDF FK: measured body29, pelvis base, wrist-local tool point."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from yourdfpy import URDF

ASSETS=Path(__file__).parent/'assets'


class G1Kinematics:
    def __init__(self,urdf=None,joint_names=None,*,tool_offset_m=.105):
        self.path=Path(urdf)if urdf is not None else ASSETS/'g1_29dof_mode_15_with_dex1_1.urdf'
        self.names=list(joint_names)if joint_names is not None else json.loads((ASSETS/'body29.json').read_text())
        expected={j.attrib['name']for j in ET.parse(self.path).getroot().findall('joint')if j.attrib['type']=='revolute'}
        if len(self.names)!=29 or len(set(self.names))!=29 or set(self.names)!=expected:
            raise ValueError('body29 joint names must match the complete original URDF revolute set')
        if isinstance(tool_offset_m,bool)or not np.isfinite(tool_offset_m):raise ValueError('finite tool offset required')
        self.offset=np.array([float(tool_offset_m),0.,0.,1.])
        self.model=URDF.load(str(self.path),build_scene_graph=True,load_meshes=False,
                            build_collision_scene_graph=False,load_collision_meshes=False)

    def ee9(self,q):
        q=np.asarray(q)
        if q.shape!=(29,)or q.dtype.kind not in 'fiu' or not np.isfinite(q).all():
            raise ValueError('finite measured body29 required')
        self.model.update_cfg(dict(zip(self.names,map(float,q))))
        values={}
        for side in ['left','right']:
            transform=self.model.get_transform(frame_to=f'{side}_wrist_yaw_link',frame_from='pelvis')
            position=(transform@self.offset)[:3]
            values[side]=np.concatenate([position,transform[:3,:2].T.reshape(-1)]).astype(np.float32)
        return values
