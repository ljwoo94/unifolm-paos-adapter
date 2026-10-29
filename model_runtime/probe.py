"""Finite stored-input protocol probe; no PAOS/robot connection."""
import json
import os
from pathlib import Path
import pickle
import sys
import numpy as np
from unifolm_paos_adapter.server_client import ModelClient

bundle=Path(os.environ['WMA_MODEL_BUNDLE'])
sys.path.insert(0,str(bundle/'src'))
from unifolm_wla.dataloader.multi_source_dataset.action_mapping import STATE_SLICES
example=pickle.loads((bundle/'offline-observations.pkl').read_bytes())[0]
stats=json.loads((bundle/'dataset_statistics.json').read_text()) if (bundle/'dataset_statistics.json').exists()else None
if stats is None:
    candidates=list(bundle.glob('*stat*.json'))
    raise RuntimeError(f'Provide exact bundled normalization file: {candidates}')
stats=stats['UnifoLM_G1_Dex1']['state']
state=example['state']*(np.asarray(stats['scale'],np.float32)+1e-8)+np.asarray(stats['offset'],np.float32)
obs={'instruction':'Move forward slowly.','unnorm_key':'UnifoLM_G1_Dex1','seed':10000}
for target,source in [('left_ee_6d','left_xyz_rot6d'),('right_ee_6d','right_xyz_rot6d'),('left_gripper','left_gripper'),('right_gripper','right_gripper')]:
    obs['observation.state.'+target]=state[STATE_SLICES[source]]
obs['observation.state.lower_body']=np.concatenate([state[STATE_SLICES[k]]for k in ['left_leg_joint','right_leg_joint','waist_joint']])
for role,img in zip(['cam_left_high','cam_left_wrist','cam_right_wrist'],example['image']):
    obs['observation.images.'+role]=np.asarray(img)[:,:,::-1].copy()
with ModelClient.connect('ws://127.0.0.1:18600',period_s=1/30,timeout=30)as client:
    chunk=client.infer(obs)
    result=dict(scope='WLA stored observation -> real W8A8 official WebSocket; no robot commands',
        metadata=client.metadata,shapes={k:list(v.shape)for k,v in chunk.actions.items()},
        base_first=chunk.actions['action.base_command'][0,0].tolist())
    Path('runs/stored-protocol-probe.json').write_text(json.dumps(result,indent=2,default=str)+'\n')
    np.savez('runs/stored-protocol-actions.npz',**dict(chunk.actions))
    print(json.dumps({k:result[k]for k in ['scope','shapes','base_first']}))
