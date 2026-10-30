"""Offline narrow check of recording completeness and read-only client calls."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
import numpy as np

spec=importlib.util.spec_from_file_location('shadow',Path(__file__).with_name('shadow_chunks.py'))
shadow=importlib.util.module_from_spec(spec);spec.loader.exec_module(shadow)
calls=[]
class Reader:
    run_id=19
    def status(self):calls.append('status');return {'run_id':19}
    def observe(self,**kwargs):
        calls.append('observe');return {'robot':{'run_id':19,'state':{'q':[0.]*29}},'frames':{}}
    def close(self):calls.append('close')
    def __getattr__(self,name):raise AssertionError('Unexpected robot client operation '+name)
class Model:
    metadata={'rtc':False};last_timing={'model_roundtrip_s':.1}
    def __enter__(self):return self
    def __exit__(self,*exc):pass
    def infer(self,obs):
        return SimpleNamespace(actions={'action.left_ee_rpy':np.zeros((1,30,6),np.float32)},received_at=time.monotonic())
shadow.RemoteRobotClient=SimpleNamespace(connect=lambda *a,**k:Reader())
shadow.ModelClient=SimpleNamespace(connect=lambda *a,**k:Model())
shadow.G1Kinematics=lambda:None
shadow.build_observation=lambda *a,**k:{'instruction':'test','observation.state.left_ee_6d':np.zeros(9,np.float32)}
shadow.load_config=lambda *a,**k:dict(robot_target='unused',model_server_url='unused',observation_timeout_s=1,
    model_timeout_s=1,period_s=1/30,horizon=30,instruction='test',gripper_mapping={},image_encoding='rgb8',jpeg_quality=80)
with tempfile.TemporaryDirectory()as tmp:
    out=Path(tmp)/'capture';sys.argv=['shadow','--config','unused','--output',str(out),'--chunks','2']
    shadow.main()
    assert calls==['status','observe','observe','close'],calls
    assert len(list(out.glob('*-input.npz')))==2
    assert len(list(out.glob('*-chunk.npz')))==2
    assert len(json.loads((out/'trace.json').read_text())['rows'])==2
    assert np.load(out/'000-input.npz')['seed'].item()==10000
print('PASS: two inputs/chunks/timing rows saved; robot calls status/observe/close only')
