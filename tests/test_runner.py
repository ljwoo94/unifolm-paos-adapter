import pytest
from unifolm_paos_adapter.runner import load_config


def test_independent_endpoints_and_explicit_input_profile(tmp_path):
    p=tmp_path/'adapter.yaml'
    p.write_text('''robot_target: 192.168.1.10:15051
model_server_url: ws://127.0.0.1:18600
instruction: Move forward slowly.
period_s: 0.03333333333333333
gripper_mapping:
  kind: dex1_native_rad
  offsets: {left: 0, right: 0}
''')
    c=load_config(p)
    assert c['robot_target']=='192.168.1.10:15051'
    assert c['model_server_url']=='ws://127.0.0.1:18600'
    assert c['horizon']==30
    p.write_text('robot_target: 192.168.1.10\n')
    with pytest.raises(ValueError):load_config(p)


def test_robot_send_failure_abandons_transport_without_cancelling_goals(tmp_path,monkeypatch):
    import threading
    import unifolm_paos_adapter.runner as runner
    from test_adapter import raw_chunk
    from unifolm_paos_adapter.chunk import ModelActionChunk
    import time
    clients=[]
    class Robot:
        run_id=73
        def __init__(self):self.closed=False;self.abandoned=False;clients.append(self)
        def status(self):return {'enabled':True}
        def observe(self):return {}
        def acquire_control(self):pass
        def set_velocity(self,*goal):raise ConnectionError('send failed before dispatch')
        def close(self):self.closed=True
        def abandon(self):self.abandoned=True
    class Model:
        def __init__(self):self.calls=0;self.done=threading.Event()
        def infer(self,obs):
            self.calls+=1
            if self.calls>1:self.done.wait(1)
            return ModelActionChunk.from_response(raw_chunk(),received_at=time.monotonic(),period_s=1/30)
        def close(self):self.done.set()
    monkeypatch.setattr(runner.RemoteRobotClient,'connect',lambda *a,**k:Robot())
    monkeypatch.setattr(runner.ModelClient,'connect',lambda *a,**k:Model())
    monkeypatch.setattr(runner,'G1Kinematics',lambda:None)
    monkeypatch.setattr(runner,'build_observation',lambda *a,**k:{})
    config=dict(robot_target='robot:1',model_server_url='ws://model:2',instruction='walk',
        period_s=1/30,horizon=30,model_timeout_s=1,rpc_timeout_s=1,gripper_mapping={})
    with pytest.raises(ConnectionError):runner.run(config,seconds=1,output=tmp_path/'run')
    assert clients[0].closed
    assert clients[1].abandoned and not clients[1].closed
    import json
    result=json.loads((tmp_path/'run/result.json').read_text())
    assert not result['normal_stop'] and result['model_worker_stopped']
    assert result['goals'][0]['stages']['base']['outcome']=='unknown'
    assert result['error']=='ConnectionError: send failed before dispatch'


def test_resume_after_control_stream_end_is_rejected_before_inference(tmp_path,monkeypatch):
    import threading
    import unifolm_paos_adapter.runner as runner
    resumes=[];event=threading.Event()
    class Robot:
        run_id=73;control_active=False
        def status(self):return {'enabled':True}
        def acquire_control(self):pass
        def close(self):pass
        def abandon(self):pass
    class Player:
        paused=True;error='model failed';alive=False
        def __init__(self,**kwargs):pass
        def start(self):event.set()
        def resume(self):resumes.append(True)
        def stop(self):pass
        def join(self,timeout):pass
        def snapshot(self):return {}
    monkeypatch.setattr(runner.RemoteRobotClient,'connect',lambda *a,**k:Robot())
    monkeypatch.setattr(runner,'G1Kinematics',lambda:None)
    monkeypatch.setattr(runner,'ChunkExecution',Player)
    config=dict(robot_target='robot:1',model_server_url='ws://model:2',instruction='walk',
        period_s=1/30,horizon=30,model_timeout_s=1,rpc_timeout_s=1,gripper_mapping={})
    with pytest.raises(RuntimeError,match='control session'):
        runner.run(config,seconds=1,output=tmp_path/'run',resume=event)
    assert not resumes


def test_explicit_jpeg_is_requested_without_diagnostics(tmp_path,monkeypatch):
    import threading,time
    import unifolm_paos_adapter.runner as runner
    from unifolm_paos_adapter.chunk import ModelActionChunk
    from test_adapter import raw_chunk
    calls=[]
    class Robot:
        run_id=73
        def status(self):return {'enabled':True}
        def observe(self,**options):calls.append(options);return {}
        def close(self):pass
    class Model:
        def __init__(self):self.count=0;self.done=threading.Event()
        def infer(self,obs):
            self.count+=1
            if self.count>1:self.done.wait(1)
            return ModelActionChunk.from_response(raw_chunk(),received_at=time.monotonic(),period_s=1/30)
        def close(self):self.done.set()
    monkeypatch.setattr(runner.RemoteRobotClient,'connect',lambda *a,**k:Robot())
    monkeypatch.setattr(runner.ModelClient,'connect',lambda *a,**k:Model())
    monkeypatch.setattr(runner,'G1Kinematics',lambda:None)
    monkeypatch.setattr(runner,'build_observation',lambda *a,**k:{})
    c=dict(robot_target='robot:1',model_server_url='ws://model:2',instruction='walk',period_s=1/30,
        horizon=30,model_timeout_s=1,rpc_timeout_s=1,gripper_mapping={},image_encoding='jpeg',jpeg_quality=70,diagnostics=False)
    runner.run(c,seconds=1,output=tmp_path/'run',probe=True)
    assert calls and calls[0]['image_encoding']=='jpeg' and calls[0]['jpeg_quality']==70


def test_final_trace_includes_observation_completed_during_stop(tmp_path,monkeypatch):
    import threading,time
    import unifolm_paos_adapter.runner as runner
    from unifolm_paos_adapter.execution import ChunkExecution
    from unifolm_paos_adapter.chunk import ModelActionChunk
    from test_adapter import snapshot,raw_chunk
    second=threading.Event();release=threading.Event()
    class Robot:
        run_id=73
        count=0
        def status(self):return {'enabled':True}
        def observe(self,**kwargs):
            self.count+=1
            if self.count==2:second.set();release.wait(2)
            value=snapshot();value['robot']['state']['state_seq']=self.count
            for frame in value['frames'].values():
                frame['metadata'].update(sequence=self.count,source_sequence=self.count,source_epoch='test',
                    rgb_time_ns=1000,received_ns=1000,published_ns=1000)
            return value
        def close(self):pass
    class Model:
        def infer(self,obs):return ModelActionChunk.from_response(raw_chunk(),received_at=time.monotonic(),period_s=1/30)
        def close(self):release.set()
    class ControlledExecution(ChunkExecution):
        def current_goal(self):
            assert second.wait(2)
            return super().current_goal()
    monkeypatch.setattr(runner.RemoteRobotClient,'connect',lambda *a,**k:Robot())
    monkeypatch.setattr(runner.ModelClient,'connect',lambda *a,**k:Model())
    monkeypatch.setattr(runner,'ChunkExecution',ControlledExecution)
    config=dict(robot_target='robot:1',model_server_url='ws://model:2',instruction='walk',period_s=1/30,
        horizon=30,model_timeout_s=1,rpc_timeout_s=1,gripper_mapping={'kind':'dex1_native_rad','offsets':{'left':0.,'right':0.}},diagnostics=True)
    result=runner.run(config,seconds=1,output=tmp_path/'run',probe=True)
    assert result['model_worker_stopped'] and result['observation_snapshot_complete']
    assert len(result['observations'])==2 and result['responses']==1
