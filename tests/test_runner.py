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
    p.write_text(p.read_text()+'image_size: [448, 320]\n')
    c=load_config(p)
    assert c['image_size']==[448,320]
    original=p.read_text()
    for size in ('[0, 1]', '[641, 1]', '[true, 2]', '[1.5, 2]', '[1]'):
        p.write_text(original.replace('[448, 320]',size))
        with pytest.raises(ValueError,match='image_size'):load_config(p)
    p.write_text(original)
    assert c['robot_target']=='192.168.1.10:15051'
    assert c['model_server_url']=='ws://127.0.0.1:18600'
    assert c['horizon']==30
    assert c['request_after_fraction']==2/3
    assert load_config(p,request_after_fraction=1.)['request_after_fraction']==1.
    for fraction in [-.1,1.1,float('nan'),float('inf'),True]:
        with pytest.raises(ValueError,match='request_after_fraction'):
            load_config(p,request_after_fraction=fraction)
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
        horizon=30,model_timeout_s=1,rpc_timeout_s=1,gripper_mapping={},image_encoding='jpeg',jpeg_quality=70,diagnostics=False,image_size=[448,320])
    runner.run(c,seconds=1,output=tmp_path/'run',probe=True)
    assert calls and calls[0]['image_encoding']=='jpeg' and calls[0]['jpeg_quality']==70
    assert calls[0]['image_size']==[448,320]


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
        horizon=30,model_timeout_s=1,rpc_timeout_s=1,gripper_mapping={'kind':'dex1_native_rad','offsets':{'left':0.,'right':0.}},diagnostics=True,
        request_after_fraction=0.)  # This test isolates stop during an in-flight observation.
    result=runner.run(config,seconds=1,output=tmp_path/'run',probe=True)
    assert result['model_worker_stopped'] and result['observation_snapshot_complete']
    assert len(result['observations'])==2 and result['responses']==1


@pytest.mark.parametrize('seconds',[None,1.0])
@pytest.mark.parametrize('close_fails',[False,True])
def test_operator_stop_releases_control_before_waiting_for_model(tmp_path,monkeypatch,seconds,close_fails,capsys):
    import threading
    from types import SimpleNamespace
    import unifolm_paos_adapter.runner as runner
    stop=threading.Event();events=[];clients=[]
    class Robot:
        run_id=73
        def __init__(self):self.index=len(clients);clients.append(self)
        def status(self):return {'enabled':True}
        def acquire_control(self):pass
        def set_velocity(self,*goal):return SimpleNamespace(run_id=73,goal_ref=1,accepted_ns=1,expires_ns=2)
        def close(self):
            events.append('owner-close'if self.index else'observer-close')
            if self.index and close_fails:raise TimeoutError('cancellation not confirmed')
        def abandon(self):events.append('abandon')
    class Player:
        paused=False;alive=False
        def __init__(self,**kwargs):self.count=0
        def start(self):pass
        def current_goal(self):
            self.count+=1
            if self.count==3:stop.set()
            return (.1,0.,0.)
        def latest_chunk(self):return SimpleNamespace(actions={'example':runner.np.zeros(1)})
        def snapshot(self):return {'responses':1}
        def stop(self):events.append('model-stop')
        def join(self,timeout):events.append('model-join')
    monkeypatch.setattr(runner.RemoteRobotClient,'connect',lambda *a,**k:Robot())
    monkeypatch.setattr(runner,'G1Kinematics',lambda:None)
    monkeypatch.setattr(runner,'ChunkExecution',Player)
    def dump(*args,**kwargs):
        assert 'owner-close'in events, 'control must stop before file export'
    monkeypatch.setattr(runner.np,'savez',dump)
    config=dict(robot_target='robot:1',model_server_url='ws://model:2',instruction='walk',
        period_s=1/30,horizon=30,model_timeout_s=1,rpc_timeout_s=1,command_hz=100,gripper_mapping={})
    result=runner.run(config,seconds=seconds,output=tmp_path/'run',stop=stop)
    assert result['goal_count']==3 and result['normal_stop']
    assert result['stop_reason']=='operator_stop'
    assert events.index('owner-close')<events.index('model-stop')<events.index('model-join')
    messages = capsys.readouterr().out
    if close_fails:
        assert result['input_control'] == 'unknown' and result['close_error']
        assert 'input_control_release_unknown' in messages and 'input_control_released' not in messages
    else:
        assert result['input_control'] == 'released'
        assert 'input_control_released' in messages


def test_cli_close_failure_does_not_report_success(monkeypatch, tmp_path):
    import sys
    import unifolm_paos_adapter.runner as runner
    monkeypatch.setattr(sys, 'argv', ['unifolm-paos','--config','unused.yaml','--output',str(tmp_path/'run')])
    monkeypatch.setattr(runner.signal, 'signal', lambda *args: None)
    monkeypatch.setattr(runner, 'load_config', lambda *args, **kwargs: {})
    monkeypatch.setattr(runner, 'run', lambda *args, **kwargs: dict(
        responses=1, goal_count=1, close_error='cancellation not confirmed', input_control='unknown'))
    with pytest.raises(SystemExit) as caught:
        runner.main()
    assert caught.value.code == 1


def test_prompt_override_preserves_connection_configuration(tmp_path):
    from pathlib import Path
    path=tmp_path/'config.yaml'
    path.write_text((Path(__file__).resolve().parents[1]/'configs/g1-real-base.yaml').read_text())
    config=load_config(path,instruction='Pick up the banana.')
    assert config['instruction']=='Pick up the banana.'
    assert config['robot_target']=='192.168.123.164:15051'
    assert 'Move forward slowly.'in path.read_text()
