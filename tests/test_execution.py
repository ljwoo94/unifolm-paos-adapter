import threading
import time
import numpy as np
import pytest
from websockets.sync.server import serve

from unifolm_paos_adapter.server_client import ModelClient
from unifolm_paos_adapter.execution import ChunkExecution
from unifolm_paos_adapter.chunk import ModelActionChunk
from unifolm_paos_adapter._vendor import msgpack_numpy as codec
from test_adapter import raw_chunk


def test_official_websocket_codec_and_received_timestamp():
    requests=[]
    def handler(ws):
        ws.send(codec.packb({'action_chunk_size':30,'available_unnorm_keys':['UnifoLM_G1_Dex1']}))
        requests.append(codec.unpackb(ws.recv()))
        ws.send(codec.packb(raw_chunk(10)))
    with serve(handler,'127.0.0.1',0,compression=None)as server:
        thread=threading.Thread(target=server.serve_forever);thread.start()
        try:
            port=server.socket.getsockname()[1]
            with ModelClient.connect(f'ws://127.0.0.1:{port}',period_s=1/30,clock=lambda:20.5)as client:
                chunk=client.infer({'test':np.array([1.,2.],np.float32)})
                assert chunk.received_at==20.5
                assert chunk.actions['action.base_command'][0,0,0]==10
                assert requests[0]['type']=='get_action'
                assert requests[0]['obs']['test'].tolist()==[1.,2.]
        finally:server.shutdown();thread.join(timeout=2)


def wait_for(test):
    end=time.monotonic()+3
    while not test():
        assert time.monotonic()<end
        time.sleep(.002)


@pytest.mark.parametrize('fraction',[None,1.0],ids=['default_20_of_30','after_chunk_end'])
def test_default_next_observation_waits_for_chunk_progress(fraction):
    second_observation=threading.Event();release=threading.Event();observations=[];received=[]
    class Model:
        def infer(self,obs):
            if received:release.wait(2)
            now=time.monotonic();received.append(now)
            return ModelActionChunk.from_response(raw_chunk(),received_at=now,period_s=1/30)
        def close(self):release.set()
    def observe():
        observations.append(time.monotonic())
        if len(observations)>1:second_observation.set()
        return observations[-1]
    options={}if fraction is None else {'request_after_fraction':fraction}
    player=ChunkExecution(model_factory=Model,observe=observe,**options)
    player.start()
    try:
        wait_for(lambda:len(received)==1)
        assert not second_observation.wait(.05), 'next observation requested before the first chunk progressed'
        assert second_observation.wait(2)
        assert observations[1]>=received[0]+(20/30 if fraction is None else fraction)
    finally:player.stop();release.set();player.join(1)


def test_stop_interrupts_wait_without_capturing_another_observation():
    calls=[]
    class Model:
        def infer(self,obs):return ModelActionChunk.from_response(raw_chunk(),received_at=time.monotonic(),period_s=1/30)
        def close(self):pass
    player=ChunkExecution(model_factory=Model,observe=lambda:calls.append(time.monotonic()),request_after_fraction=1.)
    player.start()
    try:
        wait_for(lambda:player.snapshot()['responses']==1)
        player.stop();player.join(.2)
        assert not player.alive
        assert len(calls)==1
    finally:player.stop();player.join(1)


@pytest.mark.parametrize('fraction',[-.1,1.1,float('nan'),float('inf'),True])
def test_invalid_observation_fraction_rejected_before_connecting(fraction):
    with pytest.raises(ValueError,match='request_after_fraction'):
        ChunkExecution(model_factory=lambda:None,observe=lambda:None,request_after_fraction=fraction)


def test_single_inflight_independent_playback_pause_resume_and_stop():
    clock=[0.]
    release=threading.Event();calls=[]
    class Model:
        def infer(self,obs):
            calls.append(obs)
            if len(calls)==2:
                release.wait(2)
                raise TimeoutError('inference delayed then failed')
            if len(calls)>2:
                release.wait(2)
                return ModelActionChunk.from_response(raw_chunk(100),received_at=clock[0],period_s=1/30)
            return ModelActionChunk.from_response(raw_chunk(),received_at=clock[0],period_s=1/30)
        def close(self):pass
    sequence=[0]
    def observe():sequence[0]+=1;return sequence[0]
    player=ChunkExecution(model_factory=Model,observe=observe,clock=lambda:clock[0],request_after_fraction=0.)
    player.start()
    try:
        wait_for(lambda:len(calls)==2)
        assert calls==[1,2]
        clock[0]=.5
        assert player.current_goal()==(15.,0.,0.)
        assert len(calls)==2  # Playback does not generate model requests.
        release.set();wait_for(lambda:player.paused)
        assert 'failed' in player.error
        assert player.current_goal()==(15.,0.,0.)
        clock[0]=1.1
        assert player.current_goal()is None
        assert len(calls)==2
        release.clear()
        assert player.resume()
        wait_for(lambda:len(calls)==3)
        assert calls[-1]==3
        # Stopping fences a response that arrives after explicit termination.
        player.stop();release.set();player.join(3)
        assert player.current_goal()is None
        assert not player.alive
    finally:
        player.stop();release.set();player.join(3)


def test_cleanup_failure_does_not_kill_paused_worker():
    entered=threading.Event()
    class Broken:
        def infer(self,obs):entered.set();raise TimeoutError('failed')
        def close(self):raise RuntimeError('close failed')
    player=ChunkExecution(model_factory=Broken,observe=lambda:None)
    player.start()
    try:
        assert entered.wait(2);wait_for(lambda:player.paused)
        time.sleep(.03)
        assert player.alive
        player.stop();player.join(1)
        assert not player.alive
    finally:player.stop();player.join(1)
