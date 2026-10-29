"""Single explicit WebSocket connection to the official NumPy-aware protocol."""
import math
import time
from websockets.sync.client import connect
from ._vendor import msgpack_numpy as codec
from .chunk import ModelActionChunk


class ModelClient:
    def __init__(self,url,*,period_s,timeout=5.,horizon=30,clock=time.monotonic):
        if not math.isfinite(timeout)or timeout<=0:raise ValueError('positive model request timeout required')
        self.url=url;self.period_s=period_s;self.timeout=timeout;self.horizon=horizon;self.clock=clock
        self.socket=None;self.metadata=None

    @classmethod
    def connect(cls,url,**kwargs):
        obj=cls(url,**kwargs)
        try:
            obj.socket=connect(url,open_timeout=obj.timeout,close_timeout=1,compression=None,max_size=4*1024*1024,ping_interval=None)
            obj.metadata=codec.unpackb(obj.socket.recv(timeout=obj.timeout))
            if obj.metadata.get('action_chunk_size')!=obj.horizon:raise ValueError('model horizon/config mismatch')
            if 'UnifoLM_G1_Dex1' not in obj.metadata.get('available_unnorm_keys',[]):raise ValueError('required dataset normalization absent')
            rtc=obj.metadata.get('rtc')
            if rtc not in (None,False)and not(isinstance(rtc,dict)and rtc.get('method')=='off'):
                raise ValueError('RTC must be disabled for PAOS baseline')
            return obj
        except BaseException:obj.close();raise

    def infer(self,observation):
        if self.socket is None:raise RuntimeError('explicit model connection required')
        self.socket.send(codec.packb({'type':'get_action','obs':observation}))
        message=self.socket.recv(timeout=self.timeout)
        received=self.clock()
        if isinstance(message,str):raise RuntimeError('model server failed: '+message[:512])
        return ModelActionChunk.from_response(codec.unpackb(message),received_at=received,
            period_s=self.period_s,expected_horizon=self.horizon)

    def close(self):
        socket,self.socket=self.socket,None
        if socket is not None:socket.close()

    def __enter__(self):return self
    def __exit__(self,*exc):self.close()
