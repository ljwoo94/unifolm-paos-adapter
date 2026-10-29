"""Official UniFoLM input/output conversion plus the existing native W8A8 runtime.

No RTC, IK, robot DDS, trajectory smoothing or controller is installed here.
"""
import argparse
import asyncio
from concurrent.futures import ThreadPoolExecutor
import json
import logging
import sys

import numpy as np
import cv2
import websockets.asyncio.server

from runtime import ModelRuntime,BUNDLE
sys.path.insert(0,str(BUNDLE/'src'))
from model_server.action_server_wbc_msgpack_unitree import ActionServerWBCMsgpack
from model_server.unifolm_wla_action_adapter import build_unitree_fullbody_action_mask


class NativeServer(ActionServerWBCMsgpack):
    def __init__(self,args):
        self.args=args
        self.instruction=args.instruction
        self.runtime=ModelRuntime()
        self.model=self.runtime.model
        self._action_chunk_size=self.model.action_horizon
        self._norm_stats_all=self.model.norm_stats
        self._norm_arrays={key:{field:np.array(stats[section][name],np.float32)
            for field,section,name in [('action_offset','action','offset'),('action_scale','action','scale'),
                ('state_offset','state','offset'),('state_scale','state','scale')]}
            for key,stats in self._norm_stats_all.items()}
        self._default_unnorm_key='UnifoLM_G1_Dex1'
        self._action_mask=build_unitree_fullbody_action_mask()
        self._image_size=(320,448)
        cv2.setNumThreads(0)
        self._img_pool=ThreadPoolExecutor(max_workers=4,thread_name_prefix='image')

    @property
    def metadata(self):
        result=super().metadata
        return dict(result,rtc=False,inference_backend='native_w8a8_paos',compile=self.runtime.compile_config)

    def get_action(self,obs):
        if 'rtc'in obs:raise ValueError('RTC is excluded')
        prep=self._build_example(obs)
        predicted,elapsed=self.runtime.predict(prep['example'],seed=obs.get('seed'))
        result=self._encode_action(predicted[0],prep['unnorm_key'],prep['state_unnorm'])
        logging.info('native model seconds=%.6f',elapsed)
        return result

    async def _run(self,host,port):
        async with websockets.asyncio.server.serve(self._handler,host,port,compression=None,
                max_size=4*1024*1024,ping_interval=None,ping_timeout=None)as server:
            await server.serve_forever()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host',default='127.0.0.1');p.add_argument('--port',type=int,default=18600)
    p.add_argument('--instruction',default='Move forward slowly.')
    p.add_argument('--warmups',type=int,default=2)
    a=p.parse_args();a.ckpt_path=str(BUNDLE/'model.safetensors')
    logging.basicConfig(level=logging.INFO)
    server=NativeServer(a)
    for i in range(a.warmups):
        output,elapsed=server.runtime.predict(server.runtime.examples[i%len(server.runtime.examples)],10000+i%len(server.runtime.examples))
        delta=output-server.runtime.reference[i%len(server.runtime.examples):i%len(server.runtime.examples)+1]
        rmse=float(np.sqrt(np.mean(delta**2)))
        if not np.isfinite(output).all()or rmse>=.01:raise RuntimeError('startup numerical reference check failed')
        print('MODEL_WARMUP',i,elapsed,rmse,flush=True)
    print('MODEL_READY',json.dumps(server.metadata,default=str),flush=True)
    try:server.run(a.host,a.port)
    finally:server._img_pool.shutdown()


if __name__=='__main__':main()
