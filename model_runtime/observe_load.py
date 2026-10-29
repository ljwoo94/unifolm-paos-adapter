"""Finite read-only three-camera RPC load, independent of model inference."""
import argparse
import json
from pathlib import Path
import time
from paos_remote import RemoteRobotClient

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--target',required=True);p.add_argument('--seconds',type=float,default=60)
p.add_argument('--output',required=True)
a=p.parse_args()
if not 0<a.seconds<=600:p.error('duration in (0,600] required')
latencies=[];count=0;byte_count=0;missing=0;first={};last={}
with RemoteRobotClient.connect(a.target,timeout=5.)as c:
    start=time.monotonic();tick=start
    while time.monotonic()-start<a.seconds:
        begin=time.monotonic();v=c.observe();latencies.append(time.monotonic()-begin);count+=1
        for name,frame in v['frames'].items():
            if frame is None:missing+=1;continue
            byte_count+=len(frame['rgb']);seq=frame['metadata']['sequence']
            first.setdefault(name,seq);last[name]=seq
        tick+=.1
        if tick<time.monotonic():tick=time.monotonic()
        time.sleep(max(0,tick-time.monotonic()))
    elapsed=time.monotonic()-start
latencies.sort()
r=dict(scope='WLA read-only RGB RPC load; no input ownership or model calls',requests=count,seconds=elapsed,
    rgb_bytes=byte_count,missing_frames=missing,first_frame=first,last_frame=last,
    rpc_mean_s=sum(latencies)/len(latencies),rpc_p95_s=latencies[int(.95*(len(latencies)-1))],rpc_max_s=max(latencies))
Path(a.output).write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r))
