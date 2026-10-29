"""Finite read-only three-camera RPC load, independent of model inference."""
import argparse
import json
from pathlib import Path
import time
from paos_remote import RemoteRobotClient

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--target',required=True);p.add_argument('--seconds',type=float,default=60)
p.add_argument('--output',required=True);p.add_argument('--encoding',choices=['rgb8','jpeg'],default='rgb8');p.add_argument('--hz',type=float,default=10)
a=p.parse_args()
if not 0<a.hz<=100:p.error('query rate in (0,100] required')
if not 0<a.seconds<=600:p.error('duration in (0,600] required')
latencies=[];count=0;byte_count=0;missing=0;first={};last={};rows=[];unique={}
with RemoteRobotClient.connect(a.target,timeout=5.)as c:
    start=time.monotonic();tick=start
    while time.monotonic()-start<a.seconds:
        begin=time.monotonic();v=c.observe(diagnostics=True,image_encoding=a.encoding);rows.append(dict(received_at=time.monotonic(),server=v['diagnostics'],client=v['client_timing'],frames={name:frame['metadata']['sequence']if frame else None for name,frame in v['frames'].items()}));latencies.append(time.monotonic()-begin);count+=1
        for name,frame in v['frames'].items():
            if frame is None:missing+=1;continue
            byte_count+=len(frame.get('rgb',frame.get('data',b'')));seq=frame['metadata']['sequence'];unique.setdefault(name,set()).add(seq)
            first.setdefault(name,seq);last[name]=seq
        tick+=1/a.hz
        if tick<time.monotonic():tick=time.monotonic()
        time.sleep(max(0,tick-time.monotonic()))
    elapsed=time.monotonic()-start
latencies.sort()
r=dict(scope='WLA read-only RGB RPC load; no input ownership or model calls',requests=count,seconds=elapsed,
    image_encoding=a.encoding,requested_hz=a.hz,payload_bytes=byte_count,unique_received_frames={n:len(v)for n,v in unique.items()},unique_received_fps={n:len(v)/elapsed for n,v in unique.items()},missing_frames=missing,trace=rows,first_frame=first,last_frame=last,
    rpc_mean_s=sum(latencies)/len(latencies),rpc_p95_s=latencies[int(.95*(len(latencies)-1))],rpc_max_s=max(latencies))
Path(a.output).write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r))
