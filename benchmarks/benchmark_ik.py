"""Offline deterministic EE-IK fixture replay, no model or robot execution."""
import argparse,cProfile,io,json,pstats,time,platform
from pathlib import Path
import numpy as np
from unifolm_paos_adapter.inverse_kinematics import ArmIK
p=argparse.ArgumentParser();p.add_argument('--fixtures',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--repeat',type=int,default=3);a=p.parse_args()
f=np.load(a.fixtures,allow_pickle=False);solver=ArmIK();measure=[];solutions=[];flags=[]
def solve(i):return solver.solve(f['seeds'][i],dict(zip(['left','right'],f['targets'][i])))
for i in range(min(3,len(f['seeds']))):solve(i)
for k in range(a.repeat):
 for i in range(len(f['seeds'])):
  t=time.perf_counter();r=solve(i);measure.append((time.perf_counter()-t)*1000)
  if k==0:solutions.append(r['joints']);flags.append(r['converged'])
pr=cProfile.Profile();pr.enable()
for i in range(len(f['seeds'])):solve(i)
pr.disable();stream=io.StringIO();pstats.Stats(pr,stream=stream).sort_stats('cumtime').print_stats(25)
a.output.with_suffix('.profile.txt').write_text(stream.getvalue());np.savez(a.output.with_suffix('.solutions.npz'),joints=solutions,converged=flags)
v=np.array(measure);r=dict(scope='Offline identical fixture replay on this host, excludes network/model/robot',backend='placo',adapter_version='0.3.0',python=platform.python_version(),host=platform.node(),platform=platform.platform(),cases=len(f['seeds']),repeat=a.repeat,samples=len(v),converged=sum(flags),mean_ms=float(v.mean()),p50_ms=float(np.percentile(v,50)),p95_ms=float(np.percentile(v,95)),max_ms=float(v.max()),times_ms=measure)
a.output.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({k:v for k,v in r.items()if k!='times_ms'}))
