"""Decode and plot generated chunks on an explicitly synthetic execution clock."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

BUNDLE=Path('/home/wla/WMA-JLee/models/C-W8A8-20261028')
sys.path.insert(0,str(BUNDLE/'src'))
from model_server.action_server_wbc_msgpack_unitree import ActionServerWBCMsgpack

parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory');args=parser.parse_args()
out=Path(args.directory);arrays=np.load(out/'normalized-chunks.npz')
stats=json.loads((BUNDLE/'dataset_statistics.json').read_text())['UnifoLM_G1_Dex1']
anchor=np.load(out/'state-anchor.npy')*(np.asarray(stats['state']['scale'],np.float32)+1e-8)+np.asarray(stats['state']['offset'],np.float32)
decoder=object.__new__(ActionServerWBCMsgpack)
decoder._norm_arrays={'UnifoLM_G1_Dex1':{'action_offset':np.asarray(stats['action']['offset'],np.float32),'action_scale':np.asarray(stats['action']['scale'],np.float32)}}
report=json.loads((out/'report.json').read_text())
count=arrays[arrays.files[0]].shape[0];starts=np.arange(count)*15
adoptions=np.concatenate([[0],starts[1:]+10]);clock=np.arange(starts[-1]+30)
decoded={name:[decoder._encode_action(chunk,'UnifoLM_G1_Dex1',anchor)for chunk in arrays[name]]for name in arrays.files}
results={};traces={}
methods=[('w8a8_off_receipt_index0','w8a8_off',False)]+[(name,name,True)for name in arrays.files]
colors=['#d54c36','#555555','#1875b5','#969696','#229866','#9b5ab4']
fig,axes=plt.subplots(2,3,figsize=(13,7),sharex=True)
for order,(label,source,align)in enumerate(methods):
    values=decoded[source];side_results={}
    for row,side in enumerate(['left','right']):
        key='action.'+side+'_ee_rpy'
        pos=np.stack([x[key][0,:,:3]for x in values])
        trace=[];boundary=[];plan_change=[]
        for t in clock:
            chunk=int(np.searchsorted(adoptions,t,side='right')-1)
            cursor=int(t-(starts[chunk]if align else adoptions[chunk]))
            trace.append(pos[chunk,min(cursor,29)])
            if chunk and t==adoptions[chunk]:
                last_cursor=int(t-1-(starts[chunk-1]if align else adoptions[chunk-1]))
                boundary.append(float(np.linalg.norm(pos[chunk,cursor]-pos[chunk-1,min(last_cursor,29)])*100))
                old_same_t=int(t-(starts[chunk-1]if align else adoptions[chunk-1]))
                plan_change.append(float(np.linalg.norm(pos[chunk,cursor]-pos[chunk-1,min(old_same_t,29)])*100))
        trace=np.asarray(trace);traces[label+'_'+side]=trace
        internal=np.linalg.norm(pos[:,1:-1]-(pos[:,:-2]+pos[:,2:])/2,axis=-1)*100
        side_results[side]=dict(boundary_step_cm=boundary,max_boundary_step_cm=max(boundary),
            change_from_old_plan_at_same_time_cm=plan_change,
            max_internal_midpoint_excursion_cm=float(np.max(internal)),
            max_native_step_cm=float(np.max(np.linalg.norm(np.diff(trace,axis=0),axis=-1))*100),
            path_length_cm=float(np.linalg.norm(np.diff(trace,axis=0),axis=-1).sum()*100))
        for col,axis in enumerate('XYZ'):
            ax=axes[row,col];ax.plot(clock/30,trace[:,col]*100,label=label,color=colors[order%len(colors)],lw=1.4)
    results[label]=side_results
for row,side in enumerate(['Left','Right']):
    for col,axis in enumerate('XYZ'):
        ax=axes[row,col]
        for t in adoptions[1:]/30:ax.axvline(t,color='#999999',lw=.7,ls=':')
        ax.set_title(side+' EE '+axis);ax.set_ylabel('Decoded target (cm)');ax.grid(alpha=.2)
        if row==1:ax.set_xlabel('Synthetic native execution time (s)')
axes[0,0].legend(fontsize=6)
fig.suptitle('Actual generated chunks, FIXED recorded observation; no robot or task ground truth\nSynthetic delay 0.333s / stride 0.5s; measured RTC compute is slower than assumed delay',fontsize=12)
fig.tight_layout(rect=(0,0,1,.91));fig.savefig(out/'rtc-trajectories.png',dpi=160)
np.savez(out/'decoded-replay.npz',time_s=clock/30,**traces)
summary=dict(scope='generated chunks on synthetic clock, not physically executed; no IK/OS dynamics',
    seconds_per_native_step=1/30,adoption_time_s=(adoptions/30).tolist(),methods=results,
    caveats=['fixed observation and identical EE anchor for every chunk; not a closed-loop task',
             'assumed d=10 is smaller than measured RTC head compute; not an achievable online schedule',
             'method comparisons use matching seeds per chunk; raw dequantized-head output is not original BF16',
             'boundary step includes normal motion; same-time plan change isolates replacement difference',
             'all values are model EE targets before IK; no actual robot state or safety limits'])
(out/'trajectory-metrics.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2))
