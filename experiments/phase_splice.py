"""Position-only offline nearest-phase + C2 bridge ablation; no model/robot calls.

Fixed-observation generated chunks; synthetic arrivals. Thresholds below are
experimental diagnostics, not robot operating limits. No IK/orientation/contact.
"""
import argparse,json,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

P=Path('/home/wla/WMA-JLee/models/C-W8A8-20261028');sys.path.insert(0,str(P/'src'))
from model_server.action_server_wbc_msgpack_unitree import ActionServerWBCMsgpack

def bridge(p0,v0,a0,p1,v1,a1,n=3,dt=1/30):
    T=n*dt
    coefficients=np.zeros((6,len(p0)));coefficients[:3]=[p0,v0*T,.5*a0*T*T]
    rhs=np.stack([p1-coefficients[:3].sum(0),v1*T-coefficients[1]-2*coefficients[2],a1*T*T-2*coefficients[2]])
    coefficients[3:]=np.linalg.solve(np.array([[1,1,1],[3,4,5],[6,12,20.]]),rhs)
    u=np.arange(1,n+1)/n
    return np.stack([sum(coefficients[i]*x**i for i in range(6))for x in u])

def replay(chunks,mode):
    dt=1/30;arrivals=np.array([0,25,40,55]);n=75
    q=[];buffer=list(chunks[0]);cursor=0;hold=0;switches=[]
    for t in range(n):
        if t in arrivals[1:]:
            j=int(np.flatnonzero(arrivals==t)[0]);new=chunks[j]
            current=q[-1];v=(q[-1]-q[-2])/dt if len(q)>1 else np.zeros(6)
            a=(q[-1]-2*q[-2]+q[-3])/dt**2 if len(q)>2 else np.zeros(6)
            if mode=='index0':k=0;accepted=True
            else:
                nv=np.gradient(new,dt,axis=0)
                score=((new-current)**2).sum(1)+.05**2*((nv-v)**2).sum(1)
                limit=15 if mode=='nearest_c2_tail15'else 30
                k=int(np.argmin(score[:limit]));distance=np.linalg.norm((new[k]-current).reshape(2,3),axis=1)
                accepted=mode!='nearest_c2_tail15'or bool(np.max(distance)<=.02)
            switches.append(dict(t_s=t*dt,k=k,accepted=accepted,tail_actions=30-k,
                tail_s=(30-k)*dt,max_hand_distance_cm=float(np.max(np.linalg.norm((new[k]-current).reshape(2,3),axis=1))*100)))
            if accepted:
                buffer=list(new[k:]);cursor=0
                if mode.startswith('nearest_c2'):
                    steps=min(3,len(buffer)-1)
                    if steps:
                        end=k+steps;nv=np.gradient(new,dt,axis=0);na=np.gradient(nv,dt,axis=0)
                        buffer=list(bridge(current,v,a,new[end],nv[end],na[end],steps))+list(new[end+1:])
        if cursor<len(buffer):value=buffer[cursor];cursor+=1
        else:value=q[-1];hold+=1
        q.append(np.asarray(value).copy())
    q=np.asarray(q);d=np.linalg.norm(np.diff(q,axis=0).reshape(-1,2,3),axis=2)*100
    return q,dict(switches=switches,missing_future_action_frames=hold,missing_future_action_fraction=hold/n,
        max_native_position_step_cm=float(d.max()),max_speed_m_s=float(d.max()/100/dt),
        net_displacement_cm=(np.linalg.norm((q[-1]-q[0]).reshape(2,3),axis=1)*100).tolist())

parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory');args=parser.parse_args()
out=Path(args.directory);arrays=np.load(out/'normalized-chunks.npz')
stats=json.loads((P/'dataset_statistics.json').read_text())['UnifoLM_G1_Dex1']
state=np.load(out/'state-anchor.npy')*(np.asarray(stats['state']['scale'],np.float32)+1e-8)+np.asarray(stats['state']['offset'],np.float32)
decoder=object.__new__(ActionServerWBCMsgpack);decoder._norm_arrays={'UnifoLM_G1_Dex1':{'action_offset':np.asarray(stats['action']['offset'],np.float32),'action_scale':np.asarray(stats['action']['scale'],np.float32)}}
poses=[]
for a in arrays['w8a8_off']:
    v=decoder._encode_action(a,'UnifoLM_G1_Dex1',state)
    poses.append(np.concatenate([v['action.'+side+'_ee_rpy'][0,:,:3]for side in ['left','right']],axis=1))
poses=np.asarray(poses);metrics={};traces={}
fig,axes=plt.subplots(2,1,figsize=(11,7),sharex=True)
for method in ['index0','nearest','nearest_c2','nearest_c2_tail15']:
    trace,metrics[method]=replay(poses,method);traces[method]=trace
    for ax,i,label in zip(axes,[1,5],['Left EE Y','Right EE Z']):
        ax.plot(np.arange(75)/30,trace[:,i]*100,label=method);ax.set_ylabel(label+' (cm)');ax.grid(alpha=.2)
for ax in axes:
    for t in [25/30,40/30,55/30]:ax.axvline(t,color='#888888',ls=':',lw=.8)
axes[0].legend(fontsize=9);axes[-1].set_xlabel('Synthetic execution time (s)')
fig.suptitle('Nearest-phase matching on actual generated chunks, fixed observation\nPosition-only offline ablation; held tail is recorded, not task success',fontsize=12)
fig.tight_layout(rect=(0,0,1,.92));fig.savefig(out/'phase-splice.png',dpi=160)
np.savez(out/'phase-splice.npz',**traces)
# Inspect a known single-frame spike: interpolate its time slot from neighbours.
native=np.load(P/'native-predictions.npz');bf=np.load(P/'bf16-predictions.npz');repair={};invalid=set()
for side in ['left','right']:
    q=native['action.'+side+'_ee_rpy'][100,0,:,:3]
    for i in range(1,29):
        excursion=np.linalg.norm(q[i]-(q[i-1]+q[i+1])/2)
        neighbour_gap=np.linalg.norm(q[i+1]-q[i-1])
        if excursion>.02 and excursion>2*neighbour_gap:invalid.add(i)
# An action row is shared by the two arms: reject a suspect timestamp jointly.
for side in ['left','right']:
    key='action.'+side+'_ee_rpy';q=native[key][100,0,:,:3].copy();reference=bf[key][100,0,:,:3]
    valid=np.ones(30,dtype=bool)
    for i in invalid:valid[i]=False
    fixed=q.copy()
    for c in range(3):fixed[:,c]=np.interp(np.arange(30),np.flatnonzero(valid),q[valid,c])
    repair[side]=dict(rejected_native_indices=np.flatnonzero(~valid).tolist(),
        raw_max_difference_from_bf16_cm=float(np.max(np.linalg.norm(q-reference,axis=1))*100),
        repaired_max_difference_from_bf16_cm=float(np.max(np.linalg.norm(fixed-reference,axis=1))*100))
report=dict(scope='position-only nearest phase and C2 bridge, synthetic clock; no IK, orientation, contacts or actuation',
    matching='shared index for both hands; position plus velocity difference score',
    methods=metrics,saved_spike_neighbour_repair=repair,
    caveats=['fixed observation is not a task or closed-loop rollout',
             'nearest may reuse the final point and leave no future targets',
             'tail15 diagnostic requires >0.5s tail and <=2cm entry error; neither is an approved robot limit',
             'neighbour spike threshold is only a saved-trace ablation; legitimate motion/contact may be rejected',
             'C2 bridge has no joint, velocity, acceleration or collision constraints'])
(out/'phase-splice-metrics.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
