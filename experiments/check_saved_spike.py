"""Diagnose the saved spike without any model/robot connection.
Exit 1 reproduces a native-only >5cm midpoint excursion absent from BF16.
Threshold is a diagnostic signal, not a real-robot operating limit.
"""
import json
from pathlib import Path
import sys
import numpy as np

p=Path('/home/wla/WMA-JLee/models/C-W8A8-20261028')
native=np.load(p/'native-predictions.npz');bf=np.load(p/'bf16-predictions.npz')
results={};red=False
for side in ['left','right']:
    key='action.'+side+'_ee_rpy'
    q=native[key][:,0,:,:3];b=bf[key][:,0,:,:3]
    midpoint=np.linalg.norm(q[:,1:-1]-(q[:,:-2]+q[:,2:])/2,axis=-1)
    baseline=np.linalg.norm(b[:,1:-1]-(b[:,:-2]+b[:,2:])/2,axis=-1)
    sample,interior=np.unravel_index(np.argmax(midpoint),midpoint.shape);step=interior+1
    results[side]=dict(sample=int(sample),native_step=int(step),native_time_s=step/30,
        native_midpoint_excursion_cm=float(midpoint[sample,interior]*100),
        bf16_midpoint_excursion_cm=float(baseline[sample,interior]*100))
    red|=midpoint[sample,interior]>.05 and baseline[sample,interior]<.01
print(json.dumps(dict(saved_chunk_spike_detected=bool(red),details=results),indent=2))
sys.exit(1 if red else 0)
