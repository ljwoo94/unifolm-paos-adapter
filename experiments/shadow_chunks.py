"""Record live observations and complete model chunks without input ownership.

Only RobotClient.status/observe/close are used. No command or acquire call.
Run on WLA after the user prepares the robot/scene and observer endpoint.
"""
import argparse
import json
from pathlib import Path
import time
import numpy as np

from paos_remote import RemoteRobotClient
from unifolm_paos_adapter.runner import load_config
from unifolm_paos_adapter.adapter import build_observation
from unifolm_paos_adapter.kinematics import G1Kinematics
from unifolm_paos_adapter.server_client import ModelClient


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',required=True);parser.add_argument('--output',required=True)
    parser.add_argument('--chunks',type=int,default=8);parser.add_argument('--seed',type=int,default=10000)
    args=parser.parse_args()
    if not 2<=args.chunks<=64:parser.error('--chunks must be 2..64')
    config=load_config(args.config);out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    kin=G1Kinematics();rows=[]
    client=RemoteRobotClient.connect(config['robot_target'],timeout=config['observation_timeout_s'])
    try:
        initial=client.status();run_id=client.run_id
        options=dict(image_encoding=config['image_encoding'],jpeg_quality=config['jpeg_quality'],diagnostics=True)
        if config.get('image_size')is not None:options['image_size']=config['image_size']
        with ModelClient.connect(config['model_server_url'],period_s=config['period_s'],horizon=config['horizon'],timeout=config['model_timeout_s'])as model:
            for index in range(args.chunks):
                requested=time.monotonic();snapshot=client.observe(**options);received=time.monotonic()
                if snapshot['robot']['run_id']!=run_id:raise RuntimeError('OS run changed')
                obs=build_observation(snapshot,kinematics=kin,instruction=config['instruction'],gripper_mapping=config['gripper_mapping'])
                obs['seed']=args.seed
                request_time=time.monotonic()
                try:chunk=model.infer(obs)
                finally:
                    # Explicit diagnostic recording; retain failing inputs too.
                    # Disk writes are after inference, but affect request cadence.
                    np.savez(out/f'{index:03d}-input.npz',**{k:np.asarray(v)for k,v in obs.items()})
                np.savez(out/f'{index:03d}-chunk.npz',**dict(chunk.actions))
                row=dict(index=index,observation_requested_at=requested,observation_received_at=received,
                    model_requested_at=request_time,chunk_received_at=chunk.received_at,
                    seed=args.seed,state=snapshot['robot']['state'],
                    frames={name:frame['metadata']for name,frame in snapshot['frames'].items()},
                    model_timing=model.last_timing)
                rows.append(row)
                (out/'trace.json').write_text(json.dumps(dict(scope='read/infer only; input ownership never acquired; no goals sent',
                    run_id=run_id,initial_status=initial,period_s=config['period_s'],rows=rows,
                    metadata=model.metadata),indent=2,default=lambda x:x.tolist()if hasattr(x,'tolist')else str(x))+'\n')
                print(json.dumps(dict(index=index,scope='shadow_no_commands',observation_to_receipt_s=chunk.received_at-requested)),flush=True)
    except BaseException as exc:
        (out/'capture-error.json').write_text(json.dumps(dict(error=f'{type(exc).__name__}: {exc}',completed_chunks=len(rows)),indent=2)+'\n')
        raise
    finally:client.close()


if __name__=='__main__':main()
