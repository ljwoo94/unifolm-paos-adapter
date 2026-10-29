"""Live Adapter with a time limit or operator Stop. SIGUSR1 resumes inference."""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import signal
import threading
import time
from urllib.parse import urlparse

import numpy as np
import yaml
from paos_remote import RemoteRobotClient, RemoteError
from .adapter import build_observation, frame_rgb
from .execution import ChunkExecution
from .kinematics import G1Kinematics
from .server_client import ModelClient


def load_config(path, *, instruction=None):
    value = yaml.safe_load(Path(path).read_text())
    if instruction is not None and isinstance(value,dict):value['instruction']=instruction
    required = {'robot_target', 'model_server_url', 'instruction', 'period_s', 'gripper_mapping'}
    if not isinstance(value, dict) or not required <= value.keys() or set(value) - required - {'horizon', 'model_timeout_s', 'rpc_timeout_s', 'observation_timeout_s', 'command_hz', 'projection', 'diagnostics', 'record_inputs', 'image_encoding', 'jpeg_quality', 'image_size'}:
        raise ValueError('invalid Adapter configuration fields')
    target = urlparse('//' + str(value['robot_target']))
    model = urlparse(str(value['model_server_url']))
    try:
        if not target.hostname or not target.port or target.path or target.username or target.query or target.fragment:
            raise ValueError('robot_target must be host:port')
        if model.scheme not in {'ws', 'wss'} or not model.hostname or model.username or model.fragment:
            raise ValueError('model_server_url must be an explicit WebSocket URL')
        _ = model.port
    except ValueError as exc:
        raise ValueError('invalid endpoint configuration') from exc
    value.setdefault('horizon', 30)
    value.setdefault('model_timeout_s', 30.)
    value.setdefault('rpc_timeout_s', 2.)
    value.setdefault('observation_timeout_s',value['rpc_timeout_s'])
    value.setdefault('command_hz',30.)
    value.setdefault('diagnostics',False)
    value.setdefault('record_inputs',False)
    size=value.get('image_size')
    if size is not None and (not isinstance(size,list) or len(size)!=2 or any(type(v)is not int or not 0<v<=limit for v,limit in zip(size,(640,480)))):
        raise ValueError('image_size must be [width,height] within 640x480')
    value.setdefault('image_encoding','rgb8');value.setdefault('jpeg_quality',80)
    if value['image_encoding']not in ('rgb8','jpeg')or type(value['jpeg_quality'])is not int or not 1<=value['jpeg_quality']<=95:raise ValueError('unsupported image encoding/quality')
    for field in ('period_s', 'model_timeout_s', 'rpc_timeout_s', 'observation_timeout_s', 'command_hz'):
        v = value[field]
        if type(v) not in (int, float) or not math.isfinite(v) or v <= 0:
            raise ValueError(f'positive finite {field} required')
    if value['command_hz']>100 or any(type(value[k])is not bool for k in ['diagnostics','record_inputs']):raise ValueError('bounded rate and boolean diagnostics required')
    if 'projection'in value:
        projection=value['projection']
        if projection.get('kind')!='arm_ee_gripper' or not .1<=projection.get('duration_s',0)<=30:
            raise ValueError('explicit arm/gripper projection and supported reference duration required')
        for side in ['left','right']:
            bounds=projection['gripper_endpoints'][side]
            if len(bounds)!=2 or not np.isfinite(bounds).all() or not bounds[0]<bounds[1]:raise ValueError('native-rad gripper endpoints required')
    if type(value['horizon']) is not int or not 0 < value['horizon'] <= 512:
        raise ValueError('bounded integer horizon required')
    if not isinstance(value['instruction'], str) or not value['instruction'].strip():
        raise ValueError('instruction required')
    mapping = value['gripper_mapping']
    if not isinstance(mapping, dict) or set(mapping) != {'kind', 'offsets'} or mapping['kind'] != 'dex1_native_rad':
        raise ValueError('explicit Dex1 native-rad mapping required')
    offsets = mapping['offsets']
    if not isinstance(offsets, dict) or set(offsets) != {'left', 'right'} or any(
            type(v) not in (int, float) or not math.isfinite(v) for v in offsets.values()):
        raise ValueError('finite left/right gripper offsets required')
    return value


def run(config, *, seconds, output, probe=False, stop=None, resume=None):
    if seconds is not None and (type(seconds) not in (int, float) or not math.isfinite(seconds) or not 0 < seconds <= 3600):
        raise ValueError('finite duration in (0,3600] or None for operator Stop required')
    if probe and seconds is None:raise ValueError('probe requires a finite duration')
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    stop = stop or threading.Event()
    resume = resume or threading.Event()
    kinematics = G1Kinematics()
    timeout = config['rpc_timeout_s']
    observer = RemoteRobotClient.connect(config['robot_target'], timeout=config.get('observation_timeout_s',timeout))
    owner = None
    latest_body=[None];observation_trace=deque(maxlen=256);observation_count=[0]
    projection=None
    if config.get('projection'):
        from .projection import ArmGripperProjection
        projection=ArmGripperProjection(config['projection']['gripper_endpoints'])
    if config.get('record_inputs'):(output/'inputs').mkdir()
    def observe():
        started=time.monotonic()
        options={}
        if config.get('diagnostics'):options['diagnostics']=True
        if 'image_encoding'in config:options.update(image_encoding=config['image_encoding'],jpeg_quality=config.get('jpeg_quality',80))
        if config.get('image_size') is not None:options['image_size']=config['image_size']
        snapshot=observer.observe(**options)
        received=time.monotonic()
        result=build_observation(snapshot, kinematics=kinematics, instruction=config['instruction'],
                                 gripper_mapping=config['gripper_mapping'])
        if not (config.get('diagnostics')or projection or config.get('record_inputs')):return result
        latest_body[0]=dict(q=np.asarray(snapshot['robot']['state']['q'],dtype=float),state_seq=snapshot['robot']['state']['state_seq'],received_at=received)
        observation_count[0]+=1
        row=dict(index=observation_count[0],requested_at=started,received_at=received,
            roundtrip_s=received-started,state_seq=snapshot['robot']['state']['state_seq'],
            frames={name:{key:frame['metadata'][key]for key in ['sequence','source_sequence','source_epoch','rgb_time_ns','received_ns','published_ns']}for name,frame in snapshot['frames'].items()},
            server=snapshot.get('diagnostics'),client=snapshot.get('client_timing'))
        if config.get('record_inputs') and row['index']<=256:
            for name,frame in snapshot['frames'].items():
                np.save(output/'inputs'/f"{row['index']:04d}-{name}.npy",frame_rgb(frame))
        row['conversion_and_recording_s']=time.monotonic()-received
        observation_trace.append(row)
        return result
    player = ChunkExecution(model_factory=lambda: ModelClient.connect(config['model_server_url'],
        period_s=config['period_s'], horizon=config['horizon'], timeout=config['model_timeout_s']), observe=observe)
    report = dict(scope=('live PAOS observations -> WLA model; base3 and experimental arm/gripper projection'if projection else 'live PAOS observations -> WLA model; base3 only') if not probe else 'live observation/model probe; no input ownership or commands',
        configuration=config, run_id=observer.run_id, goal_count=0, paused=False, close_error=None,
        input_control='not_acquired',
        goals=[], duration_limit_s=seconds, robot_status_before=observer.status())
    goals = deque(maxlen=256)
    player_started = False
    latest = None
    normal_stop = False
    try:
        if not probe:
            owner = RemoteRobotClient.connect(config['robot_target'], timeout=timeout)
            if owner.run_id != observer.run_id:
                raise RuntimeError('OS restarted before input acquisition')
            owner.acquire_control()
            report['input_control'] = 'owned'
            print(json.dumps(dict(event='input_control_owned',run_id=owner.run_id,
                release_control='Ctrl+C in this Adapter terminal cancels its goals and releases input; WBC and the separate model server keep running')),flush=True)
            if not owner.status()['enabled']:
                raise RuntimeError('explicit Homepose/start_wbc preparation required before Adapter execution')
        player.start()
        player_started = True
        started = time.monotonic()
        report['started_at']=started
        next_tick = started
        previous_pause = False
        while not stop.is_set() and (seconds is None or time.monotonic() - started < seconds):
            if resume.is_set():
                resume.clear()
                # A current status call rejects a changed OS run; never reacquire automatically.
                if owner is not None:
                    if not owner.control_active:
                        raise RuntimeError("cannot resume without the current control session")
                    owner.status()
                    if not owner.control_active:
                        raise RuntimeError("control session ended during resume validation")
                player.resume()
            paused = player.paused
            if paused != previous_pause:
                print(json.dumps(dict(paused=paused, error=player.error)), flush=True)
                previous_pause = paused
            sample=player.current_sample()if projection else None
            goal=(tuple(float(v)for v in sample[0].actions['action.base_command'][0,sample[1],:3])if sample else None)if projection else player.current_goal()
            if goal is not None and owner is not None:
                begin = time.monotonic()
                row=dict(at_s=begin-started,attempted_at=begin,velocity=goal,stages={})
                goals.append(row)
                body=latest_body[0]
                projected=projection.targets(sample[0],sample[1],body['q'])if projection else None
                def dispatch(name,call):
                    stage=dict(outcome='attempted');row['stages'][name]=stage
                    before=time.monotonic()
                    try:
                        value=call()
                    except RemoteError as error:
                        stage.update(outcome='unknown'if error.code in (3,9)else'rejected',code=error.code,error=str(error)[:512])
                        raise
                    except Exception as error:
                        stage.update(outcome='unknown',error=f'{type(error).__name__}: {error}'[:512])
                        raise
                    else:
                        stage.update(outcome='accepted',run_id=value.run_id,goal_ref=value.goal_ref)
                        return value
                    finally:
                        stage['rpc_s']=time.monotonic()-before
                accepted=dispatch('base',lambda:owner.set_velocity(*goal))
                report['goal_count'] += 1
                row.update(run_id=accepted.run_id,goal_ref=accepted.goal_ref,accepted_ns=accepted.accepted_ns,expires_ns=accepted.expires_ns)
                if projected is not None:
                    row.update(chunk_received_at=sample[0].received_at,sample_index=sample[1],projection=projected,
                        ik_state_seq=body['state_seq'],ik_observation_age_s=begin-body['received_at'])
                    arm=dispatch('arms',lambda:owner.set_joint_positions(projected['positions'],duration_s=config['projection']['duration_s']))
                    row['arm_goal_ref']=arm.goal_ref
                    row['gripper_goal_refs']={}
                    for side,opening in projected['grippers'].items():
                        grip=dispatch('gripper_'+side,lambda side=side,opening=opening:owner.set_gripper(side,opening))
                        row['gripper_goal_refs'][side]=grip.goal_ref
                row['rpc_s']=sum(v['rpc_s']for v in row['stages'].values())
                row['tick_elapsed_s']=time.monotonic()-begin
            latest = player.latest_chunk()
            if probe and (latest is not None or player.paused):
                break
            send_period=1/config.get('command_hz',1/config['period_s'])
            next_tick += send_period
            now = time.monotonic()
            if next_tick <= now:
                next_tick += (math.floor((now-next_tick)/send_period)+1)*send_period
            stop.wait(max(0., next_tick-time.monotonic()))
        normal_stop = True
        report['stop_reason']='operator_stop' if stop.is_set() else ('probe_end' if probe else 'duration_elapsed')
        report['elapsed_s'] = time.monotonic()-started
    except BaseException as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
        raise
    finally:
        report['goals'] = list(goals)
        cleanup_errors = []
        # Stop control before waiting for model/observation cleanup.
        try:
            if owner is not None:
                if normal_stop:
                    owner.close()
                    report['input_control'] = 'released'
                    print(json.dumps(dict(event='input_control_released',run_id=owner.run_id,
                        wbc='unchanged',model_server='unchanged')),flush=True)
                else:
                    owner.abandon()
                    report['input_control'] = 'session_ended_without_cancellation'
        except Exception as exc:
            report['close_error'] = f'{type(exc).__name__}: {exc}'
            report['input_control'] = 'unknown'
            print(json.dumps(dict(event='input_control_release_unknown',error=report['close_error'])),flush=True)
        try:
            if player_started:
                player.stop()
                player.join(timeout=config['model_timeout_s']+2)
                report['model_worker_stopped'] = not player.alive
                run_error=report.get('error')
                snapshot=player.snapshot()
                report.update(snapshot)
                report['model_error']=snapshot.get('error')
                if run_error is not None:report['error']=run_error
        except Exception as exc:
            cleanup_errors.append(f'model cleanup: {exc}')
        try:
            if latest is not None:
                np.savez(output/'latest-actions.npz', **dict(latest.actions))
                report['action_shapes'] = {k:list(v.shape) for k,v in latest.actions.items()}
        except Exception as exc:
            cleanup_errors.append(f'action export: {exc}')
        try:
            report['robot_status_after']=observer.status()
        except Exception as exc:
            cleanup_errors.append(f'final status: {exc}')
        try:
            observer.close()
        except Exception as exc:
            cleanup_errors.append(f'observer cleanup: {exc}')
        report['cleanup_errors'] = cleanup_errors
        report['normal_stop'] = normal_stop
        report['observations']=list(observation_trace)
        report['observation_snapshot_complete']=not player.alive
        report['attempt_count']=len(goals)
        (output/'result.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    duration=parser.add_mutually_exclusive_group()
    duration.add_argument('--seconds', type=float, default=10)
    duration.add_argument('--until-stopped',action='store_true',help='keep control until Ctrl+C/SIGTERM; no automatic task completion')
    parser.add_argument('--instruction',help='task prompt override; preserve configured endpoints')
    parser.add_argument('--output', required=True)
    parser.add_argument('--probe', action='store_true', help='read and infer once; never acquire input or send goals')
    args = parser.parse_args()
    if args.probe and args.until_stopped:parser.error('--probe requires a finite --seconds limit')
    stop = threading.Event()
    resume = threading.Event()
    for name in ('SIGINT', 'SIGTERM'):
        signal.signal(getattr(signal, name), lambda *_: stop.set())
    if hasattr(signal, 'SIGUSR1'):
        signal.signal(signal.SIGUSR1, lambda *_: resume.set())
    result = run(load_config(args.config,instruction=args.instruction), seconds=None if args.until_stopped else args.seconds, output=args.output,
                 probe=args.probe, stop=stop, resume=resume)
    print(json.dumps({k:result.get(k) for k in ('scope','run_id','responses','goal_count','paused','error','close_error','stop_reason','input_control')}), flush=True)
    if result.get('close_error') is not None or not result.get('responses') or (not args.probe and not result['goal_count']):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
