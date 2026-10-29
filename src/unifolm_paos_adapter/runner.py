"""Finite live Adapter. SIGUSR1 explicitly resumes paused model inference."""
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
from paos_remote import RemoteRobotClient
from .adapter import build_observation
from .execution import ChunkExecution
from .kinematics import G1Kinematics
from .server_client import ModelClient


def load_config(path):
    value = yaml.safe_load(Path(path).read_text())
    required = {'robot_target', 'model_server_url', 'instruction', 'period_s', 'gripper_mapping'}
    if not isinstance(value, dict) or not required <= value.keys() or set(value) - required - {'horizon', 'model_timeout_s', 'rpc_timeout_s'}:
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
    for field in ('period_s', 'model_timeout_s', 'rpc_timeout_s'):
        v = value[field]
        if type(v) not in (int, float) or not math.isfinite(v) or v <= 0:
            raise ValueError(f'positive finite {field} required')
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
    if type(seconds) not in (int, float) or not math.isfinite(seconds) or not 0 < seconds <= 3600:
        raise ValueError('finite experiment duration in (0,3600] required')
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    stop = stop or threading.Event()
    resume = resume or threading.Event()
    kinematics = G1Kinematics()
    timeout = config['rpc_timeout_s']
    observer = RemoteRobotClient.connect(config['robot_target'], timeout=timeout)
    owner = None
    def observe():
        snapshot = observer.observe()
        return build_observation(snapshot, kinematics=kinematics, instruction=config['instruction'],
                                 gripper_mapping=config['gripper_mapping'])
    player = ChunkExecution(model_factory=lambda: ModelClient.connect(config['model_server_url'],
        period_s=config['period_s'], horizon=config['horizon'], timeout=config['model_timeout_s']), observe=observe)
    report = dict(scope='live PAOS observations -> WLA model; base3 only' if not probe else 'live observation/model probe; no input ownership or commands',
        configuration=config, run_id=observer.run_id, goal_count=0, paused=False, close_error=None,
        goals=[], robot_status_before=observer.status())
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
            if not owner.status()['enabled']:
                raise RuntimeError('explicit Homepose/start_wbc preparation required before Adapter execution')
        player.start()
        player_started = True
        started = time.monotonic()
        next_tick = started
        previous_pause = False
        while not stop.is_set() and time.monotonic() - started < seconds:
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
            goal = player.current_goal()
            if goal is not None and owner is not None:
                begin = time.monotonic()
                accepted = owner.set_velocity(*goal)
                report['goal_count'] += 1
                goals.append(dict(at_s=begin-started, run_id=accepted.run_id,
                    goal_ref=accepted.goal_ref, velocity=goal, expires_ns=accepted.expires_ns,
                    rpc_s=time.monotonic()-begin))
            latest = player.latest_chunk()
            if probe and (latest is not None or player.paused):
                break
            next_tick += config['period_s']
            now = time.monotonic()
            if next_tick <= now:
                next_tick += (math.floor((now-next_tick)/config['period_s'])+1)*config['period_s']
            stop.wait(max(0., next_tick-time.monotonic()))
        report['elapsed_s'] = time.monotonic()-started
        report['robot_status_after'] = observer.status()
        report.update(player.snapshot())
        report['goals'] = list(goals)
        if latest is not None:
            np.savez(output/'latest-actions.npz', **dict(latest.actions))
            report['action_shapes'] = {k:list(v.shape) for k,v in latest.actions.items()}
        normal_stop = True
    except BaseException as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
        raise
    finally:
        report['goals'] = list(goals)
        cleanup_errors = []
        try:
            if player_started:
                player.stop()
                player.join(timeout=config['model_timeout_s']+2)
                report['model_worker_stopped'] = not player.alive
                report.update(player.snapshot())
        except Exception as exc:
            cleanup_errors.append(f'model cleanup: {exc}')
        try:
            if owner is not None:
                if normal_stop:
                    owner.close()
                else:
                    owner.abandon()
        except Exception as exc:
            report['close_error'] = str(exc)
        try:
            observer.close()
        except Exception as exc:
            cleanup_errors.append(f'observer cleanup: {exc}')
        report['cleanup_errors'] = cleanup_errors
        report['normal_stop'] = normal_stop
        (output/'result.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--seconds', type=float, default=10)
    parser.add_argument('--output', required=True)
    parser.add_argument('--probe', action='store_true', help='read and infer once; never acquire input or send goals')
    args = parser.parse_args()
    stop = threading.Event()
    resume = threading.Event()
    for name in ('SIGINT', 'SIGTERM'):
        signal.signal(getattr(signal, name), lambda *_: stop.set())
    if hasattr(signal, 'SIGUSR1'):
        signal.signal(signal.SIGUSR1, lambda *_: resume.set())
    result = run(load_config(args.config), seconds=args.seconds, output=args.output,
                 probe=args.probe, stop=stop, resume=resume)
    print(json.dumps({k:result.get(k) for k in ('scope','run_id','responses','goal_count','paused','error','close_error')}), flush=True)
    if not result.get('responses') or (not args.probe and not result['goal_count']):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
