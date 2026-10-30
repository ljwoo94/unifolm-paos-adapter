"""Run a finite model-only experiment and restore the existing model server.

Only stops the one verified model_runtime/server.py when no TCP client is
connected. Does not access PAOS, robot services or command interfaces.
"""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

FIXED='--fixed-seed'in sys.argv
OUT=Path('/home/wla/paos-rtc-trajectory-20261030/runs')/('rtc-fixed-seed'if FIXED else'rtc-comparison')
OUT.mkdir(parents=True,exist_ok=True)
matches=[]
for p in Path('/proc').iterdir():
    if not p.name.isdigit():continue
    try:
        args=(p/'cmdline').read_bytes().rstrip(b'\0').split(b'\0')
        args=[a.decode()for a in args]
        if 'model_runtime/server.py'in args and str((p/'cwd').resolve())=='/home/wla/paos-model-adapter':
            matches.append((int(p.name),p,args))
    except (OSError,UnicodeDecodeError):pass
if len(matches)!=1:raise RuntimeError('Expected one existing original model server')
pid,proc,args=matches[0]
port=int(args[args.index('--port')+1])
connections=subprocess.check_output(['ss','-Htn','state','established','sport','=',':'+str(port)],text=True)
if connections.strip():raise RuntimeError('Model server has connected clients; experiment not started')
env=dict(x.decode().split('=',1)for x in(proc/'environ').read_bytes().split(b'\0')if x)
cwd=str((proc/'cwd').resolve())
record=dict(original_pid=pid,original_port=port,experiment_exit=None,restored_pid=None,restored_listening=False)
print('STOP_MODEL_SERVER',pid,'no connected clients',flush=True)
os.kill(pid,signal.SIGTERM)
for _ in range(100):
    try:
        if (proc/'stat').read_text().split()[2]=='Z':break
    except FileNotFoundError:break
    time.sleep(.1)
else:raise RuntimeError('Server did not terminate; no force kill performed')
try:
    exp_env=env.copy();exp_env['WMA_TORCH_COMPILE']='0'
    cmd=[args[0],'-u','experiments/rtc_compare.py','--out',str(OUT),'--chunks','4']
    if FIXED:cmd+=['--fixed-seed','--only-int8']
    record['experiment_exit']=subprocess.run(cmd,cwd='/home/wla/paos-rtc-trajectory-20261030',env=exp_env,timeout=240).returncode
finally:
    print('RESTORE_ORIGINAL_MODEL_SERVER',flush=True)
    with (OUT/'restored-server.log').open('ab',buffering=0)as log:
        server=subprocess.Popen(args,cwd=cwd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    record['restored_pid']=server.pid
    for _ in range(120):
        if server.poll()is not None:break
        try:
            with socket.create_connection(('127.0.0.1',port),timeout=.5):pass
            record['restored_listening']=True;break
        except OSError:time.sleep(1)
    (OUT/'server-restoration.json').write_text(json.dumps(record,indent=2)+'\n')
    print('SERVER_RESTORATION',json.dumps(record),flush=True)
    if not record['restored_listening']:raise RuntimeError('Original server restoration failed; see restored-server.log')
sys.exit(record['experiment_exit']or 0)
