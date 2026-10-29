"""Launch command/env checks only: no GPU, model or robot is started."""
from pathlib import Path
import os
import shutil
import subprocess

ROOT=Path(__file__).resolve().parents[1]


def test_wla_launcher_uses_selected_checkpoint_and_preserves_compile_override(tmp_path):
    repo=tmp_path/'adapter';(repo/'scripts').mkdir(parents=True)
    shutil.copy(ROOT/'scripts/serve_wla.sh',repo/'scripts/serve_wla.sh')
    bundle=tmp_path/'model bundle';bundle.mkdir();(bundle/'model.safetensors').touch()
    python=tmp_path/'model python';python.write_text('#!/bin/sh\nprintf "%s\\n" "$WMA_MODEL_BUNDLE" "$WMA_COMPILE_PROFILE" "$@"\n');python.chmod(0o755)
    env=dict(os.environ,WMA_MODEL_PYTHON=str(python),WMA_MODEL_BUNDLE=str(bundle),WMA_COMPILE_PROFILE='max-autotune')
    result=subprocess.run(['sh',str(repo/'scripts/serve_wla.sh')],env=env,text=True,capture_output=True)
    assert result.returncode==0,result.stderr
    assert result.stdout.splitlines()==[str(bundle),'max-autotune','-u','model_runtime/server.py','--host','127.0.0.1','--port','18610']
    checked=subprocess.run(['sh',str(repo/'scripts/serve_wla.sh'),'--check'],env=env,text=True,capture_output=True)
    assert checked.returncode==0 and 'model_runtime/server.py'not in checked.stdout


def test_wla_launcher_rejects_missing_checkpoint_before_model_execution(tmp_path):
    python=tmp_path/'python';python.write_text('#!/bin/sh\ntouch "$WAS_STARTED"\n');python.chmod(0o755)
    marker=tmp_path/'started'
    env=dict(os.environ,WMA_MODEL_PYTHON=str(python),WMA_MODEL_BUNDLE=str(tmp_path/'missing'),WAS_STARTED=str(marker))
    result=subprocess.run(['sh',str(ROOT/'scripts/serve_wla.sh')],env=env,text=True,capture_output=True)
    assert result.returncode!=0 and not marker.exists()
