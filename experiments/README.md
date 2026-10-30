# RTC trajectory experiment

These scripts are isolated experiments; they are not an enabled serving feature.
No script sends robot goals. `shadow_chunks.py` only reads robot observations.
RTC server requests remain rejected by the normal serving baseline.

`rtc_compare.py` applies the original RTC guidance/soft-mask equations to the
existing four-step head. W8A8 uses an explicit approximate STE backward. A second
ablation dequantizes only head weights into BF16 compute; this does **not** restore
original BF16 weights. Compare these labels separately. Each method uses matched
seeds. `--fixed-seed --only-int8` tests seed 10000 on every request.

The fixed-observation experiment uses a synthetic 0.333 s delay and 0.5 s stride.
Measured RTC head computation exceeds that assumed delay. Plots demonstrate the
continuation mechanism, **not a deployable schedule or task success**.

`with_server_restore.py` is specific to this WLA installation. It refuses to run
if the existing model server has a connected TCP client, stops only that verified
process, runs a bounded experiment, and restores the original argv/environment
in `finally`. It never controls robot or PAOS services.

## Next capture on the prepared robot

On WLA, once PAOS observation access and the scene are prepared:

```sh
cd /home/wla/paos-rtc-trajectory-20261030
PYTHONPATH=src /home/wla/paos-model-adapter/.venv-adapter/bin/python \
  experiments/shadow_chunks.py \
  --config /home/wla/paos-model-adapter/configs/g1-real-arm-gripper.yaml \
  --output runs/robot-shadow-01 --chunks 8 --seed 10000
```

The checked config uses `192.168.123.164:15051` and
`ws://127.0.0.1:18610`. This command does not acquire input ownership, start/stop
WBC or move the robot. It saves each exact model input, complete returned chunk,
robot state, frame metadata and request/receipt times. Output directory must be
new. Captures are bounded to 2–64 chunks; disk recording affects request cadence
and is not a production performance benchmark. It stops on model/observation
failure without retrying. Confirm live connectivity when the environment is ready;
only offline fake-client flow and the installed `--help` path were tested so far.

Offline recorder check (no network):

```sh
PYTHONPATH=src /home/wla/paos-model-adapter/.venv-adapter/bin/python experiments/check_shadow.py
```

`check_saved_spike.py` exits 1 when it finds the known stored single-chunk spike;
that is the expected reproduction signal, not a failed installation.
