# UniFoLM PAOS Adapter

WLA에서 실행하는 모델 측 Adapter. `paos-remote` 경량 SDK와 공식 UniFoLM WebSocket/NumPy MessagePack 규격을 사용한다. OS·iceoryx2·Torch를 Adapter 의존성으로 요구하지 않는다. 모델 backend는 별도 model_runtime에서 기존 W8A8 bundle/모델 환경을 사용한다.

`configs/adapter.example.yaml`에 robot_target과 model_server_url을 각각 설정한다. 기존 PAOS/SIM에서 명시적으로 Homepose/StartWbc를 완료한 뒤 준비 caller가 입력권을 반납해야 한다. 이 프로그램의 접속은 로봇 준비를 자동 시작하지 않는다.

```sh
unifolm-paos --config configs/adapter.example.yaml --probe --seconds 45 --output runs/probe
unifolm-paos --config configs/adapter.example.yaml --seconds 10 --output runs/control
```

한 제어 세션으로 전체 입력권을 소유하고 base3만 적용한다. 출력7field는 모두 검사·보존한다. 모델 오류는 추론 일시 정지이며 현재 chunk 소진 후 원래 목표 TTL을 따른다. SIGUSR1은 명시적 재개, SIGINT/SIGTERM은 정상 종료다. 재개 전에 기존 제어 세션/run을 확인하며 자동 입력권 재획득·모델 요청 재시도는 없다. 비정상 로봇 RPC 오류는 transport만 폐기한다.

모델 단독 실행은 `model_runtime/benchmark.py`, 공식 입력/출력 변환을 사용한 native W8A8 서버는 `model_runtime/server.py`다. WMA_MODEL_BUNDLE은 기존 checkpoint 경로, WMA_TORCH_COMPILE=1, WMA_COMPILE_PROFILE은 baseline / max-autotune / max-autotune-no-cudagraphs 중 명시적으로 선택한다. max-autotune 실패 시 자동 fallback하지 않는다. model_runtime/source-baseline.json과 PAOS 검증 보고에 원본·변경 범위와 실제 환경을 기록한다.

공식 G1 URDF는 geometry 변경 없이 보관한다. 실제29관절 전체 FK, pelvis 기준 EE9, wrist-local+x0.105m, 명시적 Dex1 native-rad profile을 사용한다. 모델 서버의 normalization을 중복 적용하지 않는다. simulator oracle나 이전 원격 임의 실험의 pose/offset은 입력으로 사용하지 않는다.

Vendor codec 출처/라이선스는 `_vendor/UNIFOLM_LICENSE`, URDF 및 모델 근거/검증 범위는 PAOS 작업의 docs/implementation/remote-model-interface.md와 .scratch/remote-model-interface/research에 있다.
