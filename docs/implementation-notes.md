# 구현 배경과 이전 SIM 실험

WLA에서 실행하는 모델 측 Adapter. `paos-remote` 경량 SDK와 공식 UniFoLM WebSocket/NumPy MessagePack 규격을 사용한다. OS·iceoryx2·Torch를 Adapter 의존성으로 요구하지 않는다. 모델 backend는 별도 model_runtime에서 기존 W8A8 bundle/모델 환경을 사용한다.

`configs/adapter.example.yaml`에 robot_target과 model_server_url을 각각 설정한다. 기존 PAOS/SIM에서 명시적으로 Homepose/StartWbc를 완료한 뒤 준비 caller가 입력권을 반납해야 한다. 이 프로그램의 접속은 로봇 준비를 자동 시작하지 않는다.

```sh
unifolm-paos --config configs/adapter.example.yaml --probe --seconds 45 --output runs/probe
unifolm-paos --config configs/adapter.example.yaml \
  --instruction "Move forward slowly." --until-stopped --output runs/control
```

`--until-stopped`는 운영자가 Adapter 터미널에서 Ctrl+C를 누를 때까지 실행한다. 정상 종료는 자기 목표 취소·입력권 반납을 모델 worker 정리보다 먼저 처리한다. 모델 서버는 계속 켜두고, 다음 프롬프트는 새 output 경로로 Adapter를 다시 실행한다. 모델 응답에 완료 신호가 없으므로 작업 성공을 자동 판정하지 않는다. 짧은 시험은 `--seconds 5`로 제한할 수 있다. `--instruction`은 설정 파일과 연결 주소를 변경하지 않는 프롬프트 덮어쓰기다.

한 제어 세션으로 전체 입력권을 소유하고 기본 설정에서는 base3만 적용한다. 출력7field는 모두 검사·보존한다. 모델 오류는 추론 일시 정지이며 현재 chunk 소진 후 원래 목표 TTL을 따른다. SIGUSR1은 명시적 재개, SIGINT/SIGTERM은 정상 종료다. 재개 전에 기존 제어 세션/run을 확인하며 자동 입력권 재획득·모델 요청 재시도는 없다. 비정상 로봇 RPC 오류는 transport만 폐기한다.

모델 단독 실행은 `model_runtime/benchmark.py`, 공식 입력/출력 변환을 사용한 native W8A8 서버는 `model_runtime/server.py`다. WMA_MODEL_BUNDLE은 기존 checkpoint 경로, WMA_TORCH_COMPILE=1, WMA_COMPILE_PROFILE은 baseline / max-autotune / max-autotune-no-cudagraphs 중 명시적으로 선택한다. max-autotune 실패 시 자동 fallback하지 않는다. model_runtime/source-baseline.json과 PAOS 검증 보고에 원본·변경 범위와 실제 환경을 기록한다.

공식 G1 URDF는 geometry 변경 없이 보관한다. 실제29관절 전체 FK, pelvis 기준 EE9, wrist-local+x0.105m, 명시적 Dex1 native-rad profile을 사용한다. 모델 서버의 normalization을 중복 적용하지 않는다. simulator oracle나 이전 원격 임의 실험의 pose/offset은 입력으로 사용하지 않는다.

Vendor codec 출처/라이선스는 `_vendor/UNIFOLM_LICENSE`, URDF 및 모델 근거/검증 범위는 PAOS 작업의 docs/implementation/remote-model-interface.md와 .scratch/remote-model-interface/research에 있다.

Kitchen 실험 설정 `configs/kitchen-rgb.yaml`/`kitchen-jpeg.yaml`은 base3에 더해 실험용 EE→arm14 IK와 Dex1 gripper projection을 켠다. 공통 Facade는 typed joint/torso/cancel 요청을 기존 PAOS API로 전달한다. IK는 measured q29를 seed로 쓰는 URDF 기반 PlaCo0.10.2/Pinocchio3.8.0 QP이며 공식 배포 IK·충돌 계획은 아니다. 수렴하지 않은 경우도 제한 안의 근사 관절값을 residual/수렴 flag와 함께 전송하는 현재 실험 정책에 주의한다. waist/lower15/height/pivot까지 재현한 whole-body 연결은 아직 아니다.

실제 작업 추론은 Ready/Homepose→발 접지→WBC→줄 지지 해제와 자세 안정화가 완료된 후 시작한다. optional JPEG는 Orin NX에서 raw 대비 우위가 검증된 기본값이 아니다. Mac/WLA Wi-Fi 시험과 Orin NX/WLA1Gbps Ethernet 측정은 분리한다. `record_inputs`와 diagnostics는 관측/모델/명령별 측정과256bundle 이내 입력 보관을 위한 실험 옵션이다.

0.3.0부터 팔 IK 기본 backend는 PlaCo다. 기존 자체 DLS는 패키지에서 제거하고 비교용 commit29dfbf3에 보존했다. 측정 waist/lower15와 floating base를 고정하고 양팔만 함께 최적화한다. [성능 비교와 재현 방법](ik-performance.md)에 수치·제약을 기록했다. WLA/실물에 자동 배포하지 않는다.

실물 G1 실행용 설정은 `configs/g1-real-base.yaml`, `configs/g1-real-arm-gripper.yaml`입니다. `scripts/serve_wla.sh`가 기존 W8A8 checkpoint·Python·compile 환경을 선택합니다. 기본 로봇 주소는 유선192.168.123.164:15051이며 실제 WLA→G1 네트워크 도달성을 먼저 확인합니다. 모델/Adapter를 시작하는 것만으로 Homepose/WBC 준비나 입력권 반납이 이루어지지는 않습니다.
