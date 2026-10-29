# 실물 G1 Quickstart

WLA에서 모델·Adapter, G1에서 PAOS·Monitor를 실행합니다. 이 문서에는 Simulator나 Kitchen 실행 단계가 없습니다.

## 1. WLA 환경 설치 — 최초 한 번

이미 `/home/wla/paos-model-adapter`에 HTTPS clone했다면 그대로 사용합니다. 경량 SDK `paos_remote-0.2.1-py3-none-any.whl`을 WLA의 `/tmp/`로 전달한 뒤 실행합니다. 이 wheel은 비공개 PAOS 프로젝트에서 제공합니다. 세 VGA raw 영상의 관측에는 SDK 0.2.1과 gRPC 메시지 한도 8 MiB 수정이 적용된 G1 Facade가 모두 필요합니다. 기존 SDK·서버의 2 MiB 한도로는 이 관측을 전달할 수 없습니다.

```sh
cd /home/wla/paos-model-adapter
/home/wla/WMA-JLee/.venv311/bin/python -m venv .venv-adapter
.venv-adapter/bin/python -m pip install /tmp/paos_remote-0.2.1-py3-none-any.whl -e .
mkdir -p private runs
if [ ! -e private/g1.yaml ]; then cp configs/g1-real-base.yaml private/g1.yaml; fi
.venv-adapter/bin/python -m pip check
scripts/serve_wla.sh --check
```

기존 환경과 `private/g1.yaml`이 있으면 venv 생성과 설정 복사는 생략합니다. 현재 WLA의 기존 private 설정에는 arm14·그리퍼 적용이 활성화되어 있으므로 기본 템플릿으로 덮어쓰지 않습니다. 기존 모델 환경은 변경하지 않습니다.

## 2. 실물 설정 확인 — WLA

`private/g1.yaml`은 다음 주소와 첫 시험 지시문을 사용합니다.

```yaml
robot_target: 192.168.123.164:15051
model_server_url: ws://127.0.0.1:18610
instruction: Move forward slowly.
```

새 설치용 `configs/g1-real-base.yaml` 템플릿은 base 속도만 적용합니다. 기존 설치는 `private/g1.yaml`의 팔·그리퍼 설정을 유지합니다. 연결 전 확인은 다음과 같습니다.

```sh
ip route get 192.168.123.164
nmcli device status
```

현재 설치의 유선 연결명은 `wma-g1-direct`, WLA 주소는 `192.168.123.222/24`입니다. 로봇 전원·케이블을 연결하고 이 NetworkManager 연결이 활성화되어야 합니다. WLA의 모델 서버 기동은 로봇 연결 없이 가능합니다.

## 3. 모델 서버 시작 — WLA 터미널 A

이미 `127.0.0.1:18610`에서 실행 중이면 중복으로 시작하지 않습니다. 이번 설치에서 백그라운드로 시작한 서버는 `runs/model-server.pid`와 `runs/model-server.log`에 PID·로그가 있습니다. `tail -f runs/model-server.log`로 확인할 수 있습니다. 서버가 없을 때 다음처럼 실행합니다.

```sh
cd /home/wla/paos-model-adapter
scripts/serve_wla.sh
```

`MODEL_READY`와 `127.0.0.1:18610`의 listening 로그를 기다립니다. 시작 시 저장 입력으로 warmup·수치 검사를 수행합니다. 이 단계는 로봇에 접속하거나 명령을 보내지 않습니다. 기존 W8A8 checkpoint를 사용하며 RTC는 꺼져 있습니다. 기본 compile은 성공 이력이 있는 `baseline`이고, 요청한 옵션은 `WMA_COMPILE_PROFILE=max-autotune scripts/serve_wla.sh`로 선택할 수 있습니다. 실패 시 자동 fallback은 없습니다.

Adapter 0.3.3은 세 카메라의 640×480 raw RGB8과 기존 320×240 QVGA를 지원합니다. 기본 관측 전송은 `rgb8`이며 JPEG는 명시적 옵션입니다. Adapter는 원본 크기를 유지하고 RGB를 공식 입력 BGR로 변환하며, 모델 서버가 기존 checkpoint 전처리에 따라 다시 RGB로 바꾸고 **448×320(너비×높이)**으로 resize합니다. 640×480은 OS 관측·전송 해상도이고 모델 전처리·checkpoint·compile 정책은 바뀌지 않습니다.

## 4. 로봇 준비 — G1 Monitor

최신 PAOS의 실물 HAL은 서비스 기동 시 Unitree 기본 motion service를 반납시킵니다. **서비스를 시작하기 전에 외부 지지를 준비**하고, G1의 OS 실행 가이드에 따라 최신 인계·Homepose 수정을 적용합니다. 이 수정은 현재 실물 재검증을 기다리고 있어, WLA 서버 기동 성공과 로봇 동작 검증은 구분합니다. 서비스 기동만으로 Homepose/WBC를 시작하지는 않습니다.

PAOS가 실물 프로필 `profiles/g1_real_orin_groot.yaml` 또는 설치된 대응 프로필을 사용하고 있는지 확인합니다. 현재 G1은 DDS domain0/eth0, `hal.mode_machine: 5`, 양손 Dex1 설정을 사용합니다. `paos status`에서 state가 존재하고 HAL/WBC가 Green이어야 합니다.

최신 실물 프로필의 기본 Home은 참고 사진을 바탕으로 팔꿈치를 굽히고 전완을 앞으로 둔 nominal 자세입니다. 관절 목표는 PAOS가 정의하며, 아래 준비·실제 자세 확인 절차를 그대로 수행합니다.

1. Monitor에서 Ready/Home pose 실행.
2. 지지를 내려 양발을 바닥에 접지.
3. 같은 Monitor에서 Start WBC.
4. 1초 명령 전환이 끝난 뒤 지지 해제·실제 자세 안정화 확인.
5. Monitor의 Release control / 입력권 반납 완료 확인.

설치·모델 서버 시작은 이 준비 절차를 대신하지 않습니다.

## 5. 실물 관측·추론 확인 — WLA 터미널 B

로봇 준비가 끝난 후 실행합니다. 입력권을 획득하거나 목표를 보내지 않습니다.

```sh
cd /home/wla/paos-model-adapter
RUN_TAG=$(date +%Y%m%d-%H%M%S)
.venv-adapter/bin/unifolm-paos --config private/g1.yaml \
  --probe --seconds 45 --output "runs/probe-$RUN_TAG"
```

`responses ≥ 1`, `goal_count = 0`, `error = null`을 확인합니다. 실패하면 제어를 시작하지 않습니다.

세 640×480 raw RGB 영상은 합계 2,764,800바이트입니다. `pick carrot` 지시문과 상태·배열 메타데이터를 포함한 모델 MessagePack 요청은 2,765,640바이트로 기존 4 MiB WebSocket 한도에 들어갑니다. 허용되는 최대 길이 8,192자의 4바이트 UTF-8 지시문을 사용해도 요청은 2,798,399바이트입니다. 이 크기 검사는 로컬 직렬화 검증이며 실제 로봇·모델 실행 결과는 아닙니다.

## 6. 프롬프트 제어·Stop — WLA 터미널 B

이 명령부터 로봇 목표를 전송합니다. 로봇 안정화와 입력권 반납 후 실행합니다.

```sh
RUN_TAG=$(date +%Y%m%d-%H%M%S)
.venv-adapter/bin/unifolm-paos --config private/g1.yaml \
  --instruction "Move forward slowly." --until-stopped \
  --output "runs/control-$RUN_TAG"
```

운영자가 **터미널 B에서 Ctrl+C**를 누를 때까지 실행합니다. 정상 Stop은 자기 목표 취소·입력권 반납을 먼저 처리하고 추론 worker를 정리합니다. Monitor 창을 닫는 것은 Adapter Stop이 아닙니다. 모델 서버 A는 계속 켜두고, 다음 프롬프트는 새 `RUN_TAG`로 Adapter만 다시 실행합니다. 첫 제어를 5초로 제한하려면 `--until-stopped` 대신 `--seconds 5`를 사용합니다.

시작 시 입력권 소유와 Ctrl+C 반납 안내가 출력됩니다. Ctrl+C 뒤에는 모델 worker 정리를 기다리기 전에 **`input_control_released`**가 출력됩니다. 반납·취소를 확인하지 못하면 `input_control_release_unknown`/`close_error`와 종료 코드1을 반환합니다. 모델 서버·WBC는 이 반납으로 종료되지 않습니다.

Homepose로 돌아가려면 반납 확인 → 외부 지지 → Monitor의 Stop WBC → `last command held` → 별도 Homepose 순서입니다. 최신 PAOS Stop WBC는 직전 관절 목표·gain·피드포워드 토크를 고정하고 원하는 속도만1초 동안0으로 줄입니다. 측정각으로 목표를 바꾸거나 준비 gain으로 낮추지 않습니다. 균형 제어가 아니므로 외부 지지가 필요합니다. 다른 caller의 입력권을 강제로 반환하는 명령은 제공하지 않습니다.

모델에 자동 작업 완료 신호는 없습니다. 추론 오류는 재시도 없이 일시 정지하고 입력권을 유지합니다. 명시적 재개는 `kill -USR1 <Adapter-PID>`, 종료는 Ctrl+C입니다. 로봇 RPC 오류로 세션이 끊긴 경우 자동 재획득하지 않으며 수락된 목표는 원래 TTL을 따릅니다.

## 7. 갱신·종료

Adapter를 Stop한 뒤 갱신합니다. SDK 0.2.1 wheel을 WLA의 `/tmp/`에 먼저 준비하고, G1 Facade에도 8 MiB gRPC 수정이 적용되어 있는지 확인합니다. `private/g1.yaml`은 유지합니다.

```sh
cd /home/wla/paos-model-adapter
git pull --ff-only
.venv-adapter/bin/python -m pip install /tmp/paos_remote-0.2.1-py3-none-any.whl -e .
```

SDK wheel 갱신 안내가 있으면 같은 버전이라도 새 파일로 교체·재설치합니다. Homepose 응답 대기는 최신 SDK에서10초이며 실제 준비 자세 이동5초와는 별개입니다.

```sh
.venv-adapter/bin/python -m pip install --force-reinstall --no-deps /tmp/paos_remote-0.2.1-py3-none-any.whl
```

 모델 서버 코드가 변경됐다면 서버도 재시작합니다. 모든 실험을 마치면 Adapter를 먼저 Stop하고, 터미널 A의 모델 서버를 Ctrl+C로 종료합니다. 이번 설치의 백그라운드 서버는 해당 PID가 이 모델 서버인지 확인한 후 `kill -TERM "$(cat runs/model-server.pid)"`로 종료합니다. 결과는 각 `runs/.../result.json`에 남습니다.

새로 팔·그리퍼 적용을 선택할 때는 `configs/g1-real-arm-gripper.yaml`을 별도 `private/` 설정으로 복사합니다. 현재 WLA에서 이미 arm14·그리퍼를 사용하는 private 설정은 그대로 유지합니다. 프롬프트만 바꿔서는 팔 제어가 켜지지 않습니다. 이 경로는 base3·arm14·Dex1을 적용하며, measured q29 기반 PlaCo IK를 사용합니다. IK 미수렴 근사값도 전송하는 현재 실험 정책이 있어 base 입력 확인과 구분해 검증해야 합니다.
