# UniFoLM 연속 궤적: 요청 시점·IK·유선/Wi-Fi 조사 종합

2026-10-30 KST. 논문·공식 문서·공식 코드와 이미 저장된 PAOS 실행 기록을 대조했다. 이번 조사에서는 모델/로봇 실행, 네트워크 전환, 운영 설정 변경을 하지 않았다.

## 결론

**연속 실행에는 요청 시점과 접합뿐 아니라, IK 관절 구성과 실제 실행 시간의 연속성도 필요하다.** 이전 값을 IK에 주는 것은 유효한 후보지만 seed만 바꾸는 것으로 충분하지 않다. 이전 해를 선호하는 비용, 최종 명령의 시간 제약, 실행 중인 `(q,v,a)` 유지가 다른 역할을 맡는다. 이는 기존 라이브러리 안에서도 구현할 수 있는 패턴이며 라이브러리 교체만으로 해결된다고 볼 수 없다.

공식 UniFoLM-WLA 공개 범위에서 실제 robot IK/command scheduler는 확인되지 않았다. 별도 **Unitree 공식 XR teleoperation**에는 seed·이전 자세 비용·출력 필터를 함께 사용하는 구현이 있다. 이것을 UniFoLM의 실제 배포 방식이라고 부르면 안 된다. 아래 권고는 확인된 공식 기법을 현재 Adapter에 적용하는 설계 제안이며, 현재 사고의 원인이나 수정 성공을 확정한 결과는 아니다.

세부 출처와 버전 검증은 [공식 WLA 조사](official-unifolm-continuity-20261030.md), [IK 기법 조사](ik-continuity-patterns-20261030.md)를 참조한다.

## 1. 무엇을 이어야 하는가

| 연결 지점 | 필요한 연속성 | 예시 실패 |
|---|---|---|
| 모델 chunk → 모델 chunk | 관측/실행 시각, 같은 좌표계의 EE pose, 진행 방향 | 새 chunk를 0번부터 재시작해 이전 동작으로 되돌아감 |
| EE pose → IK 관절 경로 | 이전 유효 관절 구성, 관절 범위·속도·도달 가능성 | 손끝은 비슷한데 팔꿈치/어깨의 다른 해 선택 |
| 관절 경로 → 실행 reference | 현재 실행 중인 위치·속도·가속도와 실제 시간 | 점을 부드럽게 연결했지만 매번 속도를 리셋하거나 너무 짧은 도착 시간 지정 |
| 원격 입력 → 로봇 로컬 제어 | bounded age, deadline, 남은 유효 tail | 네트워크 지연으로 늦은 목표를 쌓아 실행하거나 반복 공백 발생 |

현재 OS에는 C1 `JointReference`가 있지만 모델 목표의 바람직함, IK 해 선택, jerk 제한까지 보장하지 않는다. 이번 기록에서 약 36°의 관절 목표 변화는 전송 목표에서 확인됐으며 실물 각도의 순간 이동을 뜻하지 않는다. [저장 실행 분석](../../../artifacts/control-023502-analysis/findings.md).

## 2. 이전 값을 IK에 알려주는 세 가지 방법

1. **Seed:** 이전 유효 IK 해 또는 실행 reference 근처에서 탐색을 시작한다. 이것은 탐색 시작점이지 최종 해 이동량의 제한이 아니다.
2. **Reference cost:** 손끝 목표 오차와 함께 `||q−q_previous_reference||²`를 비용으로 둔다. 손끝을 비슷하게 맞추는 여러 자세 중 기존 구성을 선호한다. soft cost이므로 여전히 hard bound를 대신하지 못한다.
3. **최종 시간 제약:** 최종 외부 명령을 이전 reference와 비교해 속도·가속도 조건을 적용한다. 내부 반복 한 번의 Δq clamp와 분리해야 한다.

PlaCo 0.10.2의 `add_regularization_task()`는 **한 solve의 Δq**를 벌주는 항이다. `JointsTask`의 목표 자세 비용과 다르다. velocity bound도 각 solve에서 `|Δq|≤vmax·dt`다. 45번의 수치 반복에 매번 전체 0.1초 이동 예산을 주고 합치면, 외부 0.1초의 최종 이동을 제한했다고 볼 수 없다. [고정 버전 regularization](https://github.com/Rhoban/placo/blob/a91e11506fe0909b38d8eff4f0e3d6febb3cc3d3/src/placo/kinematics/regularization_task.cpp), [JointsTask](https://github.com/Rhoban/placo/blob/a91e11506fe0909b38d8eff4f0e3d6febb3cc3d3/src/placo/kinematics/joints_task.cpp), [solver와 시간 한계](https://github.com/Rhoban/placo/blob/a91e11506fe0909b38d8eff4f0e3d6febb3cc3d3/src/placo/kinematics/kinematics_solver.cpp).

`q_measured`, `q_reference`, `q_last_solution`도 구분해야 한다. 측정각은 실제 추종 상태이고, reference는 현재 실행하도록 요청한 상태이며, 마지막 solver 해는 채택되거나 실제 도달했음을 보장하지 않는다. 미수렴 해를 무조건 다음 seed로 삼는 것, 측정각이 reference를 잘 따라간다고 가정하는 것 모두 별도 검증이 필요하다. fresh measured waist/body와 arm planning reference의 갱신 책임도 분리한다.

이번 실제 run에서 입력 q를 관측 요청과 함께만 갱신해 IK seed age가 중앙값 약 0.73초, 최대 1.24초였다. 모델용 영상과 별도로 작은 상태 조회를 갱신하는 방안도 후보지만, 추가 RPC와 frame 정렬을 측정해야 한다. 현재 역사적 사건에는 exact IK q/target이 빠졌고 마지막 chunk의 warm-start 대조 효과도 작았으므로, seed 변경을 입증된 수정으로 취급하지 않는다.

## 3. 다른 공개 시스템의 구체적 처리

### Unitree XR teleoperation

고정 commit의 `G1_29_ArmIK`는 `set_initial`과 `sumsqr(q−q_last)` 비용을 함께 사용하고, 결과에 `WeightedMovingFilter`를 적용한다. measured arm q가 인자로 오면 seed와 q_last reference를 그 관측값으로 덮어쓴다. 그렇지 않을 때 저장한 이전 해를 잇는다. 필터는 시간/jerk 한계를 가진 trajectory generator가 아니다. [공식 IK](https://github.com/unitreerobotics/xr_teleoperate/blob/817fb00c63cde15e5f24a0f8fa08e1e33ed89d3b/teleop/robot_control/robot_arm_ik.py), [공식 filter](https://github.com/unitreerobotics/xr_teleoperate/blob/817fb00c63cde15e5f24a0f8fa08e1e33ed89d3b/teleop/utils/weighted_moving_filter.py).

그대로 복사할 수 있는 로봇 설정은 아니다. 이 예제는 waist를 고정하는 reduced model을 만들며 tool offset도 현재 PAOS와 다르다. **seed+자세 비용+필터라는 처리 패턴**을 참고하되, 현재 사용자의 움직이는 허리·tool frame·URDF 계약을 유지해야 한다.

### MoveIt / Pink

MoveIt Humble은 연속 IK, consistency 제한, jump 검출/경로 단축, time parameterization을 분리한다. Pink는 configuration에서 속도를 풀어 적분하고 이전 속도 기반 acceleration limit를 별도로 제공한다. 둘 다 이전 q 하나만으로 모든 연속성을 보장한다는 접근은 아니다. [MoveIt Cartesian interpolation](https://moveit.picknik.ai/humble/api/html/cartesian__interpolator_8cpp_source.html), [Pink 예제](https://github.com/pink-kinematics/pink/blob/main/examples/arm_ur3.py), [공식 유지관리자 acceleration-limit 논의](https://github.com/orgs/pink-kinematics/discussions/103).

### Ruckig

현재·목표 `(q,v,a)`와 속도·가속도·jerk 한계를 받아 retarget한다. 매 업데이트의 출력 상태를 다음 입력에 이어주는 방식이다. 목표를 바꿀 때 매번 `v=a=0`으로 두면 진행 중인 reference를 잇는 의미가 사라진다. 물리적 한계 때문에 0.1초 도착이 불가능하면 정확한 0.1초 도착과 한계 준수를 동시에 요구할 수 없다. Ruckig는 다른 IK branch 선택이나 충돌 회피를 대신하지 않는다. [공식 tutorial](https://docs.ruckig.com/tutorial.html).

Ruckig 0.19.7 문서상 Community의 point-to-point OTG는 로컬에서 사용할 수 있지만 **intermediate_positions를 넣으면 cloud API**로 전환된다. 로컬 multi-waypoint와 tracking은 Pro 기능이다. 따라서 Community에 전체 chunk waypoint를 넣어 로봇 로컬 realtime 재생이 된다고 제안하지 않는다. 후보를 쓰더라도 지원되는 기능 범위를 먼저 정한다. [waypoint와 tracking 제한](https://docs.ruckig.com/tutorial.html#autotoc_md3).

## 4. 실제 UniFoLM-WLA는 어디까지 확인됐는가

공식 main `0a1aa87be8f775c7883fcd27216e3666ac18a8c5`와 로컬 clean checkout의 전체 tracked blob을 대조했다. 공개 server는 현재 관측으로 전체 chunk를 생성하고, 각 상대 EE pose를 **그 요청의 관측 EE anchor**에 독립 합성한다. 이전 chunk prefix나 IK 해를 인자로 쓰지 않는다. [서버 추론](https://github.com/unitreerobotics/unifolm-wla/blob/0a1aa87be8f775c7883fcd27216e3666ac18a8c5/model_server/action_server_wbc_msgpack_unitree.py#L285-L359).

실제 로봇의 request trigger·IK·DDS·joint smoothing client는 공개 tree에서 확인하지 못했다. 공개 evaluator는 저장 episode에 대한 오프라인 도구여서 그 stride를 실물 소비 정책으로 해석할 수 없다. 데이터 resampling의 B-spline helper도 runtime smoothing 근거가 아니다. 공식 project page는 매끄러운 실물 결과를 소개하지만 실행 구현을 설명하지 않는다. 연결된 공식 자료와 추가 arXiv 검색에서 별도 WLA 기술 논문을 확보하지 못했으며, 존재하지 않는다고 단정하지 않는다. [공식 server/evaluator 설명](https://github.com/unitreerobotics/unifolm-wla/blob/0a1aa87be8f775c7883fcd27216e3666ac18a8c5/docs/train_action_expert_en.md#L146-L168), [공식 project](https://unigen-x.github.io/unifolm-wla.github.io/).

따라서 우리가 만든 EE→PlaCo IK 및 접합 정책을 “공식 UniFoLM 실물 실행기의 재현”이라고 부르면 안 된다. 별도 XR teleop 코드는 유용한 Unitree 공식 참고 구현이지만 동일 배포기라는 근거는 없다.

## 5. 요청 시점은 어떤 값으로 정해야 하는가

고정 fraction 하나보다 **남은 유효 실행 시간 R, 전체 지연 budget L, 여유 M**을 함께 사용한다.

```text
다음 관측을 요청할 수 있는 가장 늦은 시점 = 현재 + R − L − M
```

여기서 L은 GPU inference만이 아니라 관측 수집·영상 전송·전처리·추론·응답·접합·IK·실제 송신 기회까지 포함하는 budget이다. 수락 시각과 실제 controller 적용 시각도 구분해야 한다. `남은 유효 시간`은 단순 전체 chunk 길이가 아니고 지연/phase matching으로 이미 건너뛴 구간을 제외한 값이다.

필요한 최소 동작 진행을 본 뒤 관측하고 싶다면, 그 최소 진행 시점이 위의 가장 늦은 시점보다 앞이어야 한다. **구간이 비면 자동 튜닝으로 해결할 수 없다.** 학습·호환성이 검증된 긴 horizon, 지연 단축, prefix-conditioned generation 또는 계획된 감속/대기가 필요하다.

실제 run에서 1초 chunk, 관측 대기 약 0.667초, 관측→응답 중앙값 약 0.563초였다. 합이 약 1.23초라 0.21–0.26초 공백이 매번 발생했다. 요청을 늦추면 관측 freshness를 개선할 수 있지만, 동시에 continuous coverage가 확보되는 것은 아니다. 순수 async-prefetch도 이전 계획과 시간 정렬/continuation이 있으면 유효한 구조이며, 이 시스템의 문제는 그것 없이 새 응답을 0번부터 교체한 조합이었다. [실제 run 분석](../../../artifacts/control-023502-analysis/analysis.json), [원 RTC의 overlap 원리](https://www.pi.website/research/real_time_chunking).

현재 receipt-origin 재생 시간을 기준으로도 20/30와 1초 horizon은 위 지연에 맞지 않는다. observation-origin prediction 시간을 엄격히 정렬하면 유효 tail은 더 짧아진다. R을 실제 cursor와 action timestamps에서 계산하고, clock domain을 섞어 직접 빼지 않는다.

## 6. Wi-Fi로 바꾸면 무엇이 달라지는가

**로봇의 물리적 한계는 그대로, 전달 성능 budget은 다시 측정해야 한다.** 네트워크 종류만으로 특정 fraction이나 timeout을 정하지 않는다.

| 항목 | 네트워크 변경 시 처리 |
|---|---|
| joint position/velocity/acceleration/jerk 한계·gain | Wi-Fi라는 이유로 변경하지 않는다. 로봇/제어기와 동작 검증 기준 |
| 로봇 내부 HAL/WBC 주기·DDS/iceoryx2 | 원격 연결 종류와 분리. 현재 내부 설정을 Wi-Fi용으로 바꾸지 않는다 |
| request lead time·지연 forecast·유효 buffer headroom | 실제 end-to-end 지연 분포와 jitter에 맞춰 갱신 |
| 영상 해상도·RGB/JPEG·요청 rate | effective goodput, Orin NX encode/resize, WLA decode 비용을 합쳐 비교 |
| RPC/관측/모델 timeout | 실제 실패·기동 비용에 맞춰 검토. 길게 기다리는 것이 오래된 목표의 실행 허가가 되지는 않음 |
| 목표 TTL·관측 freshness | timeout과 별도 계약. 느린 Wi-Fi를 숨기기 위해 무조건 늘리지 않음 |
| 접합 duration | 고정 네트워크 상수가 아니라 현재 `(q,v,a)`와 목표·물리 한계에서 계산 |

현재 구성은 Model+Adapter가 모두 WLA에 있어 model WebSocket은 loopback이다. Wi-Fi의 직접 영향을 받는 것은 **Robot↔WLA의 관측 gRPC와 goal RPC**다. 영상 observer와 goal owner는 별도 SDK client/channel이지만 같은 무선 링크의 대역폭·queue·AP 부하는 공유한다. gRPC 문서는 channel/stub 재사용, 고부하 영역 분리를 권고하지만 Python streaming의 overhead도 별도로 설명한다. 따라서 streaming으로 바꾸면 무조건 빨라진다고 단정하지 않는다. [gRPC performance](https://grpc.io/docs/guides/performance/).

외부 접속 경로가 바뀌면 `robot_target`과 Facade 수신 주소/라우팅을 확인한다. 이것을 Unitree 내부 DDS NIC/domain 변경과 혼동하지 않는다. WLA가 Wi-Fi를 쓴다고 로봇 내부 HAL의 `eth0`를 `wlan0`으로 바꾸는 요구는 아니다.

### 이번 유선 run의 기준값

`control-20261030-023502`, 42 responses / 51.35초의 저장 기록에서:

- 관측 RPC 왕복 중앙값 **114.6ms**, p95 **126.1ms**. 이는 네트워크 순수 RTT가 아니라 상태/카메라 취득·resize·직렬화 등이 포함된다.
- WLA 내부 모델 WebSocket 왕복 중앙값 **443.7ms**, p95 **448.8ms**.
- 4개 순차 명령 RPC의 합계 중앙값 **23.7ms**, p95 **55.8ms**. 이 값은 후반에 남은 256개 목표 구간의 기록이다.
- 3개 448×320 RGB payload는 **1,290,240 bytes/관측**이다. 모델 요청 rate는 약 **0.82Hz**였다. 30Hz는 action 간격과 로컬 camera 생성 주기이지 모델에 매초 30개 영상 묶음을 보냈다는 뜻이 아니다.

이 payload를 옮기는 직렬화 시간의 산술 하한은 1Gbps에서 약 10ms, effective 50Mbps에서 약 206ms다. 프로토콜·경합·재전송·encode/decode를 제외한 예시이며 실제 Wi-Fi 측정이 아니다. JPEG가 줄인 전송 시간보다 Orin NX encode와 WLA decode 시간이 크면 오히려 느려질 수 있으므로 실제 장치에서 비교한다. 카메라 profile 자체를 덮어쓰기보다 이미 지원되는 요청별 크기/encoding 옵션을 평가한다.

위 측정 요약과 원 결과 SHA256은 [보존된 근거](evidence/control-023502-network.json)에 기록했다. Wi-Fi 수치는 측정하지 않았다.

### 고정 유선/무선 숫자 대신 제안하는 운용

1. 실제 영상 payload와 동시 Monitor 트래픽을 포함해 전체 지연의 p50/p95/p99·관측 age·명령 도착 간격·buffer 소진을 기록한다. 평균/ping만으로 budget을 정하지 않는다.
2. 예측 지연과 여유를 보수적으로 갱신하고 남은 tail에서 요청 deadline을 계산한다. 지연 추정이 좋아져도 로봇이 따라가지 못한 상태를 target 도달로 간주하지 않는다.
3. 일정한 주기의 reference 보간은 로봇 로컬에서 유지하고, 원격 패킷 도착 시각이 곧 servo tick이 되지 않게 한다. 현재 SDK의 단일 joint target+duration 이상으로 time-tagged trajectory buffer를 도입하려면 공통 interface·수명 계약 설계가 필요하다. 이미 구현됐다고 주장하지 않는다.
4. 기존 계약인 모델 실패 시 pause/no automatic retry, control-session 종료 시 입력권 회수, 기존 목표 TTL 독립 유지와 분리한다. timeout이 난 command는 outcome unknown일 수 있어 자동 재송신으로 해결하지 않는다. 앱 heartbeat API를 새로 요구하지 않는다. gRPC keepalive는 transport 연결 확인이며 100Hz 제어 signal로 쓰는 기능이 아니다. [Deadlines](https://grpc.io/docs/guides/deadlines/), [Keepalive](https://grpc.io/docs/guides/keepalive/).

## 7. 현재 프로젝트에 적용할 순서

1. exact IK q/target/이전 유효 reference를 저장해 같은 문제를 로봇 없이 재현한다. 기록 보강 커밋 `3579389`는 준비만 됐고 운영 적용은 아직 하지 않았다.
2. 동일 입력에서 seed 변경, 고정 previous-reference 자세 비용, 최종 변위/시간 제한을 개별 대조한다. solver 교체나 필터 하나를 성공으로 가정하지 않는다.
3. EE 경로 접합과 IK 경로를 함께 검증하고, 실제 실행 reference `(q,v,a)`에서 retarget한다. pose interpolation은 SO(3)/SE(3) 의미를 유지한다. gripper event를 관절 평균으로 섞거나 최근접 index 선택으로 임의 생략하지 않는다.
4. wired/Wi-Fi 프로파일에서 영상 전송과 전체 timing budget을 측정하고 요청 scheduling을 갱신한다. 물리 gain을 네트워크 보상값으로 사용하지 않는다.

이 순서는 조사로 도출한 권고다. 이번 run의 급동작 근본 원인이 입증되거나 구현/실물 검증이 완료됐다는 의미는 아니다.
