# 이전 관절값을 사용하는 IK와 action chunk 실행 연속성

조사일: 2026-10-30 KST. 범위: 기존 PlaCo 백엔드의 연속성 설계에 필요한 공식 문서·소스 조사. 설치, 벤치마크, 로봇 실행, 런타임 코드 변경은 하지 않았다.

## 결론

**이전 관절값은 도움이 되지만, 어디에 사용하는지가 중요하다.** 시작점(seed), 해가 선호할 자세(reference), 시간에 따른 이동 한계(constraint), 실제 실행 궤적(stateful trajectory)은 서로 다른 계약이다. 이전 해로 초기화하는 것만으로 관절 점프나 속도·가속도 연속성을 보장하지 못한다. PlaCo도 초기 상태를 이어가는 사용법, 변위 regularization, 자세 task, 속도 제약을 별도로 제공한다. [PlaCo 기본 루프](https://placo.readthedocs.io/en/latest/kinematics/getting_started.html#running-the-solver), [regularization](https://placo.readthedocs.io/en/latest/kinematics/regularization.html), [속도 한계](https://placo.readthedocs.io/en/latest/kinematics/joint_limits.html)

현재 어댑터에는 **연속된 IK reference와 최종 명령의 시간 한계, chunk 교체 시 궤적 상태 유지**를 함께 검토하는 것이 적절하다. 이 권고는 아래 소스에서 도출한 설계 판단이며, 특정 실험에서 warm-start가 원인을 해결했다는 주장과는 다르다.

## 용어를 분리해야 하는 이유

| 방식 | 이전 값의 역할 | 해결하는 문제 | 남는 문제 |
|---|---|---|---|
| Seed / warm-start | 수치 반복 시작점 `q_seed` | 가까운 해에서 탐색 시작, 같은 해 계열을 유지할 가능성 | 최종 해의 거리·시간 제한이 아님 |
| 이전 자세 reference | `||q − q_ref||²` 같은 목적함수 | EE를 비슷하게 맞추는 여러 자세 중 이전 자세 선호 | soft cost이면 큰 이동도 비용을 지불하고 선택 가능 |
| 속도·가속도 제약 | 실행 간격과 이전 `q/v`로 허용 집합 정의 | 관절 이동률 제한 | Cartesian 오차·도달 시간과 절충 필요 |
| 연속 궤적 생성 | 현재 실행 상태 `(q,v,a)`를 다음 계획으로 전달 | retarget 시 위치·속도·가속도 연결 | IK branch, 충돌, Cartesian 경로 정확도는 별도 문제 |

표의 구분은 아래 PlaCo·Unitree·Pink·Ruckig API를 비교한 해석이다.

## PlaCo에서 이미 가능한 것

1. `robot.state.q`는 현재 선형화·적분 상태다. `solver.solve(True)`는 작은 `Δq`를 구해 그 상태에 적분한다. 공식 루프는 상태를 이어서 갱신한다. 따라서 매 샘플의 관측값으로 상태를 재설정하는 것과 이전 계획 상태를 이어가는 것은 서로 다른 seed 정책이다. [공식 설명, Running the solver](https://placo.readthedocs.io/en/latest/kinematics/getting_started.html#running-the-solver)
2. `add_regularization_task(weight)`는 **각 solve의 `||Δq||²`** 비용이다. 0.10.2 소스는 `W Δq = 0`, `W = sqrt(weight) I`를 구성한다. 전체 외부 호출의 최종 `q`와 고정된 `q_previous` 차이를 직접 벌주는 항과 동일하지 않다. 큰 weight는 추종 지연을 유발할 수 있다고 문서에 명시되어 있다. [0.10.2 RegularizationTask, 28–53행](https://github.com/Rhoban/placo/blob/a91e11506fe0909b38d8eff4f0e3d6febb3cc3d3/src/placo/kinematics/regularization_task.cpp#L28-L53), [Regularization 설명](https://placo.readthedocs.io/en/latest/kinematics/regularization.html#default-l2-regularization)
3. 이전 자세를 명시적으로 선호하려면 `solver.add_joints_task()`, `task.configure(name, "soft", weight)`, `task.set_joints({joint_name: q_ref})`라는 공개 API가 있다. 최적화 반복 동안 `q_ref`를 고정해야 “이전 외부 샘플에 가까움”이라는 의미를 유지한다. 반복마다 reference를 현재 iterate로 옮기면 다른 목적이 된다. API 자체와 단위(rad/m)는 공식 문서, reference 갱신 정책은 본 조사에서의 권고다. [Joints task](https://placo.readthedocs.io/en/latest/kinematics/joints_task.html)
4. `solver.enable_velocity_limits(True)`와 `solver.dt`를 함께 설정한다. 속도 한계는 URDF에서 로드되며 기본적으로 꺼져 있다. 문서는 두 연속 solve 사이의 실제 시간 간격을 `dt`로 정의한다. [Joint and velocity limits](https://placo.readthedocs.io/en/latest/kinematics/joint_limits.html)

특히 **한 외부 샘플을 풀기 위해 수행하는 45번의 수치 반복에 각각 전체 실행 간격을 주면 안 된다.** 0.10.2 구현은 solve마다 `−dt*vmax ≤ Δq ≤ dt*vmax`를 적용하고 `solve(True)`마다 적분한다. 따라서 반복당 한계만으로 최종 외부 명령의 변위를 제한했다고 볼 수 없다. 여러 번 적분한 결과에는 별도의 누적 예산 또는 고정 시작점 기준 최종 제약이 필요하다. 이것은 소스 식으로부터의 직접적인 추론이다. [0.10.2 속도 bound, 290–322행](https://github.com/Rhoban/placo/blob/a91e11506fe0909b38d8eff4f0e3d6febb3cc3d3/src/placo/kinematics/kinematics_solver.cpp#L290-L322), [적분과 qd/qdd 갱신, 431–442행](https://github.com/Rhoban/placo/blob/a91e11506fe0909b38d8eff4f0e3d6febb3cc3d3/src/placo/kinematics/kinematics_solver.cpp#L431-L442)

고정 reference의 의미도 0.10.2 `JointsTask::fill`에서 확인된다. `b = target − current_joint`, 해당 관절 column의 `A = 1`이므로 이전 자세를 target으로 두는 것과 zero-motion regularization은 명시적으로 다르다. [0.10.2 JointsTask, 37–50행](https://github.com/Rhoban/placo/blob/a91e11506fe0909b38d8eff4f0e3d6febb3cc3d3/src/placo/kinematics/joints_task.cpp#L37-L50)

현재 `.015**2` regularization과 반복당 `.15 rad` clamp는 위의 이유로 외부 명령의 물리적 속도 계약이 아니다. 숫자가 작다는 사실만으로 최종 관절 이동량이 작다고 결론낼 수 없다.

## 실제 다른 시스템의 패턴

### Unitree 공식 XR teleoperation: seed + reference cost + 출력 필터

Unitree 공식 저장소 `G1_29_ArmIK`는 세 가지를 함께 사용한다. `set_initial(var_q, init_data)`로 시작점을 주고, 별도로 `sumsqr(var_q - var_q_last)`를 목적함수에 포함하고, 해에 `WeightedMovingFilter`를 적용한다. `current_lr_arm_motor_q`가 전달되면 `init_data`를 그 관측값으로 덮어쓴 뒤 seed와 reference 양쪽에 사용한다. 없으면 저장한 직전 해를 이어 쓴다. **따라서 “공식 구현은 무조건 직전 명령만을 seed로 쓴다”는 해석은 틀리다.** [고정 commit의 cost, 130–167행](https://github.com/unitreerobotics/xr_teleoperate/blob/817fb00c63cde15e5f24a0f8fa08e1e33ed89d3b/teleop/robot_control/robot_arm_ik.py#L130-L167), [seed/reference/filter, 233–260행](https://github.com/unitreerobotics/xr_teleoperate/blob/817fb00c63cde15e5f24a0f8fa08e1e33ed89d3b/teleop/robot_control/robot_arm_ik.py#L233-L260)

필터는 최근 해들의 가중 평균이다. 이 필터 자체에는 `dt`, 속도, 가속도, jerk 한계가 없다. 따라서 이를 물리적 한계 보장으로 해석하면 안 된다. [동일 commit 필터 구현, 14–35행](https://github.com/unitreerobotics/xr_teleoperate/blob/817fb00c63cde15e5f24a0f8fa08e1e33ed89d3b/teleop/utils/weighted_moving_filter.py#L14-L35)

### Pink: 위치 상태를 이어 적분하되 가속도는 별도 제한

공식 UR3 예제는 `velocity = solve_ik(configuration, tasks, dt, ...)` 후 `configuration.integrate_inplace(velocity, dt)`를 수행한다. 이전 IK 상태를 이어서 **속도**를 계산하는 패턴이다. [공식 예제](https://github.com/pink-kinematics/pink/blob/main/examples/arm_ur3.py)

유지관리자는 2024-07-23 답변에서 1차 differential IK만으로 속도 연속성이 보장되지 않는다고 설명했고, 2024-07-29 후속 답변에서는 이전 속도를 사용하는 `AccelerationLimit` 구현을 보고했다. 초기 답변의 “아직 미구현”만 인용하면 이후 내용을 놓친다. 관절 한계에 접근하며 정지할 수 있도록 하는 추가 bound도 설명한다. [공식 유지관리자 discussion #103, 7월 23일과 29일 답변](https://github.com/orgs/pink-kinematics/discussions/103)

### MoveIt Humble: 연속 IK와 jump 검출, timing을 별도로 취급

Humble의 Cartesian interpolation 구현은 상태를 연속 갱신하며 IK를 수행하고, random seed 재시도가 jump를 만들 수 있다는 이유로 한 번의 시도만 사용한다. 절대 consistency limit 및 이후 jump 검사를 사용하며 jump가 발생하면 경로를 잘라낸다. jump를 자동으로 올바른 경로로 바꾸는 기능은 아니다. [Humble 소스, 원본 127–174행 및 jump 검사](https://moveit.picknik.ai/humble/api/html/cartesian__interpolator_8cpp_source.html), [Humble API 설명](https://moveit.picknik.ai/humble/api/html/classmoveit_1_1core_1_1CartesianInterpolator.html)

기하학적 경로를 만든 뒤 속도·가속도를 부여하는 time parameterization과 jerk를 제한하는 Ruckig 후처리도 구분한다. [MoveIt Humble Time Parameterization](https://moveit.picknik.ai/humble/doc/examples/time_parameterization/time_parameterization_tutorial.html)

### Ruckig: `(q,v,a)`를 이어서 새 목표로 retarget

Ruckig는 현재·목표 위치/속도/가속도와 속도/가속도/jerk 한계를 입력받아 상태 간 궤적을 생성한다. `output.pass_to_input(input)`으로 출력 상태를 다음 입력에 이어 준다. 새 입력이 예정 상태와 다르면 새 궤적을 계산한다. [공식 README](https://github.com/pantor/ruckig/blob/main/README.md), [원 논문](https://arxiv.org/abs/2105.04830)

`minimum_duration`은 정확한 도착 시간 보장이 아니다. 속도와 가속도 기본값을 매 chunk마다 0으로 초기화하면 진행 중인 궤적 상태를 잇는 설계가 되지 않는다. 도착 가능한 시간이 `.1 s`보다 길면 한계를 유지하면서 `.1 s`에 그 목표에 정확히 도달한다는 두 요구를 동시에 만족할 수 없다. 단순히 계속 바뀌는 목표점을 넣으면 지연될 수 있고, 공식 tracking interface는 Pro 기능이다. [공식 tutorial, Input Parameter / Tracking Interface](https://docs.ruckig.com/tutorial.html)

**Ruckig는 EE IK 해 선택기나 충돌 회피기가 아니다.** 관절공간에 적용할 경우 EE 경로 정확도와 충돌을 별도로 검증해야 한다는 것이 입력 계약에서 도출되는 한계다. 긴 관절 이동을 천천히 실행하도록 바꾸는 것과, 그 이동이 처음부터 불필요하도록 IK 해를 고르는 것은 다른 문제다.

Community의 로컬 point-to-point 상태 간 OTG와 multi-waypoint 기능은 구분해야 한다. 공식 0.19.7 tutorial은 `intermediate_positions`를 하나라도 넣으면 Community가 실시간성이 없는 cloud API로 전환한다고 명시한다. 로컬 multi-waypoint 계산은 Pro 기능이다. 따라서 전체 action chunk를 Community의 로컬 waypoint 배열로 넣으면 된다는 제안은 하지 않는다. [공식 tutorial, Intermediate Waypoints](https://docs.ruckig.com/tutorial.html)

## 현재 어댑터에 대한 권고

다음은 조사 결과에 따른 설계 제안이며 구현 완료나 검증된 수정사항이 아니다.

1. **정확한 입력을 먼저 보존한다.** target pose, full measured q, 실제 사용한 seed, 고정 reference, 이전 accepted command, 시각, chunk ID/선택 index, iteration별 누적 변위, 원시 IK 해와 최종 명령을 함께 기록한다. 저장 run의 동일 seed/chunk에서 shoulder yaw 약 36° 이동과 EE 오차 약 6.4 mm/2.1°가 관찰되었다는 사실만으로 실제 사건의 원인을 확정하지 않는다. 정확한 사건 입력이 빠져 있고 마지막 chunk warm-start 비교 효과도 작았다.
2. **시간 의미를 먼저 고정한다.** native 30 Hz/H=30 target과 10 Hz dispatch/0.1 s 명령 사이에 timestamp 기반 선택·보간 규칙을 둔다. chunk 교체 시 현재 실행 중인 관절 궤적 상태에서 다음 목표를 연결한다. 추론 완료 시점과 chunk 생성·관측 시점을 분리한다. 이는 이 시스템의 입력 조건으로부터 도출한 권고다.
3. **IK seed와 reference를 분리해서 비교한다.** 같은 exact input에서 (a) measured seed, (b) previous accepted solution seed, (c) fixed previous-reference JointsTask, (d) 두 방법 결합을 비교한다. 측정 full q와 고정 lower-body/waist를 반영하되 arm planning state와의 차이를 관찰한다. tracking error가 큰 상태에서 과거 command만 믿는 정책도 자동으로 정답은 아니다.
4. **최종 명령 단계에 실제 시간 한계를 둔다.** 허용된 관절 이동·속도·가속도·jerk는 로봇/제어기에서 확인한 값으로 정의한다. iteration clamp나 soft cost로 대체하지 않는다. 한계 안에서 EE 오차를 받아들일지, 시간이 더 필요할지, 해당 목표를 거절할지를 명시한다. 단순 clip 후에는 FK 오차를 다시 평가한다.
5. **연속 IK 경로와 실행 궤적을 함께 평가한다.** chunk 안의 인접 EE target을 이전 해부터 이어 풀고 chunk 경계도 동일하게 잇는다. 한 샘플 최종 q만 평가하지 말고 max joint delta, v/a/jerk, EE residual, lag, 도달률을 함께 본다. 필요하면 Ruckig 같은 상태 기반 실행층을 검토하되, 불필요한 자세 이동을 해결하는 reference/제약을 먼저 분리한다.

안전한 수치 threshold, 고주파 제어 주기, 허용 EE 오차는 이 문서에서 임의로 정하지 않는다. 현재 문헌 조사만으로 하드웨어별 한계나 최적 weight를 확정할 수 없다.

## 출처 버전과 재현성 한계

- 로컬 프로젝트의 [ik-performance.md](../ik-performance.md)는 Adapter 0.3.0 / PlaCo 0.10.2 / Pinocchio 3.8.0과 현재 설정을 명시한다. 본 조사에서 설치본을 실행하거나 교체하지 않았다.
- MoveIt는 **Humble** 문서·소스에 범위를 고정했다.
- PlaCo `v0.10.2`의 annotated tag object는 `b060e1cef8019f41ed0e553974489c410b8cc382`, peeled commit은 `a91e11506fe0909b38d8eff4f0e3d6febb3cc3d3`다. `git ls-remote`로 확인한 ref를 사용해 고정 raw 소스의 regularization, JointsTask, 속도 bound와 적분을 직접 확인했다. 이는 공식 0.10.2 소스 검증이며 설치 바이너리의 build hash 일치까지 검증한 것은 아니다.
- Unitree `main`은 조사 시 `817fb00c63cde15e5f24a0f8fa08e1e33ed89d3b`로 고정하고 해당 raw IK·필터 소스를 확인했다.
- PlaCo `latest` 문서, Pink/Ruckig `main` 문서는 가변 출처다. Ruckig tutorial은 읽은 시점에 **0.19.7**을 표시했다. Pink 유지관리자 답변은 날짜를 명시했다. 이후 구현에서는 사용 버전의 API를 다시 확인해야 한다.
