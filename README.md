# UniFoLM → 실물 G1 Adapter

WLA에서 기존 UniFoLM W8A8 모델과 Adapter를 실행하고, G1 Orin NX의 PAOS에 유선으로 연결합니다. 실물 준비와 실행 순서는 **[실물 G1 Quickstart](docs/real-g1-quickstart.md)**를 따릅니다.

| 실행 위치 | 구성 |
|---|---|
| WLA `/home/wla/paos-model-adapter` | 모델 서버·Adapter |
| G1 Orin NX | PAOS·WBC·HAL·카메라·Dex1 |
| 모델 checkpoint | `/home/wla/WMA-JLee/models/C-W8A8-20261028` |
| G1 Facade | `192.168.123.164:15051` |
| WLA 내부 모델 주소 | `ws://127.0.0.1:18610` |

HTTPS clone·pull에는 GitHub 토큰이 필요하지 않습니다. 경량 SDK `paos-remote`는 별도 wheel로 설치합니다. 로봇별 설정은 Git에서 제외한 `private/g1.yaml`에 둡니다.

기본 실물 설정은 **base 속도 입력**입니다. 팔·그리퍼 설정은 별도로 선택하며, 현재 lower_body/pivot/height까지 연결한 whole-body 재현은 완료되지 않았습니다. 모델 서버는 여러 프롬프트 동안 유지하고 Adapter의 Ctrl+C로 자기 목표 취소·입력권 반납을 처리합니다.

[IK 성능 비교](docs/ik-performance.md) · [구현 배경과 이전 SIM 실험](docs/implementation-notes.md) · [외부 소스·라이선스](THIRD_PARTY.md)
