# UniFoLM → 실물 G1 Adapter

WLA에서 기존 UniFoLM W8A8 모델과 Adapter를 실행하고, G1 Orin NX의 PAOS에 유선으로 연결합니다. 실물 준비와 실행 순서는 **[실물 G1 Quickstart](docs/real-g1-quickstart.md)**를 따릅니다.

| 실행 위치 | 구성 |
|---|---|
| WLA `/home/wla/paos-model-adapter` | 모델 서버·Adapter |
| G1 Orin NX | PAOS·WBC·HAL·카메라·Dex1 |
| 모델 checkpoint | `/home/wla/WMA-JLee/models/C-W8A8-20261028` |
| G1 Facade | `192.168.123.164:15051` |
| WLA 내부 모델 주소 | `ws://127.0.0.1:18610` |

HTTPS clone·pull에는 GitHub 토큰이 필요하지 않습니다. 경량 SDK `paos-remote==0.2.1`은 별도 wheel로 설치합니다. VGA 관측에는 G1 Facade와 WLA SDK 양쪽에 gRPC 메시지 한도 8 MiB 수정이 필요합니다. 로봇별 설정은 Git에서 제외한 `private/g1.yaml`에 둡니다.

새 설치용 기본 템플릿은 **base 속도 입력**입니다. 현재 WLA의 기존 `private/g1.yaml`에는 arm14·그리퍼 적용이 활성화되어 있으므로 갱신할 때 이 파일을 유지합니다. 현재 lower_body/pivot/height까지 연결한 whole-body 재현은 완료되지 않았습니다. 모델 서버는 여러 프롬프트 동안 유지하고 Adapter의 Ctrl+C로 자기 목표 취소·입력권 반납을 처리합니다.

Adapter 0.3.3은 Head·Left Wrist·Right Wrist의 **640×480 raw RGB8** 관측을 받으며, 기존 320×240 QVGA도 지원합니다. 기본 전송은 `rgb8`이고 JPEG는 명시적으로 선택하는 옵션입니다. Adapter는 크기를 바꾸지 않고 공식 서버 입력의 BGR 채널 순서로 변환합니다. 기존 W8A8 모델 서버는 추론 전 **448×320(너비×높이)**으로 resize하므로, VGA는 관측·전송 해상도입니다. checkpoint·전처리·compile 정책은 그대로입니다. 세 VGA 영상의 raw 데이터는 2,764,800바이트이며, `pick carrot` 지시문·상태·MessagePack 메타데이터를 포함한 요청은 2,765,640바이트로 기존 4 MiB WebSocket 한도에 들어갑니다.

최신 PAOS 실물 기본 Home은 참고 사진처럼 팔꿈치를 굽히고 전완을 앞으로 둔 nominal 자세입니다. 실제 관절 목표와 준비 순서는 PAOS의 실물 실행 가이드를 따릅니다.

[IK 성능 비교](docs/ik-performance.md) · [구현 배경과 이전 SIM 실험](docs/implementation-notes.md) · [외부 소스·라이선스](THIRD_PARTY.md)
