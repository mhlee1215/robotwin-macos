# RoboTwin macOS: 시뮬레이션 환경 검증

이 문서의 검증 대상은 환경의 실행 가능성이다. 전문가 스크립트의 작업 성공률,
IK 탐색 성능, VLA 성능을 합격 기준으로 사용하지 않는다.
검증 환경은 Apple M5 Pro / macOS 26.6.2 / Python 3.11.15 / SAPIEN 3.0.3이며,
별도 폴더의 macOS 포트를 사용한다. 수정하지 않은 upstream의 macOS 지원을 뜻하지 않는다.

## 최종 결과

**이 Mac에서 50개 RoboTwin 태스크 환경의 실행을 확인했다.**
seed 0은 49/50 통과했다. `pick_diverse_bottles` seed 0은 초기 물체 안정화 검사에서
거부됐고, seed 1을 명시해 재검사한 결과 통과했다. 거부된 seed 0 기록은 보존했다.
이는 모든 seed가 유효하다는 주장이나 전문가 작업 성공률이 아니다.

- 50개 환경에서 4개 RGB-D 카메라, 실제 양팔 이동·복귀, 실제 그리퍼 닫기·열기 통과.
- 동작 후 실제 관절 오차 최대 약 0.00089 rad, 복귀 오차 최대 약 0.00068 rad.
- 전수 검사에서 IK 호출 0회. 목표 관절 명령을 물리 엔진으로 실행했다.
- 대표 randomized 환경의 MPS pointcloud와 1600×1200 외부 카메라 확인.
- 대표 randomized 환경과 병 태스크 seed 1에서 종료 후 동일 인스턴스 재초기화 통과.
- 별도 물리 검사에서 자유낙하·바닥 접촉·모터 구동·렌더링 통과.
- 물리 검사 이미지와 randomized 로봇 전체 이미지를 직접 확인했다.

종합 기록: `artifacts/simulator_verification_final.json`.
전수 검사 원본: `artifacts/simulator_all50_final/summary.json`.
seed 1 재검사: `artifacts/simulator_diverse_seed1_final/result.json`.

## 검사 내용

`run_task.py --mode simulator`는 다음을 실행한다.

1. 원본 태스크·에셋·기본 ALOHA-agilex 로봇을 로드한다.
2. 4개 카메라의 RGB·depth가 비어 있지 않고 유효한지 검사한다.
3. 양팔에 각각 +0.06 / -0.06 rad 관절 명령을 보내고 그리퍼를 닫는다.
4. 구동 목표를 유지하며 물리를 1초 진행한 뒤, 실제 articulation `qpos/qvel`을 읽는다.
   drive target이나 캐시된 gripper 명령값을 물리 측정값으로 사용하지 않는다.
5. 두 팔이 각각 0.02 rad 이상 움직였고 목표 오차가 0.02 rad 미만인지,
   실제 gripper 위치가 유의하게 변했는지 검사한다.
6. 복귀·그리퍼 열기 명령 후 같은 방식으로 실제 복귀를 검사한다.
7. 관찰·성공 판정 API를 호출하고 환경을 종료한다. 이 과정의 IK 호출은 0회여야 한다.

관절 명령의 시간 보간은 기존 action API가 사용하지만, 목표 그리퍼 자세를
검색하거나 전문가 작업 순서를 생성하는 경로 계획은 호출하지 않는다.

대표 randomized 환경에서는 추가로 MPS pointcloud, 1600×1200 외부 카메라,
`close_env()` 후 동일 인스턴스의 `setup_demo()` 재초기화도 검사한다.
별도 SAPIEN 검사에서는 물체 낙하·바닥 접촉·모터 구동·RGB-D 렌더링을 확인한다.

## 실행

이 폴더에서 실행한다.

```sh
# 단일 태스크: 직접 관절 제어, 실제 물리 상태 확인, 관찰, 재초기화
.venv/bin/python run_task.py --mode simulator --reset-check --pointcloud --overview --output artifacts/my_simulator_check

# 50개 태스크 전체
.venv/bin/python validate_suite.py --mode simulator --name my_simulator_suite

# 초기 안정화가 거부된 경우 seed를 명시해 별도 검사
.venv/bin/python run_task.py --mode simulator --task pick_diverse_bottles --seed 1 --reset-check --output artifacts/my_bottle_seed1
```

## 범위

기본 ALOHA embodiment와 offscreen 이미지 기반 실행을 검증한다.
모든 seed·모든 embodiment·인터랙티브 GUI·원본 CUDA 벤치마크와의 동등성은 이 검사로
보장하지 않는다. 별도 전문가 평가의 성공/실패 기록은 시뮬레이터 실행 판정에 합산하지 않는다.
