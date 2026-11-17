# Mac GPU backend

RoboTwin의 물리는 CPU PhysX에 두고, 반복되는 수치 계산은 Apple GPU에서 수행한다.
원본 cuRobo를 Metal에서 실행하는 구현은 아니다. 기존 Mac CPU 계획기의 인터페이스에
직접 작성한 Metal compute kernel을 연결했다. 검증 embodiment는 ALOHA-agilex의 6축 팔이다.

| 처리 | 장치 / 구현 |
|---|---|
| 접촉, 중력, 관절 드라이브, 물리 상태 갱신 | CPU / SAPIEN PhysX |
| 카메라 RGB-D 렌더링 | Apple GPU / Vulkan → MoltenVK → Metal |
| FK, IK 후보 계산 및 6×6 선형방정식 | Apple GPU / native Metal |
| 로봇 collision sphere와 테이블·벽 충돌 검사 | Apple GPU / native Metal |
| 속도·가속도 제한 quintic 궤적 평가 | Apple GPU / native Metal |
| Pointcloud 좌표 변환·필터링 | Apple GPU / PyTorch MPS |
| Farthest-point sampling | Apple GPU / native Metal |
| URDF 읽기, RRT 그래프 제어, 작은 좌표/명령 준비, 시뮬레이터 호출, 파일 저장 | CPU host orchestration |
| VLA/VLM 추론 | 이번 포트에는 모델 미연결. 모델 추론 가속을 검증했다고 주장하지 않음 |

“물리만 CPU”는 계산량이 큰 처리에 대한 구성 목표다. Python 실행 제어와 I/O,
RRT 노드 선택 및 작은 행렬 준비까지 GPU로 옮긴 것은 아니다.
카메라 이미지는 SAPIEN에서 host array로 읽은 뒤 MPS로 전송하므로 zero-copy는 아니다.

기본 실행은 Metal이다. 명시적 CPU 선택은 비교·진단용으로만 남겨 두었다.

## 실행과 검증

```sh
.venv/bin/python run_task.py --backend metal --mode planner
.venv/bin/python run_task.py --backend metal --mode ee
.venv/bin/python run_task.py --backend metal --mode qpos --pointcloud
.venv/bin/python run_task.py --backend metal --task handover_block --mode expert --randomized --overview --pointcloud --output artifacts/my_metal_rollout
.venv/bin/python verify_metal.py --output artifacts/my_metal_verification
ROBOTWIN_MAC_PLANNER=metal .venv/bin/python validate_suite.py --mode expert --name my_metal_suite
```

명시적 CPU 비교는 `--backend cpu` 또는 `ROBOTWIN_MAC_PLANNER=cpu`로 실행한다.
Metal 계획기는 GPU가 없거나 지원하지 않는 embodiment이면 오류를 내며 CPU 수치 솔버로
자동 전환하지 않는다. Metal 실행기의 PyTorch unsupported-op CPU fallback도 비활성화한다.
`result.json`의 `acceleration.device`, 커널 호출 횟수, 처리 건수, GPU 실행 시간과
`pointcloud_device`로 실제 사용 경로를 확인한다. 커널 시간은 episode 총시간이 아니다.

최초 실행은 Xcode Command Line Tools의 clang++로 작은 Objective-C++ 브리지를 빌드하고,
Metal 드라이버에서 shader를 컴파일한다. 새 외부 패키지는 필요 없다.
소스 hash가 달라지면 브리지 캐시를 다시 빌드한다. `bootstrap_macos.py`는 설치 끝에
실제 Metal FPS 계산을 검증한다.

## 수치 검증과 측정

`artifacts/metal_numerics_v2/result.json` — Apple M5 Pro:

- 양팔 각각 512개 상태, 합계 1,024개에서 CPU와 충돌 판정 불일치 0개.
- FK 행렬 최대 오차 약 1.6e-6.
- 도달 가능한 IK 목표 16개에서 전부 성공. CPU FK로 독립 측정한 최대 위치 오차 0.14 mm 미만.
- 정지·미세·일반·다중 구간 궤적의 유한 값, 끝점, 1 rad/s 속도·2 rad/s² 가속도 제한 확인.
- 4,096개 점에서 128개 FPS 결과가 CPU reference와 동일.
- MPS tensor 실제 실행, CPU fallback 비활성 확인.

동일 프로세스에서 GPU pipeline warmup 후 측정한 예시다. 전체 episode 배속이나
일반적인 모든 목표의 성능을 나타내지 않는다. 단위는 ms이며 host 제출/반환을 포함한다.

| 연산 | CPU | Metal | 대략적인 배속 |
|---|---:|---:|---:|
| FK 512개 | 6.3–6.6 | 0.57–0.60 | 11× |
| 충돌 검사 512개 | 92–94 | 0.92–1.19 | 78–102× |
| 쉬운 IK 목표, 각 팔 8개 중앙값 | 6.2–6.6 | 1.9–2.1 | 3.2× |

IK는 먼저 현재 자세로 GPU에서 풀이하고, 실패하면 64개 후보를 GPU에서 병렬 탐색한다.
모든 경우가 같은 속도 향상을 보장하는 것은 아니다.

## 범위와 남는 한계

- CPU판과 같은 공식 collision spheres/ignore/locked gripper 모델을 사용한다.
  task object 접촉은 물리 엔진이 처리한다. 물리적 성공을 단순 충돌 판정으로 대신하지 않는다.
- CPU TOPPRA 대신 제한을 만족하는 quintic 구간을 사용한다. 구간 경계에서 정지하므로
  cuRobo의 시간 최적화 궤적과 같지 않다. 모든 최종 명령 샘플의 충돌을 검사한다.
- RRT 탐색 제어 및 Cartesian 목표의 작은 보간 준비는 host에서 수행한다.
- 원본 task success checker와 불안정 seed 거부 기준은 유지했다.
- GPU 가속은 실패한 작업을 자동으로 성공시키는 기능이 아니다. 실행 오류, 경로 계획 실패,
  물리 동작 후 작업 실패, 초기 상태 불안정을 별도로 기록한다.
- 원본 CUDA 벤치마크와 동등한 성능/성공률이나 다른 로봇 지원은 검증하지 않았다.


## 태스크 실행 결과와 회귀

동일 버전의 50개 task, seed 0 각 1회 검사: 27개 성공, 21개 계획 실패,
1개 작업 실패, 1개 불안정 seed 거부, 실행 코드 예외/timeout 0개.
상세 결과는 `artifacts/metal_expert_audit.md`에 있다.
`adjust_bottle`, `place_container_plate`는 이전 실패 기록에서 성공으로 바뀌었다.
반대로 `stamp_seal`은 CPU에서 재현 성공하나 Metal에서는 실패한다.
실패 상태에서 CPU IK 64회와 GPU 2,048개 후보도 유효 해를 찾지 못했다.
앞선 이동·잡기 상태 차이의 영향으로 추정되나 정확한 원인은 아직 확정하지 못했다.
가속 전환이 모든 작업의 성공을 보장하지 않으며 이 회귀는 해결되지 않았다.

원래 거부된 `pick_diverse_bottles` seed 0은 계속 실패로 기록했다.
별도의 seed 1에서는 Metal 전문가 작업 성공을 확인했으며 seed 0 집계와 섞지 않았다.
