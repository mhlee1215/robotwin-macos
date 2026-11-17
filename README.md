# RoboTwin on Apple Silicon

RoboTwin 2.0의 시뮬레이션 환경을 Apple Silicon Mac에서 실행하는 독립 포트입니다. 물리는 CPU PhysX, RGB-D 렌더링은 MoltenVK/Metal, 계산량이 큰 계획기 연산은 native Metal, pointcloud 처리는 PyTorch MPS를 사용합니다.

**50개 태스크 환경의 로드·렌더링·양팔/그리퍼 물리 구동을 확인했습니다.** Seed 0에서 49개가 통과했고, 초기 안정화 검사에 거부된 `pick_diverse_bottles`는 seed 1에서 통과했습니다. 이는 전문가 스크립트가 50개 작업을 모두 성공했다는 의미는 아닙니다.

![Mac에서 렌더링한 양팔 로봇과 randomized 환경](artifacts/simulator_canary_physical_v3/overview_final.png)

## 검증 환경과 범위

- Apple M5 Pro / macOS 26.6.2 / Python 3.11.15 / SAPIEN 3.0.3
- 기본 ALOHA-agilex embodiment, offscreen RGB-D 카메라 4개
- 원본 RoboTwin commit: [`ea8b21121ebb3cd201ff5b3fe361944ac94eda3f`](https://github.com/RoboTwin-Platform/RoboTwin/tree/ea8b21121ebb3cd201ff5b3fe361944ac94eda3f)
- 실제 articulation `qpos/qvel`을 측정한 직접 관절 제어 검사: IK 호출 0회
- 관절 목표 오차 최대 0.00089 rad, 복귀 오차 최대 0.00068 rad
- 대표 randomized 환경의 pointcloud, 외부 카메라, 종료 후 재초기화 확인

다른 Mac, 모든 seed, 다른 embodiment, 인터랙티브 GUI, VLA/VLM 추론은 검증하지 않았습니다. 이 저장소의 패치가 필요한 구성으로, 수정하지 않은 upstream의 공식 macOS 지원을 뜻하지 않습니다.

## 설치

Apple Silicon용 Homebrew와 Xcode Command Line Tools가 필요합니다. 명령은 저장소 루트에서 실행합니다. 공식 에셋 ZIP 다운로드는 합계 약 15 GB이며, 압축 파일·해제한 에셋·Python 환경을 함께 보관하므로 여유 공간을 최소 40 GB 정도 확보하세요.

```sh
gh repo clone mhlee1215/robotwin-macos
cd robotwin-macos

# 아직 설치하지 않았다면 실행
xcode-select --install
brew install uv molten-vk vulkan-loader

uv venv --python 3.11 .venv
.venv/bin/python bootstrap_macos.py
```

`bootstrap_macos.py`는 고정된 Python 패키지 설치, 원본 RoboTwin checkout, macOS 패치 적용, 공식 에셋 다운로드 및 SHA-256 검증, Metal compute 실행 검사를 수행합니다. Homebrew 기본 경로 `/opt/homebrew`를 사용합니다. 에셋과 가상환경은 Git에 포함하지 않습니다.

## 실행

```sh
# 시뮬레이터: 실제 관절/그리퍼 구동, RGB-D, pointcloud, 재초기화
.venv/bin/python run_task.py --mode simulator --reset-check --pointcloud --overview --output artifacts/my_simulator_check

# 배경을 무작위화한 환경
.venv/bin/python run_task.py --mode simulator --randomized --overview --pointcloud --output artifacts/my_randomized_check

# 50개 환경 검사: 각 태스크를 별도 프로세스로 실행
.venv/bin/python validate_suite.py --mode simulator --name my_simulator_suite

# seed 0에서 초기 안정화가 거부된 태스크의 명시적 seed 1 검사
.venv/bin/python run_task.py --mode simulator --task pick_diverse_bottles --seed 1 --reset-check --output artifacts/my_bottle_seed1

# 독립적인 중력·접촉·모터·렌더링 검사
sh mac_python.sh "$PWD/probe_sapien.py" --render --output artifacts/my_physics_check
```

`result.json`에 판정과 수치가, 출력 폴더에 관찰 이미지가 저장됩니다. `--overview`는 양팔 전체를 보여주는 1600×1200 이미지를 추가합니다. 전체 suite는 초기 안정화 거부를 포함해 하나라도 실패하면 종료 코드 1을 반환합니다. 대체 seed 검사는 별도 기록으로 남기며 원래 실패를 덮어쓰지 않습니다. 매번 새 출력 이름을 사용하는 것을 권장합니다.

## 계획기와 GPU 가속

기본 backend는 Metal입니다. CPU 비교 구현은 `--backend cpu`로 선택합니다. CUDA cuRobo를 Metal로 변환한 것이 아니라, RoboTwin 인터페이스에 연결한 별도 FK/IK·충돌·궤적 계산기입니다. CPU는 물리와 Python 실행 제어, RRT 그래프 관리, I/O를 담당합니다.

```sh
.venv/bin/python verify_metal.py --output artifacts/my_metal_verification
.venv/bin/python -m unittest discover -s tests -v
.venv/bin/python run_task.py --mode expert --task handover_block --overview --output artifacts/my_expert_run
```

전문가 스크립트의 seed 0 검사에서는 50개 중 27개 성공, 21개 계획 실패, 1개 작업 실패, 1개 초기 안정화 거부를 기록했습니다. `stamp_seal`은 CPU 구현에서 성공하지만 Metal 구현에서는 실패하는 미해결 회귀가 있습니다. 시뮬레이터 실행 가능성과 계획기 성공률은 구분해야 하며, 원본 CUDA 벤치마크와 동등한 성공률을 주장하지 않습니다.

구현과 수치 검증은 [METAL_BACKEND.md](METAL_BACKEND.md), 시뮬레이션 검증 절차는 [SIMULATOR_VERIFICATION.md](SIMULATOR_VERIFICATION.md)를 참고하세요.

## 포함된 검증 기록

| 기록 | 내용 |
|---|---|
| [종합 판정](artifacts/simulator_verification_final.json) | 50개 환경, seed 및 검증 범위 |
| [전체 seed 0 검사](artifacts/simulator_all50_final/summary.json) | 성공 49개와 거부 1개의 실제 기록 |
| [병 태스크 seed 1](artifacts/simulator_diverse_seed1_final/result.json) | 별도 재검사와 재초기화 |
| [randomized 검사](artifacts/simulator_canary_physical_v3/result.json) | 물리 관절 상태, pointcloud, 카메라 |
| [독립 물리 검사](artifacts/simulator_physics_final/sapien_render.json) | 낙하·접촉·모터·RGB-D |
| [Metal 수치 검사](artifacts/metal_numerics_v2/result.json) | FK/IK·충돌·궤적·FPS 비교 |
| [전문가 실행 검사](artifacts/metal_expert_audit.md) | 태스크별 성공/실패와 알려진 한계 |

기존 검증에서 사용한 실행 코드·패치의 SHA-256은 [SOURCE_MANIFEST.json](SOURCE_MANIFEST.json)에 있습니다. 선택한 기록과 대표 이미지만 포함하며, JSON의 로컬 경로 접두어는 익명화했습니다. 새 실행 결과와 대용량 데이터는 `.gitignore`로 제외합니다.

## 출처

원본 프로젝트: [RoboTwin-Platform/RoboTwin](https://github.com/RoboTwin-Platform/RoboTwin). 이 저장소는 독립적인 macOS 포트이며 공식 배포판이 아닙니다. 코드의 MIT 고지와 의존성·에셋의 별도 라이선스는 [LICENSE](LICENSE), [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)에 정리했습니다.
