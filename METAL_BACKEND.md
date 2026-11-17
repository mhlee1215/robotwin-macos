# Metal backend

The macOS backend keeps RoboTwin physics on CPU PhysX and runs repeated numerical operations on the Apple GPU. It connects native Metal kernels to the RoboTwin planner interface; it does not run CUDA cuRobo on Metal. The verified embodiment is ALOHA-agilex with six joints per arm.

| Operation | Implementation |
|---|---|
| Contact, gravity, joint drives, and physics state | CPU / SAPIEN PhysX |
| RGB-D rendering | Apple GPU / Vulkan → MoltenVK → Metal |
| FK, IK candidates, and 6×6 linear solves | Apple GPU / native Metal |
| Robot collision spheres against robot, table, and wall | Apple GPU / native Metal |
| Velocity- and acceleration-limited quintic trajectories | Apple GPU / native Metal |
| Pointcloud transforms and filtering | Apple GPU / PyTorch MPS |
| Farthest-point sampling | Apple GPU / native Metal |
| URDF parsing, RRT graph control, small coordinate preparations, simulator calls, and file I/O | CPU host code |
| VLA/VLM inference | No model integrated or inference acceleration verified |

Images are read from SAPIEN into host arrays before transfer to MPS; this path is not zero-copy. Python control flow, RRT node selection, and small matrix preparations also remain on the CPU.

Metal is the default backend. The explicit CPU backend remains available for reference and diagnosis.

## Execution

```sh
.venv/bin/python run_task.py --backend metal --mode planner
.venv/bin/python run_task.py --backend metal --mode ee
.venv/bin/python run_task.py --backend metal --mode qpos --pointcloud
.venv/bin/python run_task.py --backend metal --task handover_block --mode expert --randomized --overview --pointcloud --output artifacts/my_metal_rollout
.venv/bin/python verify_metal.py --output artifacts/my_metal_verification
ROBOTWIN_MAC_PLANNER=metal .venv/bin/python validate_suite.py --mode expert --name my_metal_suite
.venv/bin/python -m unittest discover -s tests -v
```

Select the CPU reference with `--backend cpu` or `ROBOTWIN_MAC_PLANNER=cpu`. The Metal planner raises an error for an unavailable GPU or unsupported embodiment; it does not automatically switch to a CPU numerical solver. PyTorch unsupported-operation CPU fallback is also disabled in Metal runs.

Inspect `acceleration.device`, kernel counts, GPU timing, and `pointcloud_device` in `result.json` to check execution paths. Kernel time is not total episode time.

The first run builds a small Objective-C++ bridge using Xcode Command Line Tools and compiles shaders through the Metal driver. The bridge cache is keyed by its source hash. The bootstrap script verifies an actual Metal farthest-point-sampling computation.

## Numerical verification

[Recorded results](artifacts/metal_numerics_v2/result.json) on Apple M5 Pro:

- Zero CPU/GPU collision-classification mismatches across 512 states per arm, or 1,024 states total.
- Maximum FK matrix error of approximately 1.6e-6.
- All 16 reachable IK targets solved, with maximum position error below 0.14 mm as independently measured by CPU FK.
- Finite trajectory values, endpoints, and 1 rad/s velocity and 2 rad/s² acceleration limits checked for stationary, small-motion, ordinary, and multisegment paths.
- Identical CPU and Metal sampling results for 128 samples from 4,096 points.
- Actual MPS tensor execution confirmed with CPU fallback disabled.

Timing examples below were measured after GPU pipeline warmup in the same process. Values include host submission and return overhead. They do not measure whole-episode acceleration or establish performance for all targets.

| Operation | CPU (ms) | Metal (ms) | Approximate speedup |
|---|---:|---:|---:|
| FK, 512 states | 6.3–6.6 | 0.57–0.60 | 11× |
| Collision checks, 512 states | 92–94 | 0.92–1.19 | 78–102× |
| Easy IK targets, median of 8 per arm | 6.2–6.6 | 1.9–2.1 | 3.2× |

IK first solves from the current configuration on the GPU. If that fails, it searches 64 candidates in parallel on the GPU. Speedups depend on the target.

## Compatibility and limits

The implementation uses the same official collision spheres, exclusions, and locked-gripper model as the CPU reference. Task-object contacts are handled by physics; collision classification does not certify task success.

Metal uses bounded quintic segments instead of CPU TOPPRA. It stops at segment boundaries and is not equivalent to cuRobo time-optimal trajectories. Final commanded trajectory samples are collision-checked. RRT graph control and small Cartesian interpolation preparations run on the host.

The upstream task-success checker and unstable-seed rejection criteria are retained. Runtime errors, planning failures, task failures after execution, and unstable initial states are recorded separately. Equivalent CUDA benchmark performance and other embodiments have not been verified.

## Expert-script results

A fixed implementation was evaluated on all 50 tasks, once per task at seed 0: 27 successes, 21 planning failures, one task failure, and one unstable-seed rejection. There were no runtime exceptions or timeouts. See the [task-by-task audit](artifacts/metal_expert_audit.md).

`adjust_bottle` and `place_container_plate` succeeded after failing in earlier development runs. `stamp_seal` is an unresolved regression: the current CPU implementation succeeds, while Metal fails. At the failed state, neither 64 CPU IK attempts nor 2,048 GPU candidates found a valid solution. Earlier motion or grasp-state differences may contribute, but the exact cause is unconfirmed.

The rejected `pick_diverse_bottles` seed 0 remains a failure in the original results. A separate Metal expert run at seed 1 succeeded and is not merged into the seed 0 counts.
