# Simulator verification

These checks establish that the macOS port can run RoboTwin environments. Expert task success, IK search performance, and VLA performance are separate measurements.

Test platform: Apple M5 Pro, macOS 26.6.2, Python 3.11.15, and SAPIEN 3.0.3, with the default ALOHA-agilex embodiment and offscreen cameras.

## Results

All 50 task environments were exercised using explicitly recorded valid seeds. At seed 0, 49 of 50 passed. `pick_diverse_bottles` was rejected by the initial object-stability check at seed 0 and passed a separate test at seed 1. The rejected run remains in the records. This does not establish validity for every seed or expert task success.

- Four RGB-D cameras, physical arm movement and return, and gripper closing and opening were checked in all 50 environments.
- Maximum measured joint target error was approximately 0.00089 rad; maximum return error was approximately 0.00068 rad.
- The simulator suite made zero IK calls and executed joint commands through the physics engine.
- A representative randomized environment passed MPS pointcloud and 1600×1200 overview-camera checks.
- The representative randomized environment and the bottle task at seed 1 passed close-and-reinitialize checks on the same task instance.
- Independent tests passed gravity, ground contact, motor-drive, and RGB-D rendering checks.
- The physics image and full-robot randomized image were visually inspected.

## Procedure

`run_task.py --mode simulator` performs the following checks:

1. Load the upstream task, assets, and default ALOHA-agilex robot.
2. Check that all four cameras produce nonempty RGB and valid depth.
3. Command the arms by +0.06 and -0.06 rad respectively and close the grippers.
4. Hold the targets for one simulated second, then read actual articulation `qpos/qvel`. Drive targets and cached gripper commands are not used as physical measurements.
5. Verify that each arm moved more than 0.02 rad, target error is below 0.02 rad, and measured gripper positions changed materially.
6. Command return and gripper opening, then verify the measured physical return using the same procedure.
7. Call observation and task-success APIs and close the environment. Assert zero IK calls.

The existing action API uses time interpolation for joint commands. The test does not search for target gripper poses or generate an expert task plan.

Additional representative checks cover MPS pointcloud processing, a 1600×1200 external camera, and `close_env()` followed by `setup_demo()` on the same instance. A separate SAPIEN probe checks free fall, ground contact, a driven hinge, and RGB-D rendering.

## Reproduction

Run from the repository root. Use a new output name for each run to preserve previous evidence.

```sh
# Direct joint control, measured physics, observations, and reinitialization
.venv/bin/python run_task.py --mode simulator --reset-check --pointcloud --overview --output artifacts/my_simulator_check

# Randomized observations
.venv/bin/python run_task.py --mode simulator --randomized --overview --pointcloud --output artifacts/my_randomized_check

# All 50 tasks at seed 0
.venv/bin/python validate_suite.py --mode simulator --name my_simulator_suite

# Explicit separate retry after initial stability rejection
.venv/bin/python run_task.py --mode simulator --task pick_diverse_bottles --seed 1 --reset-check --output artifacts/my_bottle_seed1

# Independent physics and rendering checks
sh mac_python.sh "$PWD/probe_sapien.py" --render --output artifacts/my_physics_check
```

The suite returns exit code 1 if any task fails, including an initial stability rejection. A separate seed retry does not change the original result.

## Records

| Record | Contents |
|---|---|
| [Aggregate verification](artifacts/simulator_verification_final.json) | Verdict, seeds, measurements, and scope |
| [Full seed 0 suite](artifacts/simulator_all50_final/summary.json) | 49 passes and the retained rejection |
| [Bottle task, seed 1](artifacts/simulator_diverse_seed1_final/result.json) | Separate retry and reinitialization |
| [Randomized environment](artifacts/simulator_canary_physical_v3/result.json) | Physical states, cameras, and pointcloud |
| [Independent physics](artifacts/simulator_physics_final/sapien_render.json) | Gravity, contact, motor, and rendering |
| [Relocated checkout check](artifacts/release_smoke/result.json) | Representative simulator run after packaging |

The [source manifest](SOURCE_MANIFEST.json) records SHA-256 hashes of the executable code and patch used for verification. Selected records and representative images are included; local path prefixes in JSON were anonymized. New outputs, full datasets, and intermediate diagnostics are excluded from Git.

## Scope

The checks cover the default ALOHA embodiment and offscreen image-based operation. Other hardware, every seed, other embodiments, interactive GUI operation, policy inference, and equivalence to the upstream CUDA benchmark were not verified. Expert-planning results are reported separately and are not counted as simulator pass/fail outcomes.
