"""Run an actual upstream RoboTwin task with native macOS backends."""
import argparse
import importlib
import json
import os
from pathlib import Path
import sys
import time
import traceback
import hashlib
import subprocess

ROOT = Path(__file__).resolve().parent
ORIGINAL_CWD = Path.cwd()
REPO = ROOT / 'vendor/RoboTwin'
sys.path.insert(0, str(REPO))
os.environ.setdefault('VK_ICD_FILENAMES', '/opt/homebrew/opt/molten-vk/etc/vulkan/icd.d/MoltenVK_icd.json')
os.environ.setdefault('DYLD_LIBRARY_PATH', '/opt/homebrew/opt/vulkan-loader/lib:/opt/homebrew/opt/molten-vk/lib')
os.chdir(REPO)

import numpy as np
from PIL import Image, ImageDraw
import yaml

p = argparse.ArgumentParser()
p.add_argument('--task', default='handover_block')
p.add_argument('--seed', type=int, default=0)
p.add_argument('--mode', choices=['reset', 'qpos', 'simulator', 'ee', 'planner', 'expert'], default='reset')
p.add_argument('--reset-check', action='store_true', help='Close and reinitialize the same task instance after checking it')
p.add_argument('--pointcloud', action='store_true')
p.add_argument('--randomized', action='store_true')
p.add_argument('--save-data', action='store_true')
p.add_argument('--overview', action='store_true', help='Render a 1600x1200 external view of both robot arms')
p.add_argument('--backend', choices=['cpu', 'metal'], default=os.environ.get('ROBOTWIN_MAC_PLANNER', 'metal'))
p.add_argument('--output', type=Path)
args = p.parse_args()
os.environ['ROBOTWIN_MAC_PLANNER'] = args.backend
if args.backend == 'metal':
    os.environ['PYTORCH_ENABLE_MPS_FALLBACK'] = '0'
out = args.output or ROOT / 'artifacts' / f'{args.task}_{args.seed}_{args.mode}'
if not out.is_absolute():
    out = ORIGINAL_CWD / out
if args.save_data and (out/'dataset').exists():
    p.error(f'Dataset output already exists: {out}; choose a new --output')
out.mkdir(parents=True, exist_ok=True)
result = {'task': args.task, 'seed': args.seed, 'mode': args.mode,
          'backend': 'macos_'+args.backend, 'randomized': args.randomized, 'started': time.time(),
          'upstream_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
          'source_sha256': {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                            for path in [ROOT/'run_task.py', ROOT/'robotwin_macos/cpu_planner.py',
                                         REPO/'envs/_base_task.py', REPO/'envs/robot/robot.py', REPO/'envs/camera/camera.py',
                                         REPO/'envs'/f'{args.task}.py', ROOT/'robotwin_macos/metal_planner.py',
                                         ROOT/'robotwin_macos/metal_runtime.py', ROOT/'robotwin_macos/metal/planner.metal',
                                         ROOT/'robotwin_macos/metal/runtime.mm']}}

try:
    config_name = 'demo_randomized' if args.randomized else 'demo_clean'
    config = yaml.safe_load((REPO / f'env_cfg/task_config/{config_name}.yml').read_text())
    embodiment = REPO / 'assets/embodiments/aloha-agilex'
    robot_config = yaml.safe_load((embodiment / 'config.yml').read_text())
    config.update(task_name=args.task, seed=args.seed, now_ep_num=0,
                  left_robot_file=str(embodiment), right_robot_file=str(embodiment),
                  left_embodiment_config=robot_config, right_embodiment_config=robot_config,
                  dual_arm_embodied=True, save_data=args.save_data, save_freq=15 if args.save_data else None,
                  save_path=str(out/'dataset'),
                  eval_mode=True, render_freq=0, eval_video_log=False)
    config['data_type'].update(depth=True, pointcloud=args.pointcloud)
    cls = getattr(importlib.import_module('envs.' + args.task), args.task)
    frames, stages = [], []
    if args.mode == 'expert':
        class RecordedTask(cls):
            def move(self, *actions, **kwargs):
                value = super().move(*actions, **kwargs)
                obs = self.get_obs()
                names = ['head_camera', 'front_camera']
                frame = Image.fromarray(np.concatenate([obs['observation'][n]['rgb'] for n in names], axis=1))
                ImageDraw.Draw(frame).text((5, 5), f'{args.task}: move {len(frames)+1}, plan={self.plan_success}', fill='black')
                frames.append(frame)
                if args.overview:
                    capture_overview(f'overview_move_{len(frames):02d}')
                stages.append({'move': len(frames), 'plan_success': bool(self.plan_success),
                               'task_success': bool(self.check_success()),
                               'actors': {a.name: a.get_pose().p.tolist() for a in self.scene.get_all_actors()}})
                return value
        cls = RecordedTask
    task = cls()
    task.setup_demo(**config)
    if args.overview:
        import sapien
        camera_position = np.array([1.1, 0.25, 1.6])
        camera_target = np.array([0., -0.30, 1.0])
        forward = camera_target-camera_position
        forward /= np.linalg.norm(forward)
        left = np.cross([0., 0., 1.], forward)
        left /= np.linalg.norm(left)
        camera_pose = np.eye(4)
        camera_pose[:3, :3] = np.column_stack([forward, left, np.cross(forward, left)])
        camera_pose[:3, 3] = camera_position
        overview = task.scene.add_camera('overview', 1600, 1200, np.deg2rad(60), 0.05, 10)
        overview.entity.set_pose(sapien.Pose(camera_pose))
        result['overview_camera'] = {'position': camera_position.tolist(), 'target': camera_target.tolist(),
                                     'resolution': [1600, 1200], 'fovy_degrees': 60}
        def capture_overview(label):
            task._update_render()
            overview.take_picture()
            rgb = (overview.get_picture('Color')[:, :, :3]*255).clip(0, 255).astype('uint8')
            assert rgb.shape == (1200, 1600, 3) and rgb.std() > 1
            Image.fromarray(rgb).save(out/f'{label}.png')
        capture_overview('overview_initial')
    result['physics_system'] = type(task.scene.physx_system).__name__
    result['render_system'] = type(task.scene.render_system).__name__
    assert result['physics_system'] == 'PhysxCpuSystem'
    result['reset_pass'] = True
    result['settle_windows'] = task.settle_windows
    def capture(label):
        obs = task.get_obs()
        cameras = {}
        for name, values in obs['observation'].items():
            if 'rgb' not in values:
                continue
            rgb = values['rgb']
            assert rgb.ndim == 3 and rgb.shape[-1] == 3 and rgb.std() > 1
            Image.fromarray(rgb).save(out / f'{label}_{name}.png')
            cameras[name] = {'shape': list(rgb.shape), 'std': float(rgb.std())}
            if 'depth' in values:
                depth = values['depth']
                assert np.isfinite(depth).all() and np.count_nonzero(depth) > 100
                cameras[name]['depth_nonzero'] = int(np.count_nonzero(depth))
        assert len(cameras) >= 3, cameras
        if args.pointcloud:
            cloud = np.asarray(obs['pointcloud'])
            assert cloud.ndim == 2 and cloud.shape[0] > 0 and np.isfinite(cloud).all()
            result['pointcloud_shape'] = list(cloud.shape)
            result['pointcloud_device'] = getattr(task.cameras, 'pointcloud_device', 'unreported')
        result[label+'_cameras'] = cameras
    capture('initial')
    initial = task.robot.left_entity.get_qpos().copy()
    result['initial_success'] = bool(task.check_success())
    if args.mode == 'planner':
        planner = task.robot.left_planner
        planner.current = initial.copy()
        fk = planner.robot.get_root_pose().to_transformation_matrix() @ planner._pose(initial[planner.indices])
        actual = planner.robot.get_links()[planner.tip].get_pose().to_transformation_matrix()
        result['fk_max_error'] = float(np.max(np.abs(fk-actual)))
        result['current_collision_free'] = planner._collision_free(initial[planner.indices])
        result['collision'] = planner.last_collision
        assert result['fk_max_error'] < 1e-4
        assert result['current_collision_free']
        import sapien
        nearby = sapien.Pose(actual[:3, 3]+[0, 0, 0.02], planner.robot.get_links()[planner.tip].pose.q)
        planned = planner.plan_path(initial, nearby)
        assert planned['status'] == 'Success', planned
        reached = planner.robot.get_root_pose().to_transformation_matrix() @ planner._pose(planned['position'][-1])
        result['planned_endpoint_error_m'] = float(np.linalg.norm(reached[:3, 3]-nearby.p))
        assert result['planned_endpoint_error_m'] < 0.003
        unreachable = planner.plan_path(initial, sapien.Pose([10, 10, 10]))
        result['unreachable_rejected'] = unreachable['status'] != 'Success'
        assert result['unreachable_rejected']
        import hppfcl
        planner.obstacles.append(('regression_blocker', hppfcl.Box(0.2, 0.2, 0.2),
                                  hppfcl.Transform3f(np.eye(3), actual[:3, 3].astype(float))))
        result['blocking_obstacle_rejected'] = not planner._collision_free(initial[planner.indices])
        assert result['blocking_obstacle_rejected']
        planner.obstacles.pop()
    elif args.mode == 'ee':
        left = np.array(task.robot.get_left_ee_pose())
        right = np.array(task.robot.get_right_ee_pose())
        left[2] += 0.02
        task.take_action(np.r_[left, task.robot.get_left_gripper_val(), right,
                               task.robot.get_right_gripper_val()], action_type='ee')
        result['ee_position_error_m'] = float(np.linalg.norm(np.array(task.robot.get_left_ee_pose())[:3]-left[:3]))
        assert result['ee_position_error_m'] < 0.005
    elif args.mode == 'qpos':
        command = np.array(task.robot.get_left_arm_jointState()+task.robot.get_right_arm_jointState())
        command[0] += 0.08
        task.take_action(command, action_type='qpos')
        delta = float(np.max(np.abs(task.robot.left_entity.get_qpos()-initial)))
        result['joint_motion'] = delta
        assert delta > 0.01, 'qpos command produced no joint motion'
    elif args.mode == 'simulator':
        def physical_joint_state():
            values = []
            for side in ['left', 'right']:
                entity = getattr(task.robot, side+'_entity')
                joints = entity.get_active_joints()
                qpos = entity.get_qpos()
                assert np.isfinite(qpos).all() and np.isfinite(entity.get_qvel()).all()
                values.extend(float(qpos[joints.index(j)]) for j in getattr(task.robot, side+'_arm_joints'))
                gripper_joint = getattr(task.robot, side+'_gripper')[0][0]
                low, high = getattr(task.robot, side+'_gripper_scale')
                values.append(float((qpos[joints.index(gripper_joint)]-low)/(high-low)))
            return np.array(values)
        home = np.array(task.robot.get_left_arm_jointState()+task.robot.get_right_arm_jointState())
        physical_home = physical_joint_state()
        result['measurement_source'] = 'SAPIEN articulation qpos/qvel, not drive targets'
        assert home.shape == (14,) and np.isfinite(home).all()
        command = home.copy()
        command[0] += 0.06
        command[7] -= 0.06
        command[[6, 13]] = 0.4
        task.take_action(command, action_type='qpos')
        hold_steps = round(1.0/task.scene.timestep)
        result['hold_seconds_after_command'] = 1.0
        for _ in range(hold_steps):
            task.scene.step()
        actual = physical_joint_state()
        result['outbound_joint_error_rad'] = np.abs(actual[[0, 7]]-command[[0, 7]]).tolist()
        result['arm_motion_rad'] = np.abs(actual[[0, 7]]-physical_home[[0, 7]]).tolist()
        result['gripper_motion_normalized'] = np.abs(actual[[6, 13]]-physical_home[[6, 13]]).tolist()
        assert np.isfinite(actual).all()
        assert np.all(np.abs(actual[[0, 7]]-physical_home[[0, 7]]) > 0.02), 'An arm did not move'
        assert np.all(np.abs(actual[[0, 7]]-command[[0, 7]]) < 0.02), 'Arm target not tracked'
        assert np.all(np.abs(actual[[6, 13]]-physical_home[[6, 13]]) > 0.1), 'A gripper did not move'
        capture('moving')
        task.take_action(home, action_type='qpos')
        for _ in range(hold_steps):
            task.scene.step()
        actual = physical_joint_state()
        result['return_joint_error_rad'] = np.abs(actual[[0, 7]]-home[[0, 7]]).tolist()
        assert np.isfinite(actual).all()
        assert np.all(np.abs(actual[[0, 7]]-home[[0, 7]]) < 0.02), 'Return command not tracked'
        assert np.all(np.abs(actual[[6, 13]]-home[[6, 13]]) < 0.1), 'Gripper did not reopen'
        result['simulator_action_calls'] = task.take_action_cnt
        assert task.take_action_cnt == 2
        from robotwin_macos.metal_runtime import _runtime
        result['ik_calls'] = (_runtime.stats.get('inverse_kinematics', {}).get('calls', 0)
                              if _runtime is not None else 0)
        assert result['ik_calls'] == 0, 'Simulator check unexpectedly invoked IK'
    elif args.mode == 'expert':
        task.play_once()
        result['grasp_backoffs_m'] = getattr(task, 'mac_grasp_backoffs', [])
        result['plan_success'] = bool(task.plan_success)
        result['left_plans'] = len(task.left_joint_path)
        result['right_plans'] = len(task.right_joint_path)
        result['left_last_diagnostic'] = task.robot.left_planner.last_diagnostic
        result['right_last_diagnostic'] = task.robot.right_planner.last_diagnostic
        result['plans'] = {arm: [{k: (list(v.shape) if isinstance(v, np.ndarray) else v)
                                 for k, v in plan.items()} for plan in plans]
                           for arm, plans in [('left', task.left_joint_path), ('right', task.right_joint_path)]}
    capture('final')
    if args.overview:
        capture_overview('overview_final')
    result['task_success'] = bool(task.check_success())
    if args.mode == 'expert':
        result['planned_joints_legal'], result['joint_limit_excess'] = task.planned_joints_legal()
    result['passed'] = args.mode != 'expert' or (result['task_success'] and result['plan_success'] and result['planned_joints_legal'])
    if args.save_data and result['passed']:
        task._take_picture()
        task.merge_pkl_to_hdf5_video(instructions=[args.task.replace('_', ' ')])
        import h5py
        hdf5_path = out/'dataset/data/episode_0000000.hdf5'
        with h5py.File(hdf5_path) as data:
            result['hdf5_keys'] = list(data.keys())
            assert json.loads(data['instructions'].asstr()[()]) == [args.task.replace('_', ' ')]
        result['saved_frames'] = task.FRAME_IDX
    if args.reset_check:
        task.close_env()
        task.setup_demo(**config)
        capture('reset')
        result['reset_reinitialization_passed'] = True
    task.close_env()
except Exception as exc:
    result.update(passed=False, error=repr(exc), traceback=traceback.format_exc())
    traceback.print_exc()
    if exc.__class__.__name__ == 'UnStableError':
        import sapien
        result['unstable_state'] = []
        for actor in task.scene.get_all_actors():
            body = actor.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent)
            if body:
                result['unstable_state'].append({'name': actor.name, 'position': actor.pose.p.tolist(),
                    'velocity': body.linear_velocity.tolist(), 'angular_velocity': body.angular_velocity.tolist()})
        task._update_render(); task.cameras.update_picture()
        for name, data in task.cameras.get_rgb().items():
            Image.fromarray(data['rgb']).save(out/f'unstable_{name}.png')
finally:
    if args.backend == 'metal':
        from robotwin_macos.metal_runtime import _runtime
        if _runtime is not None:
            result['acceleration'] = {'device': _runtime.device, 'kernels': _runtime.stats,
                                      'physics': 'cpu', 'rendering': 'MoltenVK/Metal', 'cpu_solver_fallback': False}
    if 'frames' in globals() and frames:
        frames[0].save(out/'rollout.gif', save_all=True, append_images=frames[1:], duration=500, loop=0)
        (out/'stages.json').write_text(json.dumps(stages, indent=2)+'\n')
    result['elapsed_seconds'] = time.time()-result['started']
    (out / 'result.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result), flush=True)
sys.exit(0 if result['passed'] else 1)
