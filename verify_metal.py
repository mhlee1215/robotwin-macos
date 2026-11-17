"""Independent CPU/Metal numerical parity and timed batched-work verification."""
import os
import sys
import json
import time
from pathlib import Path
import argparse
import numpy as np

ROOT=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--output',default='artifacts/metal_numerics_v1');args=p.parse_args()
out=ROOT/args.output;out.mkdir(parents=True,exist_ok=True)
os.environ['ROBOTWIN_MAC_PLANNER']='metal'
os.environ.setdefault('VK_ICD_FILENAMES','/opt/homebrew/opt/molten-vk/etc/vulkan/icd.d/MoltenVK_icd.json')
os.environ.setdefault('DYLD_LIBRARY_PATH','/opt/homebrew/opt/vulkan-loader/lib:/opt/homebrew/opt/molten-vk/lib')
repo=ROOT/'vendor/RoboTwin';sys.path.insert(0,str(repo));os.chdir(repo)
import yaml
from envs.handover_block import handover_block
from robotwin_macos.cpu_planner import CpuPlanner
from robotwin_macos.metal_runtime import runtime,farthest_points
from scipy.spatial.transform import Rotation

config=yaml.safe_load((repo/'env_cfg/task_config/demo_clean.yml').read_text())
embodiment=repo/'assets/embodiments/aloha-agilex'
robot_config=yaml.safe_load((embodiment/'config.yml').read_text())
config.update(task_name='handover_block',seed=0,now_ep_num=0,left_robot_file=str(embodiment),right_robot_file=str(embodiment),left_embodiment_config=robot_config,right_embodiment_config=robot_config,dual_arm_embodied=True,save_data=False,save_freq=None,eval_mode=True,render_freq=0,eval_video_log=False)
task=handover_block();task.setup_demo(**config)
report={'device':runtime().device,'arms':{},'timings':{},'passed':False}
try:
 for side in ['left','right']:
    gpu=getattr(task.robot,side+'_planner')
    cpu=CpuPlanner(gpu.robot,getattr(task.robot,side+'_arm_joints_name'),getattr(task.robot,side+'_move_group'),getattr(task.robot,side+'_urdf_path'),getattr(task.robot,side+'_srdf_path'),task.scene)
    current=gpu.robot.get_qpos().astype(float);cpu.current=current;gpu.current=current
    home=current[gpu.indices];rng=np.random.default_rng(17)
    samples=rng.uniform(gpu.lower,gpu.upper,(512,6));samples[:32]=np.clip(home+rng.normal(0,.12,(32,6)),gpu.lower,gpu.upper)
    # Warm GPU pipeline compilation before recording wall times.
    gpu.poses(samples[:1]);gpu.collision_codes(samples[:1])
    start=time.perf_counter();reference=np.array([cpu._pose(q) for q in samples]);cpu_fk=time.perf_counter()-start
    start=time.perf_counter();actual=gpu.poses(samples);gpu_fk=time.perf_counter()-start
    fk_error=float(np.max(np.abs(reference-actual)));assert fk_error<1e-5,fk_error
    start=time.perf_counter();expected=np.array([cpu._collision_free(q) for q in samples]);cpu_collision=time.perf_counter()-start
    start=time.perf_counter();observed=gpu.collision_codes(samples)==0;gpu_collision=time.perf_counter()-start
    mismatch=int(np.count_nonzero(expected!=observed));assert mismatch==0,mismatch
    targets=[q for q,valid in zip(samples[:32],expected[:32]) if valid][:8]
    assert len(targets)==8
    errors=[];cpu_times=[];gpu_times=[]
    for q in targets:
        target=cpu._pose(q)
        start=time.perf_counter();cpu_solution=cpu._ik(target,home);cpu_times.append(time.perf_counter()-start)
        start=time.perf_counter();solution=gpu._ik(target,home);gpu_times.append(time.perf_counter()-start)
        assert cpu_solution is not None and solution is not None
        actual=cpu._pose(solution)
        pe=float(np.linalg.norm(actual[:3,3]-target[:3,3]));ae=float(Rotation.from_matrix(target[:3,:3].T@actual[:3,:3]).magnitude())
        assert pe<.002 and ae<.02 and cpu._collision_free(solution)
        errors.append([pe,ae])
    for scale in [0.,1e-8,1e-5,.0009,.08,1.]:
        path=np.array([home,home+scale,home-scale/2])
        times,pos,vel,acc,duration=gpu.TOPP(path)
        assert np.isfinite(pos).all() and np.max(np.abs(vel))<=1.001 and np.max(np.abs(acc))<=2.001
        np.testing.assert_allclose(pos[-1],path[-1],atol=2e-6)
        assert times[-1]==duration and np.all(np.diff(times)>0)
    report['arms'][side]={'random_configs':512,'fk_max_error':fk_error,'collision_disagreements':mismatch,'ik_targets':8,'ik_max_position_error':max(e[0] for e in errors),'ik_max_angle_error':max(e[1] for e in errors)}
    report['timings'][side]={'cpu_fk_512_s':cpu_fk,'metal_fk_512_s':gpu_fk,'cpu_collisions_512_s':cpu_collision,'metal_collisions_512_s':gpu_collision,'cpu_ik_median_s':float(np.median(cpu_times)),'metal_ik_median_s':float(np.median(gpu_times))}
    print(side,report['arms'][side],report['timings'][side],flush=True)
 points=rng.normal(size=(4096,3)).astype(np.float32)
 farthest_points(points,4)
 start=time.perf_counter();actual=farthest_points(points,128);gpu_fps=time.perf_counter()-start
 start=time.perf_counter();dist=np.full(len(points),np.inf,dtype=np.float32);chosen=0;expected=[]
 for _ in range(128):
    expected.append(chosen);dist=np.minimum(dist,((points-points[chosen])**2).sum(-1));chosen=int(dist.argmax())
 cpu_fps=time.perf_counter()-start
 assert np.array_equal(actual,expected)
 report['fps']={'points':4096,'samples':128,'indices_equal':True,'cpu_seconds':cpu_fps,'metal_seconds':gpu_fps}
 # Verify MPS tensor work without enabling any unsupported-op CPU fallback.
 import torch
 assert os.environ.get('PYTORCH_ENABLE_MPS_FALLBACK','0')!='1'
 x=torch.as_tensor(points,device='mps');rotation=torch.eye(3,device='mps');y=x@rotation
 torch.mps.synchronize();np.testing.assert_allclose(y.cpu().numpy(),points)
 report['mps']={'device':str(y.device),'cpu_fallback_enabled':False}
 report['kernels']=runtime().stats;report['passed']=True
finally:
 (out/'result.json').write_text(json.dumps(report,indent=2)+'\n')
 task.close_env()
print('Metal parity, limits, IK, FPS and MPS checks passed',flush=True)
