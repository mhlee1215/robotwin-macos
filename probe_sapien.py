"""Assert native CPU dynamics, driven joint, and optionally offscreen RGB-D."""
import argparse
import json
from pathlib import Path

import numpy as np
import sapien

p = argparse.ArgumentParser()
p.add_argument('--render', action='store_true')
p.add_argument('--output', type=Path, default=Path('artifacts'))
args = p.parse_args()
out = Path(__file__).resolve().parent / args.output
out.mkdir(parents=True, exist_ok=True)
scene = sapien.Scene() if args.render else sapien.Scene([sapien.physx.PhysxCpuSystem()])
scene.set_timestep(1 / 240)
scene.add_ground(0, render=args.render)
b = scene.create_actor_builder()
b.add_box_collision(half_size=[0.05] * 3)
if args.render:
    b.add_box_visual(half_size=[0.05] * 3, material=[1., 0.05, 0.05])
b.initial_pose = sapien.Pose([0, 0, 0.8])
cube = b.build(name='falling_cube')
ab = scene.create_articulation_builder()
root = ab.create_link_builder()
root.set_name('base')
link = ab.create_link_builder(root)
link.set_name('moving_link')
link.set_joint_name('hinge')
link.set_joint_properties('revolute', limits=[[-1., 1.]], pose_in_parent=sapien.Pose([0, 0, 0.3]), pose_in_child=sapien.Pose())
link.add_box_collision(half_size=[0.03, 0.03, 0.1])
if args.render:
    link.add_box_visual(half_size=[0.03, 0.03, 0.1], material=[0.05, 0.3, 1.])
robot = ab.build(fix_root_link=True)
robot.set_root_pose(sapien.Pose([0, 0.25, 0]))
joint = robot.get_active_joints()[0]
joint.set_drive_properties(stiffness=100, damping=10)
joint.set_drive_target(0.5)
for _ in range(480):
    scene.step()
z = float(cube.get_pose().p[2])
q = float(robot.get_qpos()[0])
assert abs(z - 0.05) < 0.005, z
assert abs(q - 0.5) < 0.02, q
result = dict(sapien=sapien.__version__, cube_final_z=z, joint_target=0.5, joint_final=q, physics_pass=True)
if args.render:
    from PIL import Image
    scene.set_ambient_light([0.5, 0.5, 0.5])
    scene.add_directional_light([1, 0, -1], [0.7, 0.7, 0.7])
    camera = scene.add_camera('camera', 320, 240, 1.0, 0.01, 10)
    camera.set_pose(sapien.Pose([-0.8, 0, 0.55], [0.966, 0, 0.259, 0]))
    scene.update_render()
    camera.take_picture()
    rgb = camera.get_picture('Color')[..., :3]
    depth = -camera.get_picture('Position')[..., 2]
    assert np.isfinite(rgb).all() and float(rgb.std()) > 0.01
    valid = (depth > 0) & (depth < 10)
    assert valid.sum() > 100
    Image.fromarray((rgb.clip(0, 1) * 255).astype('uint8')).save(out / 'sapien_rgb.png')
    np.save(out / 'sapien_depth.npy', depth)
    result.update(render_pass=True, rgb_std=float(rgb.std()), valid_depth_pixels=int(valid.sum()))
path = out / ('sapien_render.json' if args.render else 'sapien_physics.json')
path.write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps(result), flush=True)
