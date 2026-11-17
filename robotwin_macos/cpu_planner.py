"""CPU SE(3) IK, collision-checked joint paths and time parameterization.

The adapter implements RoboTwin's planner result protocol. It uses the actual
SAPIEN articulation kinematics; it never teleports the robot or task objects.
"""
import time
from pathlib import Path

import hppfcl
import numpy as np
import pinocchio as pin
import sapien
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation, Slerp
import toppra as ta
import toppra.algorithm as algo
import toppra.constraint as constraint
import yaml


class CpuPlanner:
    def __init__(self, robot, joint_names, tip_name, urdf, srdf, scene):
        self.robot = robot
        self.scene = scene
        self.kin = robot.create_pinocchio_model()
        names = [j.name for j in robot.get_active_joints()]
        self.indices = np.array([names.index(n) for n in joint_names])
        self.tip = [l.name for l in robot.get_links()].index(tip_name)
        limits = robot.get_qlimits()[self.indices].astype(float)
        self.lower = np.where(limits[:, 0] < -2*np.pi, -np.pi, limits[:, 0])
        self.upper = np.where(limits[:, 1] > 2*np.pi, np.pi, limits[:, 1])
        self.rng = np.random.default_rng(0)
        self.last_diagnostic = {}
        self.cmodel = pin.buildModelFromUrdf(str(Path(urdf).resolve()))
        self.cdata = self.cmodel.createData()
        self.cjoints = [self.cmodel.joints[self.cmodel.getJointId(n)] for n in names]
        self.geometry = pin.buildGeomFromUrdf(self.cmodel, str(Path(urdf).resolve()),
                                              pin.GeometryType.COLLISION,
                                              package_dirs=[str(Path(urdf).resolve().parent)])
        self.geometry.addAllCollisionPairs()
        if srdf and Path(srdf).is_file():
            pin.removeCollisionPairs(self.cmodel, self.geometry, str(Path(srdf).resolve()))
        self.geometry_data = pin.GeometryData(self.geometry)
        active_ids = {self.cmodel.getJointId(n) for n in joint_names}
        self.moving = []
        for geom in self.geometry.geometryObjects:
            joint = geom.parentJoint
            while joint and joint not in active_ids:
                joint = self.cmodel.parents[joint]
            self.moving.append(bool(joint))
        self.pairs = [pair for pair in self.geometry.collisionPairs
                      if self.moving[pair.first] or self.moving[pair.second]]
        # Use the actual scene geometry, including randomized table height.
        # Task objects remain contactable for grasping; SAPIEN handles contacts.
        self.obstacles = []
        for actor in scene.get_all_actors():
            if actor.name not in ('table', 'wall'):
                continue
            body = actor.find_component_by_type(sapien.physx.PhysxRigidBaseComponent)
            for shape in body.get_collision_shapes():
                if not isinstance(shape, sapien.physx.PhysxCollisionShapeBox):
                    raise TypeError('Expected box geometry for table/wall')
                pose = (actor.get_pose() * shape.get_local_pose()).to_transformation_matrix().astype(float)
                self.obstacles.append((actor.name, hppfcl.Box((2*shape.half_size).astype(float)),
                                       hppfcl.Transform3f(pose[:3, :3], pose[:3, 3])))
        self.collision_request = hppfcl.CollisionRequest()
        # Preserve upstream's declared planning collision approximation where
        # available. Full URDF meshes otherwise reject intended fingertip contact.
        side = 'left' if joint_names[0].startswith('fl_') else 'right'
        config_path = Path(urdf).resolve().parent.parent / f'curobo_{side}_tmp.yml'
        self.spheres = None
        if config_path.exists():
            config = yaml.safe_load(config_path.read_text())['robot_cfg']['kinematics']
            sphere_path = config_path.parent / Path(config['collision_spheres']).name
            data = yaml.safe_load(sphere_path.read_text())['collision_spheres']
            self.spheres = [(name, self.cmodel.getFrameId(name), np.asarray(s['center']), float(s['radius']))
                            for name in config['collision_link_names'] for s in data.get(name, []) if s['radius'] > 0]
            self.sphere_buffer = float(config.get('collision_sphere_buffer', 0))
            ignored = {frozenset((a, b)) for a, names_ in config['self_collision_ignore'].items() for b in names_}
            self.sphere_pairs = [(i, j) for i, a in enumerate(self.spheres) for j, b in enumerate(self.spheres[:i])
                                 if a[0] != b[0] and frozenset((a[0], b[0])) not in ignored]
            self.lock_joints = config.get('lock_joints', {})

    def _full(self, arm):
        q = self.current.copy()
        q[self.indices] = arm
        return q

    def _pose(self, arm):
        self.kin.compute_forward_kinematics(self._full(arm))
        frame = int(self.kin.link_id_to_frame_index[self.tip])
        return pin.updateFramePlacement(self.kin.model, self.kin.data, frame).homogeneous.copy()

    def _collision_free(self, arm):
        q = pin.neutral(self.cmodel)
        for joint, value in zip(self.cjoints, self._full(arm)):
            if joint.nq == 1:
                q[joint.idx_q] = value
            elif joint.nq == 2:
                q[joint.idx_q:joint.idx_q+2] = [np.cos(value), np.sin(value)]
        if self.spheres is not None:
            for name, value in self.lock_joints.items():
                q[self.cmodel.joints[self.cmodel.getJointId(name)].idx_q] = value
            pin.framesForwardKinematics(self.cmodel, self.cdata, q)
            centers = np.array([self.cdata.oMf[frame].act(center) for _, frame, center, _ in self.spheres])
            radii = np.array([s[3] for s in self.spheres])
            for i, j in self.sphere_pairs:
                if np.linalg.norm(centers[i]-centers[j]) < radii[i]+radii[j]:
                    self.last_collision = [self.spheres[i][0], self.spheres[j][0]]
                    return False
            root = self.robot.get_root_pose().to_transformation_matrix().astype(float)
            centers = centers @ root[:3, :3].T + root[:3, 3]
            for name, box, pose in self.obstacles:
                local = (centers-pose.getTranslation()) @ pose.getRotation()
                closest = np.clip(local, -box.halfSide, box.halfSide)
                penetration = radii+self.sphere_buffer - np.linalg.norm(local-closest, axis=1)
                hit = penetration > 0
                if np.any(hit):
                    self.last_collision = [self.spheres[np.flatnonzero(hit)[0]][0], name]
                    self.last_collision_depth = float(penetration[np.flatnonzero(hit)[0]])
                    return False
            self.last_collision = None
            return True
        pin.updateGeometryPlacements(self.cmodel, self.cdata, self.geometry,
                                     self.geometry_data, q)
        objects = self.geometry.geometryObjects
        poses = self.geometry_data.oMg
        def collide(a, ta_, b, tb):
            result = hppfcl.CollisionResult()
            return hppfcl.collide(a, ta_, b, tb, self.collision_request, result) > 0
        for pair in self.pairs:
            a, b = pair.first, pair.second
            if collide(objects[a].geometry, hppfcl.Transform3f(poses[a].rotation, poses[a].translation),
                       objects[b].geometry, hppfcl.Transform3f(poses[b].rotation, poses[b].translation)):
                self.last_collision = [objects[a].name, objects[b].name]
                return False
        root = self.robot.get_root_pose().to_transformation_matrix().astype(float)
        for i, moving in enumerate(self.moving):
            if not moving:
                continue
            world = root @ poses[i].homogeneous
            for name, geometry, pose in self.obstacles:
                if collide(objects[i].geometry, hppfcl.Transform3f(world[:3, :3], world[:3, 3]),
                           geometry, pose):
                    self.last_collision = [objects[i].name, name]
                    return False
        self.last_collision = None
        return True

    def _edge_free(self, a, b):
        n = max(2, int(np.ceil(np.max(np.abs(b-a)) / 0.04)) + 1)
        return all(self._collision_free(a + t * (b-a)) for t in np.linspace(0, 1, n))

    def _trajectory_free(self, positions):
        return all(self._collision_free(q) for q in positions[::5])

    def _ik(self, target, initial, attempts=16):
        self.last_ik_attempts = []
        def error(q):
            pose = self._pose(q)
            return np.r_[pose[:3, 3] - target[:3, 3],
                         Rotation.from_matrix(target[:3, :3].T @ pose[:3, :3]).as_rotvec()]
        for attempt in range(attempts):
            if attempt == 0:
                seed = initial
            elif attempt < 5:
                seed = initial + self.rng.normal(0, 0.4, len(initial))
            else:
                seed = self.rng.uniform(self.lower, self.upper)
            result = least_squares(error, np.clip(seed, self.lower+1e-7, self.upper-1e-7),
                                   bounds=(self.lower, self.upper), diff_step=1e-3,
                                   max_nfev=150, ftol=1e-7, xtol=1e-7, gtol=1e-7)
            err = error(result.x)
            diagnostic = {'position_error': float(np.linalg.norm(err[:3])),
                          'angle_error': float(np.linalg.norm(err[3:])),
                          'q': result.x.tolist()}
            self.last_ik_attempts.append(diagnostic)
            if np.linalg.norm(err[:3]) < 0.002 and np.linalg.norm(err[3:]) < 0.02:
                if self._collision_free(result.x):
                    return result.x
                diagnostic['collision'] = self.last_collision
                diagnostic['penetration'] = getattr(self, 'last_collision_depth', None)
        return None

    def _path(self, start, goal):
        if self._edge_free(start, goal):
            return [start, goal]
        # Bidirectional RRT-Connect, with collision checks on every edge.
        trees = [([start], [-1]), ([goal], [-1])]
        deadline = time.monotonic() + 4
        swapped = False
        def extend(tree, target):
            nodes, parents = tree
            idx = int(np.argmin(np.linalg.norm(np.asarray(nodes)-target, axis=1)))
            delta = target - nodes[idx]
            new = nodes[idx] + delta * min(1., 0.25 / max(np.linalg.norm(delta), 1e-12))
            if not self._edge_free(nodes[idx], new):
                return None
            nodes.append(new); parents.append(idx)
            return len(nodes)-1
        def trace(tree, idx):
            path = []
            while idx != -1:
                path.append(tree[0][idx]); idx = tree[1][idx]
            return path[::-1]
        while time.monotonic() < deadline:
            idx = extend(trees[0], self.rng.uniform(self.lower, self.upper))
            if idx is not None:
                target = trees[0][0][idx]
                for _ in range(100):
                    other = extend(trees[1], target)
                    if other is None:
                        break
                    if np.linalg.norm(trees[1][0][other]-target) < 1e-6:
                        path = trace(trees[0], idx) + trace(trees[1], other)[-2::-1]
                        return path[::-1] if swapped else path
            trees.reverse(); swapped = not swapped
        return None

    def _quintic_path(self, path, dt):
        samples = []
        elapsed = 0.
        for a, b in zip(path[:-1], path[1:]):
            delta = b-a
            distance = np.max(np.abs(delta))
            duration = max(0.1, 1.875*distance, np.sqrt(5.774*distance/2))
            times = np.linspace(0, duration, max(2, int(np.ceil(duration/dt))+1))
            u = times[:, None]/duration
            pos = a + (10*u**3-15*u**4+6*u**5)*delta
            vel = (30*u**2-60*u**3+30*u**4)*delta/duration
            acc = (60*u-180*u**2+120*u**3)*delta/duration**2
            first = 1 if samples else 0
            samples.append((times[first:]+elapsed, pos[first:], vel[first:], acc[first:]))
            elapsed += duration
        return (*[np.concatenate([s[i] for s in samples]) for i in range(4)], elapsed)

    def TOPP(self, path, dt=1/250, verbose=False):
        path = np.asarray(path, dtype=float)
        path = path[np.r_[True, np.max(np.abs(np.diff(path, axis=0)), axis=1) > 1e-8]]
        if len(path) == 1:
            z = np.zeros_like(path)
            return np.array([0.]), path, z, z, 0.
        if len(path) == 2 and np.max(np.abs(path[1]-path[0])) < 1e-3:
            # TOPPRA is ill-conditioned for nearly stationary IK corrections.
            return self._quintic_path(path, dt)
        distances = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
        spline = ta.SplineInterpolator(distances / distances[-1], path)
        trajectory = algo.TOPPRA([constraint.JointVelocityConstraint(np.ones(path.shape[1])),
                                  constraint.JointAccelerationConstraint(np.ones(path.shape[1])*2)],
                                 spline, parametrizer='ParametrizeConstAccel').compute_trajectory(0, 0)
        if trajectory is None:
            return self._quintic_path(path, dt)
        times = np.linspace(0, trajectory.duration, max(2, int(np.ceil(trajectory.duration/dt))+1))
        return times, trajectory(times), trajectory(times, 1), trajectory(times, 2), trajectory.duration

    def plan_path(self, curr_joint_pos, target_gripper_pose, constraint_pose=None, arms_tag=None):
        self.current = np.asarray(curr_joint_pos, dtype=float).copy()
        start = self.current[self.indices]
        target = (self.robot.get_root_pose().inv() * target_gripper_pose).to_transformation_matrix().astype(float)
        goal = self._ik(target, start)
        if goal is None:
            self.last_diagnostic = {'reason': 'ik_or_collision', 'target': target[:3, 3].tolist(),
                                    'collision': getattr(self, 'last_collision', None),
                                    'attempts': self.last_ik_attempts}
            return {'status': 'Fail', **self.last_diagnostic}
        if constraint_pose is not None and np.any(constraint_pose):
            # Follow Cartesian interpolation for held pose components rather than
            # silently dropping upstream's straight-motion constraints.
            initial = self._pose(start)
            rotations = Slerp([0, 1], Rotation.from_matrix([initial[:3, :3], target[:3, :3]]))
            path = [start]
            n = max(3, int(np.linalg.norm(target[:3, 3]-initial[:3, 3])/0.01)+1)
            for t in np.linspace(0, 1, n)[1:]:
                waypoint = target.copy()
                waypoint[:3, 3] = (1-t)*initial[:3, 3] + t*target[:3, 3]
                waypoint[:3, :3] = rotations(t).as_matrix()
                q = self._ik(waypoint, path[-1], attempts=1)
                if q is None or not self._edge_free(path[-1], q):
                    return {'status': 'Fail', 'reason': 'cartesian_constraint'}
                path.append(q)
        else:
            path = self._path(start, goal)
        if path is None:
            return {'status': 'Fail', 'reason': 'collision_path'}
        _, pos, vel, _, _ = self.TOPP(path)
        # Spline parameterization can round corners: validate final samples too.
        if not self._trajectory_free(pos):
            # Stop at collision-checked RRT corners rather than spline-cutting
            # through an obstacle. The fallback respects the same speed bounds.
            _, pos, vel, _, _ = self._quintic_path(np.asarray(path), 1/250)
            if not self._trajectory_free(pos):
                return {'status': 'Fail', 'reason': 'parameterized_collision'}
        self.last_diagnostic = {'reason': None}
        return {'status': 'Success', 'position': pos, 'velocity': vel}

    def plan_batch(self, curr_joint_pos, target_gripper_pose_list, constraint_pose=None, arms_tag=None):
        results = [self.plan_path(curr_joint_pos, target, constraint_pose, arms_tag)
                   for target in target_gripper_pose_list]
        return {'status': np.array([r['status'] for r in results]),
                'position': [r.get('position') for r in results],
                'velocity': [r.get('velocity') for r in results]}

    def plan_grippers(self, now_val, target_val):
        return {'num_step': 200, 'per_step': (target_val-now_val)/200,
                'result': np.linspace(now_val, target_val, 200)}
