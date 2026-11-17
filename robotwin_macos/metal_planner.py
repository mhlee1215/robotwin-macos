"""Metal batched IK, FK, sphere collisions and bounded trajectory evaluation.

CPU work is model parsing, simulator I/O and graph/command orchestration.
No CPU numerical-solver fallback is used for the supported six-axis ALOHA arm.
"""
import xml.etree.ElementTree as ET
import numpy as np
import pinocchio as pin
from .cpu_planner import CpuPlanner
from .metal_runtime import runtime


def matrices(a):
    return np.asarray(a,dtype=np.float32).swapaxes(-1,-2).copy()


class MetalPlanner(CpuPlanner):
    def __init__(self, robot, joint_names, tip_name, urdf, srdf, scene):
        super().__init__(robot,joint_names,tip_name,urdf,srdf,scene)
        if len(joint_names)!=6 or self.spheres is None or len(self.spheres)>128:
            raise ValueError('Metal planner requires a six-axis arm with <=128 declared collision spheres')
        self.gpu=runtime()
        self.current=robot.get_qpos().astype(float)
        q=pin.neutral(self.cmodel)
        for joint,value in zip(self.cjoints,self.current):
            if joint.nq==1:q[joint.idx_q]=value
            elif joint.nq==2:q[joint.idx_q:joint.idx_q+2]=[np.cos(value),np.sin(value)]
        ids=[self.cmodel.getJointId(name) for name in joint_names]
        for jid in ids:
            joint=self.cmodel.joints[jid]
            if joint.nq==1:q[joint.idx_q]=0
            elif joint.nq==2:q[joint.idx_q:joint.idx_q+2]=[1,0]
            else:raise ValueError('Unsupported joint representation')
        for name,value in self.lock_joints.items():
            q[self.cmodel.joints[self.cmodel.getJointId(name)].idx_q]=value
        pin.framesForwardKinematics(self.cmodel,self.cdata,q)
        transforms=[self.cdata.oMi[j].homogeneous.copy() for j in ids]
        origins=[];prev=np.eye(4)
        xml=ET.parse(urdf).getroot()
        joints={j.attrib['name']:j for j in xml.findall('joint')}
        axes=[]
        for i,(name,T) in enumerate(zip(joint_names,transforms)):
            joint=joints[name]
            if joint.attrib['type'] not in ('revolute','continuous'):
                raise ValueError('Metal IK requires revolute arm joints')
            if i and self.cmodel.parents[ids[i]]!=ids[i-1]:
                raise ValueError('Expected serial six-axis chain')
            origins.append(np.linalg.inv(prev)@T);prev=T
            axis=np.fromstring(joint.find('axis').attrib['xyz'],sep=' ')
            axes.append(np.r_[axis/np.linalg.norm(axis),0])
        tip_id=self.cmodel.getFrameId(tip_name)
        if not np.allclose(self.cdata.oMf[tip_id].homogeneous,transforms[-1],atol=1e-6):
            raise ValueError('Tip frame differs from final joint frame')
        spheres=[];attached=[]
        for _,frame,center,radius in self.spheres:
            parent=self.cmodel.frames[frame].parent
            while parent and parent not in ids:parent=self.cmodel.parents[parent]
            anchor=ids.index(parent) if parent in ids else -1
            xyz=self.cdata.oMf[frame].act(center)
            if anchor>=0:xyz=(np.linalg.inv(transforms[anchor])@np.r_[xyz,1])[:3]
            spheres.append(np.r_[xyz,radius]);attached.append(anchor)
        self.origins=matrices(origins)
        self.axes=np.asarray(axes,dtype=np.float32)
        self.limits=np.column_stack([self.lower,self.upper]).astype(np.float32)
        self.gpu_spheres=np.asarray(spheres,dtype=np.float32)
        self.attached=np.asarray(attached,dtype=np.int32)
        self.gpu_pairs=np.asarray(self.sphere_pairs or [(0,0)],dtype=np.uint32)

    def poses(self, configs):
        configs=np.asarray(configs,dtype=np.float32).reshape(-1,6)
        if not len(configs) or not np.isfinite(configs).all():
            raise ValueError('Joint configurations must be nonempty and finite')
        out=self.gpu.run('forward_kinematics',len(configs),self.origins,self.axes,configs,
                         np.zeros((len(configs),4,4),dtype=np.float32))
        return out.swapaxes(-1,-2).astype(float)

    def _pose(self, arm):
        return self.poses([arm])[0]

    def collision_codes(self, configs):
        configs=np.asarray(configs,dtype=np.float32).reshape(-1,6)
        if not len(configs) or not np.isfinite(configs).all():
            raise ValueError('Joint configurations must be nonempty and finite')
        root=self.robot.get_root_pose().to_transformation_matrix().astype(float)
        boxes=[];half=[]
        for _,box,pose in self.obstacles:
            T=np.eye(4);T[:3,:3]=pose.getRotation();T[:3,3]=pose.getTranslation()
            boxes.append(np.linalg.inv(T)@root)
            half.append(np.r_[box.halfSide,self.sphere_buffer])
        return self.gpu.run('collisions',len(configs),self.origins,self.axes,self.gpu_spheres,
            self.attached,self.gpu_pairs,matrices(boxes or [np.eye(4)]),
            np.asarray(half or [[0,0,0,0]],dtype=np.float32),
            np.array([len(self.spheres),len(self.sphere_pairs),len(boxes)],dtype=np.uint32),
            configs,np.zeros(len(configs),dtype=np.uint32))

    def _collision_label(self, code):
        if not code:return None
        code=int(code)-1
        if code<len(self.sphere_pairs):
            i,j=self.sphere_pairs[code];return [self.spheres[i][0],self.spheres[j][0]]
        box,sphere=divmod(code-len(self.sphere_pairs),len(self.spheres))
        return [self.spheres[sphere][0],self.obstacles[box][0]]

    def _collision_free(self, arm):
        code=self.collision_codes([arm])[0]
        self.last_collision=self._collision_label(code)
        return bool(code==0)

    def _edge_free(self, a, b):
        n=max(2,int(np.ceil(np.max(np.abs(b-a))/.04))+1)
        return bool(np.all(self.collision_codes(np.linspace(a,b,n))==0))

    def _trajectory_free(self, positions):
        # Inspect every generated command, including the final endpoint.
        return bool(np.all(self.collision_codes(positions)==0))

    def _ik(self, target, initial, attempts=16):
        if not np.isfinite(target).all() or not np.isfinite(initial).all():
            raise ValueError('IK inputs must be finite')
        count=1 if attempts==1 else max(64,attempts*4)
        seeds=self.rng.uniform(self.lower,self.upper,(count,6))
        seeds[0]=initial
        if count>1:
            near=min(16,count-1)
            seeds[1:near+1]=np.clip(initial+self.rng.normal(0,.6,(near,6)),self.lower,self.upper)
        def solve(batch):
            solved=self.gpu.run('inverse_kinematics',len(batch),self.origins,self.axes,self.limits,
                batch.astype(np.float32),matrices([target]),np.array([240],dtype=np.uint32),
                np.zeros((len(batch),8),dtype=np.float32))
            codes=self.collision_codes(solved[:,:6])
            valid=(solved[:,6]<.002)&(solved[:,7]<.02)&(codes==0)&np.isfinite(solved).all(axis=1)
            return solved,codes,valid
        solved,codes,valid=solve(seeds[:1])
        if not valid[0] and count>1:
            solved,codes,valid=solve(seeds)
        ranking=np.argsort(np.linalg.norm(solved[:,:6]-initial,axis=1))
        self.last_ik_attempts=[{'position_error':float(solved[i,6]),'angle_error':float(solved[i,7]),
            'q':solved[i,:6].tolist(),'collision':self._collision_label(codes[i])} for i in np.argsort(solved[:,6]+solved[:,7])[:16]]
        self.last_collision=self.last_ik_attempts[0]['collision']
        for i in ranking:
            if valid[i]:return solved[i,:6].astype(float)
        return None

    def _quintic_path(self, path, dt):
        return self.TOPP(path, dt)

    def TOPP(self, path, dt=1/250, verbose=False):
        path=np.asarray(path,dtype=float)
        if path.ndim!=2 or path.shape[1]!=6 or not np.isfinite(path).all():
            raise ValueError('Trajectory must be finite [N,6]')
        path=path[np.r_[True,np.max(np.abs(np.diff(path,axis=0)),axis=1)>1e-8]]
        if len(path)==1:
            z=np.zeros_like(path);return np.array([0.]),path,z,z,0.
        samples=[];elapsed=0.
        for i,(a,b) in enumerate(zip(path[:-1],path[1:])):
            distance=np.max(np.abs(b-a))
            duration=max(.1,1.875*distance,np.sqrt(5.774*distance/2))
            ts=np.linspace(0,duration,max(2,int(np.ceil(duration/dt))+1))
            if i:ts=ts[1:]
            samples.extend(np.column_stack([np.full(len(ts),i),ts/duration,np.full(len(ts),duration),ts+elapsed]))
            elapsed+=duration
        samples=np.asarray(samples,dtype=np.float32)
        out=self.gpu.run('quintic',len(samples),path[:-1].astype(np.float32),path[1:].astype(np.float32),
                         samples,np.zeros((len(samples),18),dtype=np.float32))
        times=samples[:,3].astype(float);times[-1]=elapsed
        return times,out[:,:6].astype(float),out[:,6:12].astype(float),out[:,12:].astype(float),elapsed
