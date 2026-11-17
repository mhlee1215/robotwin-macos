"""Small synchronous Metal compute bridge; unsupported kernels fail explicitly."""
import ctypes as C
import hashlib
from pathlib import Path
import subprocess
import time
import numpy as np

ROOT = Path(__file__).resolve().parent

class MetalRuntime:
    def __init__(self):
        source = ROOT/'metal/runtime.mm'
        digest = hashlib.sha256(source.read_bytes()).hexdigest()[:16]
        build = ROOT/'metal/build'
        build.mkdir(exist_ok=True)
        library = build/f'runtime-{digest}.dylib'
        if not library.exists():
            import os
            temporary = build/f'{digest}-{os.getpid()}.dylib'
            subprocess.run(['xcrun', 'clang++', '-std=c++17', '-O2', '-fobjc-arc', '-dynamiclib',
                            '-framework', 'Foundation', '-framework', 'Metal', str(source), '-o', str(temporary)], check=True)
            temporary.replace(library)
        self.lib = C.CDLL(str(library))
        self.lib.metal_create.argtypes = [C.c_char_p, C.c_char_p]
        self.lib.metal_create.restype = C.c_void_p
        self.lib.metal_device.argtypes = [C.c_void_p]
        self.lib.metal_device.restype = C.c_char_p
        self.lib.metal_run.argtypes = [C.c_void_p, C.c_char_p, C.c_uint, C.POINTER(C.c_void_p),
                                      C.POINTER(C.c_size_t), C.c_uint, C.POINTER(C.c_double), C.c_char_p]
        self.lib.metal_run.restype = C.c_int
        error = C.create_string_buffer(4096)
        self.handle = self.lib.metal_create((ROOT/'metal/planner.metal').read_bytes(), error)
        if not self.handle:
            raise RuntimeError(error.value.decode())
        self.device = self.lib.metal_device(self.handle).decode()
        self.stats = {}

    def run(self, name, count, *arrays):
        if count < 1:
            raise ValueError('Metal dispatch must be nonempty')
        buffers = [np.ascontiguousarray(a) for a in arrays]
        if any(a.dtype not in (np.dtype('float32'), np.dtype('uint32'), np.dtype('int32')) or a.nbytes==0 for a in buffers):
            raise ValueError('Metal buffers must be nonempty float32/int32/uint32')
        ptrs = (C.c_void_p*len(buffers))(*(a.ctypes.data for a in buffers))
        sizes = (C.c_size_t*len(buffers))(*(a.nbytes for a in buffers))
        error = C.create_string_buffer(4096); gpu_seconds=C.c_double()
        start=time.perf_counter()
        status=self.lib.metal_run(self.handle,name.encode(),count,ptrs,sizes,len(buffers),C.byref(gpu_seconds),error)
        if status:
            raise RuntimeError(error.value.decode())
        stats=self.stats.setdefault(name, {'calls':0,'items':0,'gpu_seconds':0.,'wall_seconds':0.})
        stats['calls']+=1;stats['items']+=count;stats['gpu_seconds']+=gpu_seconds.value
        stats['wall_seconds']+=time.perf_counter()-start
        return buffers[-1]

_runtime = None

def runtime():
    global _runtime
    if _runtime is None:
        _runtime = MetalRuntime()
    return _runtime


def farthest_points(points, count):
    points=np.asarray(points,dtype=np.float32)
    if points.ndim!=2 or points.shape[1]!=3 or not np.isfinite(points).all():
        raise ValueError('FPS requires finite XYZ points')
    count=min(int(count),len(points))
    if count<=0:return np.empty(0,dtype=np.int64)
    result=runtime().run('farthest_points',256,points,np.array([len(points),count],dtype=np.uint32),
                         np.zeros(len(points),dtype=np.float32),np.zeros(count,dtype=np.uint32))
    return result.astype(np.int64)
