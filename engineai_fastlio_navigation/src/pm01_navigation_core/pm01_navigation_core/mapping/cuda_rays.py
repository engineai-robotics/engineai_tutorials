"""Optional CUDA driver/NVRTC backend. No CUDA imports are required on CPU hosts."""
import ctypes as C
from contextlib import contextmanager
from pathlib import Path

import numpy as np

P, I, U, Z, D = C.c_void_p, C.c_int, C.c_uint, C.c_size_t, C.c_uint64
ref = C.byref


class CudaError(RuntimeError):
    pass


def call(lib, name, *args):
    result = getattr(lib, name)(*args)
    if result:
        raise CudaError(f'{name}: {result}')


def compile_kernel(major, minor):
    path = Path('/opt/pm01_cuda/libnvrtc.so.12')
    nv = C.CDLL(str(path) if path.is_file() else 'libnvrtc.so.12')
    if path.is_file():
        major_version, minor_version = I(), I()
        call(nv, 'nvrtcVersion', ref(major_version), ref(minor_version))
        # NVRTC dlopens its builtins by SONAME. Resolve the read-only toolkit
        # mount without overriding the ROS process's library search path.
        builtins = C.CDLL(str(path.with_name(
            f'libnvrtc-builtins.so.{major_version.value}.{minor_version.value}')))
    source = Path(__file__).with_name('ray_samples.cuh').read_bytes()
    program = P()
    call(nv, 'nvrtcCreateProgram', ref(program), source, b'pm01.cu', 0, None, None)
    try:
        flags = (C.c_char_p*3)(f'--gpu-architecture=compute_{major}{minor}'.encode(),
                                b'--fmad=false', b'--std=c++14')
        if nv.nvrtcCompileProgram(program, len(flags), flags):
            size = Z()
            call(nv, 'nvrtcGetProgramLogSize', program, ref(size))
            log = C.create_string_buffer(size.value)
            call(nv, 'nvrtcGetProgramLog', program, log)
            raise CudaError(log.value.decode(errors='replace').strip()[:1000])
        size = Z()
        call(nv, 'nvrtcGetPTXSize', program, ref(size))
        text = C.create_string_buffer(size.value)
        call(nv, 'nvrtcGetPTX', program, text)
        return text
    finally:
        nv.nvrtcDestroyProgram(ref(program))


class CudaRays:
    """Used by the serialized map worker. Bind its context for every calling thread."""
    def __init__(self):
        self.driver = C.CDLL('libcuda.so.1')
        self.ctx, self.mod, self.fn, self.stream, self.done = P(), P(), P(), P(), P()
        self.buffers = {}
        device = I()
        self.call('cuInit', 0)
        self.call('cuDeviceGet', ref(device), 0)
        self.device = device
        major, minor = I(), I()
        self.call('cuDeviceComputeCapability', ref(major), ref(minor), device)
        text = compile_kernel(major.value, minor.value)
        self.call('cuDevicePrimaryCtxRetain', ref(self.ctx), device)
        try:
            with self.context():
                self.call('cuModuleLoadData', ref(self.mod), text)
                self.call('cuModuleGetFunction', ref(self.fn), self.mod, b'pm01_trace')
                self.call('cuStreamCreate', ref(self.stream), 1)
                self.call('cuEventCreate', ref(self.done), 3)  # blocking wait, no timing
        except Exception:
            self.close()
            raise

    def call(self, name, *args):
        call(self.driver, name, *args)

    @contextmanager
    def context(self):
        self.call('cuCtxPushCurrent_v2', self.ctx)
        try:
            yield
        finally:
            self.call('cuCtxPopCurrent_v2', ref(P()))

    def buffer(self, key, size):
        entry = self.buffers.get(key)
        if entry is None or entry[1] < size:
            ptr = D()
            self.call('cuMemAlloc_v2', ref(ptr), Z(size))
            if entry:
                self.driver.cuMemFree_v2(entry[0])
            entry = self.buffers[key] = ptr, size
        return entry[0]

    def trace(self, rays, counts, geometry, w, h, bins):
        sizes = [w*h, w*h*bins]
        packed = [np.empty((n+31)//32, dtype=np.uint32) for n in sizes]
        with self.context():
            args = []
            for key, a in enumerate((rays, counts, geometry)):
                addr = self.buffer(key, a.nbytes)
                self.call('cuMemcpyHtoD_v2', addr, P(a.ctypes.data), Z(a.nbytes))
                args.append(addr)
            for key, a in enumerate(packed, 3):
                addr = self.buffer(key, a.nbytes)
                self.call('cuMemsetD32Async', addr, U(0), Z(a.size), self.stream)
                args.append(addr)
            args.extend(map(I, (len(counts), w, h, bins)))
            argv = (P*len(args))(*(C.addressof(arg) for arg in args))
            self.call('cuLaunchKernel', self.fn, U((len(counts)+127)//128), U(1), U(1),
                      U(128), U(1), U(1), U(0), self.stream, argv, None)
            self.call('cuEventRecord', self.done, self.stream)
            self.call('cuEventSynchronize', self.done)
            for a, addr in zip(packed, args[3:5]):
                self.call('cuMemcpyDtoH_v2', P(a.ctypes.data), addr, Z(a.nbytes))
        return tuple(np.unpackbits(a.view(np.uint8), count=n, bitorder='little').view(np.bool_)
                     for a, n in zip(packed, sizes))

    def close(self):
        if not self.ctx:
            return
        try:
            with self.context():
                for addr, _ in self.buffers.values():
                    self.driver.cuMemFree_v2(addr)
                if self.done:
                    self.driver.cuEventDestroy_v2(self.done)
                if self.stream:
                    self.driver.cuStreamDestroy_v2(self.stream)
                if self.mod:
                    self.driver.cuModuleUnload(self.mod)
        finally:
            self.driver.cuDevicePrimaryCtxRelease_v2(self.device)
            self.ctx = P()
