"""Select graphics for this simulation launch without changing system settings."""
import os
import shutil
import subprocess


NVIDIA_ENV = {
    '__NV_PRIME_RENDER_OFFLOAD': '1',
    '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
}
GRAPHICS_OVERRIDES = (
    '__NV_PRIME_RENDER_OFFLOAD', '__NV_PRIME_RENDER_OFFLOAD_PROVIDER',
    '__GLX_VENDOR_LIBRARY_NAME', 'DRI_PRIME', 'LIBGL_ALWAYS_SOFTWARE',
    'MESA_LOADER_DRIVER_OVERRIDE',
)


def select_rendering(backend='auto', environ=None):
    """Return launch-scoped overrides and a reason; probe with a finite timeout.

    auto preserves an explicit graphics environment. Otherwise it selects
    NVIDIA only after an OpenGL context succeeds, rather than relying on the
    presence of nvidia-smi or a driver package. system never probes or changes
    the inherited environment. nvidia is an explicit request and must succeed.
    """
    if backend not in ('auto', 'nvidia', 'system'):
        raise ValueError('render_backend must be auto, nvidia, or system')
    env = dict(os.environ if environ is None else environ)
    if backend == 'system':
        return {}, 'system (inherited graphics selection)'
    if backend == 'auto' and any(key in env for key in GRAPHICS_OVERRIDES):
        return {}, 'system (preserving explicit graphics environment)'

    reason = 'glxinfo is unavailable'
    executable = shutil.which('glxinfo', path=env.get('PATH', os.defpath))
    if executable:
        try:
            result = subprocess.run(
                [executable, '-B'], env={**env, **NVIDIA_ENV},
                capture_output=True, text=True, timeout=5., check=False)
            renderer = next((line.strip() for line in result.stdout.splitlines()
                             if line.strip().startswith('OpenGL renderer string:')), '')
            if result.returncode == 0 and 'nvidia' in renderer.lower():
                return dict(NVIDIA_ENV), renderer
            reason = 'NVIDIA OpenGL context is unavailable'
        except (OSError, subprocess.TimeoutExpired):
            reason = 'NVIDIA OpenGL probe failed or timed out'

    if backend == 'nvidia':
        raise RuntimeError(reason + '; check the display/driver or use render_backend:=system')
    return {}, 'system (' + reason + ')'
