from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import subprocess
from types import SimpleNamespace
import unittest
from unittest.mock import patch


spec = spec_from_file_location('pm01_sim_rendering',
    Path(__file__).resolve().parents[1]/'launch/sim/rendering.py')
rendering = module_from_spec(spec)
spec.loader.exec_module(rendering)


class RenderingSelection(unittest.TestCase):
    def probe(self, output='OpenGL renderer string: NVIDIA GeForce RTX', code=0):
        return SimpleNamespace(returncode=code, stdout=output)

    def test_auto_selects_verified_nvidia_without_mutating_environment(self):
        env = {'PATH': '/usr/bin', 'DISPLAY': ':1'}
        with patch.object(rendering.shutil, 'which', return_value='/usr/bin/glxinfo'), \
                patch.object(rendering.subprocess, 'run', return_value=self.probe()) as run:
            overrides, reason = rendering.select_rendering('auto', env)
        self.assertEqual(overrides, rendering.NVIDIA_ENV)
        self.assertIn('NVIDIA', reason)
        self.assertEqual(env, {'PATH': '/usr/bin', 'DISPLAY': ':1'})
        self.assertEqual(run.call_args.kwargs['env']['DISPLAY'], ':1')
        self.assertEqual(run.call_args.kwargs['env']['__GLX_VENDOR_LIBRARY_NAME'], 'nvidia')
        self.assertEqual(run.call_args.kwargs['timeout'], 5.)
        self.assertNotIn('shell', run.call_args.kwargs)

    def test_auto_does_not_accept_a_non_nvidia_or_failed_context(self):
        for output, code in [('OpenGL renderer string: Mesa Intel', 0),
                             ('OpenGL renderer string: NVIDIA RTX', 1)]:
            with self.subTest(output=output, code=code), \
                    patch.object(rendering.shutil, 'which', return_value='/usr/bin/glxinfo'), \
                    patch.object(rendering.subprocess, 'run', return_value=self.probe(output, code)):
                self.assertEqual(rendering.select_rendering('auto', {})[0], {})

    def test_auto_falls_back_if_probe_is_missing_or_unavailable(self):
        with patch.object(rendering.shutil, 'which', return_value=None):
            self.assertEqual(rendering.select_rendering('auto', {})[0], {})
        for failure in [OSError('no display'), subprocess.TimeoutExpired('glxinfo', 5)]:
            with self.subTest(failure=type(failure).__name__), \
                    patch.object(rendering.shutil, 'which', return_value='/usr/bin/glxinfo'), \
                    patch.object(rendering.subprocess, 'run', side_effect=failure):
                self.assertEqual(rendering.select_rendering('auto', {})[0], {})

    def test_auto_preserves_each_explicit_graphics_override(self):
        with patch.object(rendering.shutil, 'which') as which:
            for key in rendering.GRAPHICS_OVERRIDES:
                self.assertEqual(rendering.select_rendering('auto', {key: 'custom'})[0], {})
            which.assert_not_called()

    def test_system_does_not_probe(self):
        with patch.object(rendering.shutil, 'which') as which:
            self.assertEqual(rendering.select_rendering('system', {})[0], {})
            which.assert_not_called()

    def test_explicit_nvidia_can_override_inherited_selection(self):
        with patch.object(rendering.shutil, 'which', return_value='/usr/bin/glxinfo'), \
                patch.object(rendering.subprocess, 'run', return_value=self.probe()):
            self.assertEqual(rendering.select_rendering('nvidia',
                {'__GLX_VENDOR_LIBRARY_NAME': 'mesa'})[0], rendering.NVIDIA_ENV)

    def test_explicit_nvidia_reports_unavailable_instead_of_silent_fallback(self):
        with patch.object(rendering.shutil, 'which', return_value=None):
            with self.assertRaisesRegex(RuntimeError, 'render_backend:=system'):
                rendering.select_rendering('nvidia', {})

    def test_invalid_backend_is_rejected(self):
        with self.assertRaises(ValueError):
            rendering.select_rendering('cuda', {})


if __name__ == '__main__':
    unittest.main()
