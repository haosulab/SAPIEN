"""Bundled GPU library selection does not require CUDA or a download."""

import ctypes
import os
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

import sapien.physx as physx


class TestGpuLoader(unittest.TestCase):
    def test_bundled_library_and_explicit_override(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            package = root / "sapien"
            package.mkdir()
            bundled = package / "libPhysXGpu_64.so"
            bundled.touch()
            override = root / "override.so"
            override.touch()
            # A cached library must not shadow the wheel's matching runtime.
            cache = root / ".sapien" / "physx" / "test-version"
            cache.mkdir(parents=True)
            (cache / bundled.name).touch()
            for setting, expected in [("", bundled), (str(override), override)]:
                with (
                    self.subTest(override=setting),
                    patch.object(physx, "__file__", str(package / "physx" / "__init__.py")),
                    patch.object(physx, "is_gpu_enabled", return_value=False),
                    patch.object(physx, "version", return_value="test-version"),
                    patch.object(physx, "_enable_gpu") as enable,
                    patch.object(Path, "home", return_value=root),
                    patch("platform.system", return_value="Linux"),
                    patch.dict(os.environ, {"SAPIEN_PHYSX_GPU_LIBRARY": setting}),
                    patch("ctypes.CDLL") as load,
                    patch("requests.get") as download,
                ):
                    physx.enable_gpu()
                    load.assert_any_call(str(expected), ctypes.RTLD_LOCAL)
                    download.assert_not_called()
                    enable.assert_called_once_with()
