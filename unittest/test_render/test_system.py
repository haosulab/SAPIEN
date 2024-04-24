import unittest
from pathlib import Path
import numpy as np

import sapien


class TestSystem(unittest.TestCase):
    def test_device(self):
        system = sapien.render.RenderSystem("cuda")
        self.assertEqual(system.device.cuda_id, 0)

        system = sapien.render.RenderSystem(sapien.Device("cuda"))
        self.assertEqual(system.device.cuda_id, 0)

    def test_light(self):
        system = sapien.render.RenderSystem()
        system.ambient_light = [0.1, 0.2, 0.3]
        self.assertTrue(np.allclose(system.ambient_light, [0.1, 0.2, 0.3]))
        system.set_ambient_light([0.2, 0.3, 0.4])
        self.assertTrue(np.allclose(system.get_ambient_light(), [0.2, 0.3, 0.4]))

    # TODO: test cmaeras, bodies, lights
