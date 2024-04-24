import unittest
import sapien


class TestDevice(unittest.TestCase):
    def test_cpu(self):
        cpu = sapien.Device("cpu")
        self.assertFalse(cpu.can_present())
        self.assertFalse(cpu.can_render())
        self.assertEqual(cpu.cuda_id, -1)
        self.assertTrue(cpu.is_cpu())
        self.assertFalse(cpu.is_cuda())

    def test_cuda(self):
        c0 = sapien.Device("cuda")
        c1 = sapien.Device("cuda:0")
        self.assertEqual(c0.cuda_id, 0)
        self.assertEqual(c1.cuda_id, 0)
        self.assertEqual(c0.pci_string, c1.pci_string)
        self.assertIsNotNone(c0.pci_string)
        self.assertFalse(c0.is_cpu())
        self.assertTrue(c1.is_cuda())
        self.assertTrue(c1.can_render())
