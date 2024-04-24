import unittest

import sapien


class TestGlobals(unittest.TestCase):
    def test_globals(self):
        sapien.pysapien.abi_version()
        sapien.pysapien.compiled_with_cxx11_abi()
        sapien.pysapien.pybind11_internals_id()
        sapien.pysapien.pybind11_use_smart_holder()
        sapien.set_log_level("warning")
