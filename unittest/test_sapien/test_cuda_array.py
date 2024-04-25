import unittest
import sapien


class TestCudaArray(unittest.TestCase):
    def test_torch(self):
        import torch

        tensor = torch.tensor([[0, 1, 2], [2, 3, 4]]).float().cuda()
        array = sapien.CudaArray(tensor)

        self.assertEqual(
            array.typestr,
            array.__cuda_array_interface__["typestr"],
        )
        self.assertEqual(
            tuple(array.shape),
            array.__cuda_array_interface__["shape"],
        )
        self.assertEqual(
            tuple(array.strides),
            array.__cuda_array_interface__["strides"],
        )
        self.assertEqual(tuple(array.strides), (12, 4))
        self.assertEqual(
            array.ptr,
            array.__cuda_array_interface__["data"][0],
        )

        self.assertEqual(
            tensor.__cuda_array_interface__["typestr"],
            array.__cuda_array_interface__["typestr"],
        )
        self.assertEqual(
            tensor.__cuda_array_interface__["shape"],
            array.__cuda_array_interface__["shape"],
        )
        self.assertEqual(
            tensor.__cuda_array_interface__["data"],
            array.__cuda_array_interface__["data"],
        )

        self.assertEqual(
            array.torch().__cuda_array_interface__["data"],
            array.__cuda_array_interface__["data"],
        )

    def test_slice(self):
        import torch

        tensor = torch.tensor([[0, 1, 2], [2, 3, 4], [3, 4, 5]]).float().cuda()
        tensor = tensor[1:, :-1]
        array = sapien.CudaArray(tensor)

        self.assertEqual(
            array.typestr,
            array.__cuda_array_interface__["typestr"],
        )
        self.assertEqual(
            tuple(array.shape),
            array.__cuda_array_interface__["shape"],
        )
        self.assertEqual(
            tuple(array.strides),
            array.__cuda_array_interface__["strides"],
        )
        self.assertEqual(
            array.ptr,
            array.__cuda_array_interface__["data"][0],
        )

        self.assertEqual(
            tensor.__cuda_array_interface__["typestr"],
            array.__cuda_array_interface__["typestr"],
        )
        self.assertEqual(
            tensor.__cuda_array_interface__["shape"],
            array.__cuda_array_interface__["shape"],
        )
        self.assertEqual(
            tensor.__cuda_array_interface__["data"],
            array.__cuda_array_interface__["data"],
        )
        self.assertEqual(
            tensor.__cuda_array_interface__["strides"],
            array.__cuda_array_interface__["strides"],
        )

        self.assertEqual(
            array.torch().__cuda_array_interface__["data"],
            array.__cuda_array_interface__["data"],
        )
