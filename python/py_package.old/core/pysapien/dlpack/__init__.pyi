from __future__ import annotations
import sapien.core.pysapien.dlpack
import typing
import numpy
_Shape = typing.Tuple[int, ...]

__all__ = [
    "dl_cuda_sync",
    "dl_ptr",
    "dl_shape",
    "dl_to_numpy_cuda_async_unchecked"
]


def dl_cuda_sync() -> None:
    pass
def dl_ptr(arg0: capsule) -> int:
    pass
def dl_shape(arg0: capsule) -> typing.List[int]:
    pass
def dl_to_numpy_cuda_async_unchecked(dl_tensor: capsule, result: numpy.ndarray[numpy.float32]) -> None:
    pass
