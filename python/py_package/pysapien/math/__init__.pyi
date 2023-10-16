from __future__ import annotations
import sapien.pysapien.math
import typing
import numpy
_Shape = typing.Tuple[int, ...]
_T = typing.TypeVar("T")

__all__ = [
    "shortest_rotation"
]


def shortest_rotation(source: numpy.ndarray[numpy.float32, _Shape, _Shape[3]], target: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]) -> numpy.ndarray[numpy.float32, _Shape, _Shape[4]]:
    pass
