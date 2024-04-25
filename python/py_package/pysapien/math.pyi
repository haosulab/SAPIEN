from __future__ import annotations
import numpy
import sapien.pysapien
import typing
__all__ = ['MassProperties', 'compute_mesh_mass_properties', 'pose_gl_to_ros', 'pose_ros_to_gl', 'shortest_rotation']
M = typing.TypeVar("M", bound=int)
class MassProperties:
    cm: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float32]]
    mass: float
    def decompose(self) -> tuple[float, sapien.pysapien.Pose, numpy.ndarray[typing.Literal[3], numpy.dtype[numpy.float32]]]:
        ...
    def scale_mass(self, scale: float) -> MassProperties:
        ...
    def scale_size(self, scale: float) -> MassProperties:
        ...
    def transform(self, pose: sapien.pysapien.Pose) -> MassProperties:
        ...
    @property
    def cm_inertia(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float32]]:
        ...
    @property
    def origin_inertia(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float32]]:
        ...
def compute_mesh_mass_properties(arg0: numpy.ndarray[tuple[M, typing.Literal[3]], numpy.dtype[numpy.float32]], arg1: numpy.ndarray[numpy.uint32[M, 3]]) -> MassProperties:
    ...
def shortest_rotation(source: numpy.ndarray[typing.Literal[3], numpy.dtype[numpy.float32]], target: numpy.ndarray[typing.Literal[3], numpy.dtype[numpy.float32]]) -> numpy.ndarray[typing.Literal[4], numpy.dtype[numpy.float32]]:
    ...
pose_gl_to_ros: sapien.pysapien.Pose  # value = Pose([0, 0, 0], [-0.5, -0.5, 0.5, 0.5])
pose_ros_to_gl: sapien.pysapien.Pose  # value = Pose([0, 0, 0], [-0.5, 0.5, -0.5, -0.5])
