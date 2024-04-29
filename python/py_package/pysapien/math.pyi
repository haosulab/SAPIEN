from __future__ import annotations
import numpy
import sapien.pysapien
import typing
__all__ = ['MassProperties', 'compute_box_mass_properties', 'compute_capsule_mass_properties', 'compute_cylinder_mass_properties', 'compute_mesh_mass_properties', 'compute_sphere_mass_properties', 'pose_gl_to_ros', 'pose_ros_to_gl', 'shortest_rotation']
M = typing.TypeVar("M", bound=int)
class MassProperties:
    cm: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float32]]
    cm_inertia: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float32]]
    mass: float
    origin_inertia: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float32]]
    def __add__(self, other: MassProperties) -> MassProperties:
        ...
    @typing.overload
    def __init__(self, mass: float, cm: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float32]] | list | tuple, inertia: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float32]] | list | tuple) -> None:
        """
        construct inertia from mass, center of mass, inertia at center of mass
        """
    @typing.overload
    def __init__(self, mass: float, cmass_local_pose: sapien.pysapien.Pose, inertia: numpy.ndarray[typing.Literal[3], numpy.dtype[numpy.float32]] | list[float] | tuple) -> None:
        """
        construct inertia from mass, cmass_local_pose, and principal inertia
        """
    def decompose(self) -> tuple[float, sapien.pysapien.Pose, numpy.ndarray[typing.Literal[3], numpy.dtype[numpy.float32]]]:
        """
        decompose mass properties into mass, cmass_local_pose, and principal inertia
        """
    def scale_mass(self, scale: float) -> MassProperties:
        """
        compute new mass properties as if  the object density is scaled uniformly
        """
    def scale_size(self, scale: numpy.ndarray[typing.Literal[3], numpy.dtype[numpy.float32]] | list[float] | tuple) -> MassProperties:
        """
        compute new mass properties as if the object volume is scaled around the origin while keeping density the same
        """
    def transform(self, pose: sapien.pysapien.Pose) -> MassProperties:
        """
        compute new mass properties as if the origin of the current object is moved to given pose
        """
def compute_box_mass_properties(half_size: numpy.ndarray[typing.Literal[3], numpy.dtype[numpy.float32]]) -> MassProperties:
    ...
def compute_capsule_mass_properties(radius: float, half_length: float) -> MassProperties:
    ...
def compute_cylinder_mass_properties(radius: float, half_length: float) -> MassProperties:
    ...
def compute_mesh_mass_properties(vertices: numpy.ndarray[tuple[M, typing.Literal[3]], numpy.dtype[numpy.float32]], triangles: numpy.ndarray[numpy.uint32[M, 3]]) -> MassProperties:
    ...
def compute_sphere_mass_properties(radius: float) -> MassProperties:
    ...
def shortest_rotation(source: numpy.ndarray[typing.Literal[3], numpy.dtype[numpy.float32]], target: numpy.ndarray[typing.Literal[3], numpy.dtype[numpy.float32]]) -> numpy.ndarray[typing.Literal[4], numpy.dtype[numpy.float32]]:
    ...
pose_gl_to_ros: sapien.pysapien.Pose  # value = Pose([0, 0, 0], [-0.5, -0.5, 0.5, 0.5])
pose_ros_to_gl: sapien.pysapien.Pose  # value = Pose([0, 0, 0], [-0.5, 0.5, -0.5, -0.5])
