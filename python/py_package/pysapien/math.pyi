from __future__ import annotations
import numpy
import numpy.typing
import sapien.pysapien
import typing
__all__ = ['MassProperties', 'compute_box_mass_properties', 'compute_capsule_mass_properties', 'compute_cylinder_mass_properties', 'compute_mesh_mass_properties', 'compute_sphere_mass_properties', 'pose_gl_to_ros', 'pose_ros_to_gl', 'shortest_rotation']
class MassProperties:
    mass: float
    @staticmethod
    def _pybind11_conduit_v1_(*args, **kwargs):
        ...
    def __add__(self, other: MassProperties) -> MassProperties:
        ...
    @typing.overload
    def __init__(self, mass: float, cm: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"], inertia: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 3]"]) -> None:
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
    @property
    def cm(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[3, 1]"]:
        ...
    @cm.setter
    def cm(self, arg0: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]) -> None:
        ...
    @property
    def cm_inertia(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[3, 3]"]:
        ...
    @cm_inertia.setter
    def cm_inertia(self, arg1: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 3]"]) -> None:
        ...
    @property
    def origin_inertia(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[3, 3]"]:
        ...
    @origin_inertia.setter
    def origin_inertia(self, arg1: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 3]"]) -> None:
        ...
def compute_box_mass_properties(half_size: numpy.ndarray[typing.Literal[3], numpy.dtype[numpy.float32]]) -> MassProperties:
    ...
def compute_capsule_mass_properties(radius: float, half_length: float) -> MassProperties:
    ...
def compute_cylinder_mass_properties(radius: float, half_length: float) -> MassProperties:
    ...
def compute_mesh_mass_properties(vertices: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, 3]"], triangles: typing.Annotated[numpy.typing.ArrayLike, numpy.uint32, "[m, 3]"]) -> MassProperties:
    ...
def compute_sphere_mass_properties(radius: float) -> MassProperties:
    ...
def shortest_rotation(source: numpy.ndarray[typing.Literal[3], numpy.dtype[numpy.float32]], target: numpy.ndarray[typing.Literal[3], numpy.dtype[numpy.float32]]) -> numpy.ndarray[typing.Literal[4], numpy.dtype[numpy.float32]]:
    ...
pose_gl_to_ros: sapien.pysapien.Pose  # value = Pose([0, 0, 0], [-0.5, -0.5, 0.5, 0.5])
pose_ros_to_gl: sapien.pysapien.Pose  # value = Pose([0, 0, 0], [-0.5, 0.5, -0.5, -0.5])
