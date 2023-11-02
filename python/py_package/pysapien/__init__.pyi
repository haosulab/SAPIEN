from __future__ import annotations
import sapien.pysapien
import typing
import numpy
_Shape = typing.Tuple[int, ...]
_T = typing.TypeVar("T")

__all__ = [
    "Component",
    "CudaArray",
    "CudaDataSource",
    "Entity",
    "Pose",
    "Scene",
    "System",
    "internal_renderer",
    "math",
    "physx",
    "render",
    "set_log_level",
    "simsense"
]


class Component():
    def __init__(self) -> None: ...
    def disable(self) -> None: 
        """
        disable the component
        """
    def enable(self) -> None: 
        """
        enable the component
        """
    def get_entity(self) -> Entity: ...
    def get_entity_pose(self) -> Pose: ...
    def get_name(self) -> str: ...
    def get_pose(self) -> Pose: ...
    def set_entity_pose(self, arg0: Pose) -> None: ...
    def set_name(self, arg0: str) -> None: ...
    def set_pose(self, arg0: Pose) -> None: ...
    @property
    def _serialization_id(self) -> int:
        """
        :type: int
        """
    @_serialization_id.setter
    def _serialization_id(self, arg1: int) -> None:
        pass
    @property
    def entity(self) -> Entity:
        """
        :type: Entity
        """
    @property
    def entity_pose(self) -> Pose:
        """
        :type: Pose
        """
    @entity_pose.setter
    def entity_pose(self, arg1: Pose) -> None:
        pass
    @property
    def is_enabled(self) -> bool:
        """
        :type: bool
        """
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @name.setter
    def name(self, arg1: str) -> None:
        pass
    @property
    def pose(self) -> Pose:
        """
        :type: Pose
        """
    @pose.setter
    def pose(self, arg1: Pose) -> None:
        pass
    pass
class CudaArray():
    @property
    def __cuda_array_interface__(self) -> dict:
        """
        :type: dict
        """
    @property
    def cuda_id(self) -> int:
        """
        :type: int
        """
    @property
    def ptr(self) -> int:
        """
        :type: int
        """
    @property
    def shape(self) -> list[int]:
        """
        :type: list[int]
        """
    @property
    def strides(self) -> list[int]:
        """
        :type: list[int]
        """
    @property
    def typstr(self) -> str:
        """
        :type: str
        """
    pass
class CudaDataSource():
    def __init__(self) -> None: ...
    pass
class Entity():
    def __init__(self) -> None: ...
    def add_component(self, component: Component) -> Entity: ...
    def add_to_scene(self, scene: Scene) -> Entity: ...
    def find_component_by_type(self, cls: typing.Type[_T]) -> _T: ...
    def get_components(self) -> list[Component]: ...
    def get_global_id(self) -> int: ...
    def get_name(self) -> str: ...
    def get_per_scene_id(self) -> int: ...
    def get_pose(self) -> Pose: ...
    def get_scene(self) -> Scene: ...
    def remove_component(self, component: Component) -> None: ...
    def remove_from_scene(self) -> None: ...
    def set_name(self, arg0: str) -> None: ...
    def set_pose(self, arg0: Pose) -> None: ...
    @property
    def components(self) -> list[Component]:
        """
        :type: list[Component]
        """
    @property
    def global_id(self) -> int:
        """
        :type: int
        """
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @name.setter
    def name(self, arg1: str) -> None:
        pass
    @property
    def per_scene_id(self) -> int:
        """
        :type: int
        """
    @property
    def pose(self) -> Pose:
        """
        :type: Pose
        """
    @pose.setter
    def pose(self, arg1: Pose) -> None:
        pass
    @property
    def scene(self) -> Scene:
        """
        :type: Scene
        """
    pass
class Pose():
    def __getstate__(self) -> tuple: ...
    @typing.overload
    def __init__(self, p: numpy.ndarray[numpy.float32, _Shape, _Shape[3]] = array([0., 0., 0.], dtype=float32), q: numpy.ndarray[numpy.float32, _Shape, _Shape[4]] = array([1., 0., 0., 0.], dtype=float32)) -> None: ...
    @typing.overload
    def __init__(self, arg0: numpy.ndarray[numpy.float32, _Shape[4, 4]]) -> None: ...
    def __mul__(self, arg0: Pose) -> Pose: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, arg0: tuple) -> None: ...
    def get_p(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]: ...
    def get_q(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[4]]: ...
    def inv(self) -> Pose: ...
    def set_p(self, p: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]) -> None: ...
    def set_q(self, q: numpy.ndarray[numpy.float32, _Shape, _Shape[4]]) -> None: ...
    def to_transformation_matrix(self) -> numpy.ndarray[numpy.float32, _Shape[4, 4]]: ...
    @property
    def p(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]
        """
    @p.setter
    def p(self, arg0: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]) -> None:
        pass
    @property
    def q(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[4]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape, _Shape[4]]
        """
    @q.setter
    def q(self, arg0: numpy.ndarray[numpy.float32, _Shape, _Shape[4]]) -> None:
        pass
    pass
class Scene():
    def __getstate__(self) -> bytes: ...
    def __init__(self, systems: list[System]) -> None: ...
    def __setstate__(self, arg0: bytes) -> None: ...
    def _find_all_python_components(self) -> list[Component]: ...
    def _swap_in_python_components(self, arg0: list[Component]) -> None: ...
    def add_entity(self, arg0: Entity) -> None: ...
    def add_system(self, arg0: System) -> None: ...
    def get_entities(self) -> list[Entity]: ...
    def get_physx_system(self) -> physx.PhysxSystem: ...
    def get_render_system(self) -> render.RenderSystem: ...
    def get_system(self, name: str) -> System: ...
    def pack_poses(self) -> bytes: ...
    def remove_entity(self, entity: Entity) -> None: ...
    def unpack_poses(self, data: bytes) -> None: ...
    @property
    def entities(self) -> list[Entity]:
        """
        :type: list[Entity]
        """
    @property
    def physx_system(self) -> physx.PhysxSystem:
        """
        :type: physx.PhysxSystem
        """
    @property
    def render_system(self) -> render.RenderSystem:
        """
        :type: render.RenderSystem
        """
    pass
class System():
    def __init__(self) -> None: ...
    def step(self) -> None: ...
    pass
def set_log_level(level: str) -> None:
    pass
