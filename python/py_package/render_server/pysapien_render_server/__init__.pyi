from __future__ import annotations
import sapien.render_server.pysapien_render_server
import typing
import numpy
import sapien.pysapien
_Shape = typing.Tuple[int, ...]
_T = typing.TypeVar("T")

__all__ = [
    "RenderClientBodyComponent",
    "RenderClientCameraComponent",
    "RenderClientShape",
    "RenderClientShapeTriangleMesh",
    "RenderClientSystem",
    "RenderServer",
    "RenderServerBuffer"
]


class RenderClientBodyComponent(sapien.pysapien.Component):
    def __init__(self) -> None: ...
    def attach(self, arg0: RenderClientShape) -> RenderClientBodyComponent: ...
    pass
class RenderClientCameraComponent(sapien.pysapien.Component):
    def __init__(self, width: int, height: int, shader_dir: str = '') -> None: ...
    def get_local_pose(self) -> sapien.pysapien.Pose: ...
    def set_local_pose(self, pose: sapien.pysapien.Pose) -> None: ...
    def set_perspective_parameters(self, near: float, far: float, fx: float, fy: float, cx: float, cy: float, skew: float) -> None: ...
    def take_picture(self) -> None: ...
    @property
    def local_pose(self) -> sapien.pysapien.Pose:
        """
        :type: sapien.pysapien.Pose
        """
    @local_pose.setter
    def local_pose(self, arg1: sapien.pysapien.Pose) -> None:
        pass
    pass
class RenderClientShape():
    def get_local_pose(self) -> sapien.pysapien.Pose: ...
    def get_per_scene_id(self) -> int: ...
    def set_local_pose(self, pose: sapien.pysapien.Pose) -> None: ...
    @property
    def local_pose(self) -> sapien.pysapien.Pose:
        """
        :type: sapien.pysapien.Pose
        """
    @local_pose.setter
    def local_pose(self, arg1: sapien.pysapien.Pose) -> None:
        pass
    @property
    def per_scene_id(self) -> int:
        """
        :type: int
        """
    pass
class RenderClientShapeTriangleMesh(RenderClientShape):
    def __init__(self, filename: str, scale: numpy.ndarray[numpy.float32, _Shape, _Shape[3]] = array([1., 1., 1.], dtype=float32)) -> None: ...
    def get_filename(self) -> str: ...
    def get_scale(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]: ...
    @property
    def filename(self) -> str:
        """
        :type: str
        """
    @property
    def scale(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]
        """
    pass
class RenderClientSystem():
    def __init__(self, address: str, process_index: int) -> None: ...
    def add_point_light(self, position: numpy.ndarray[numpy.float32, _Shape, _Shape[3]], color: numpy.ndarray[numpy.float32, _Shape, _Shape[3]], shadow: bool = False, shadow_near: float = 0.009999999776482582, shadow_far: float = 100.0, shadow_map_size: int = 1024) -> None: ...
    def get_process_index(self) -> int: ...
    @typing.overload
    def set_ambient_light(self, color: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]) -> None: ...
    @typing.overload
    def set_ambient_light(self, direction: numpy.ndarray[numpy.float32, _Shape, _Shape[3]], color: numpy.ndarray[numpy.float32, _Shape, _Shape[3]], shadow: bool = False, position: numpy.ndarray[numpy.float32, _Shape, _Shape[3]] = array([0., 0., 0.], dtype=float32), shadow_scale: float = 5.0, shadow_near: float = 0.009999999776482582, shadow_far: float = 100.0, shadow_map_size: int = 1024) -> None: ...
    def update_render_and_take_pictures(self, arg0: list[RenderClientCameraComponent]) -> None: ...
    @property
    def process_index(self) -> int:
        """
        :type: int
        """
    pass
class RenderServer():
    def __init__(self, max_num_materials: int = 500, max_num_textures: int = 500, default_mipmap_levels: int = 1, device: str = 'cuda', do_not_load_texture: bool = False) -> None: ...
    @staticmethod
    def _set_shader_dir(shader_dir: str) -> None: ...
    def auto_allocate_buffers(self, render_targets: list[str]) -> list[RenderServerBuffer]: ...
    def start(self, address: str) -> None: ...
    def stop(self) -> None: ...
    def summary(self) -> str: ...
    def wait_all(self, timeout: int = 18446744073709551615) -> bool: ...
    def wait_scenes(self, scenes: list[int], timeout: int = 18446744073709551615) -> bool: ...
    pass
class RenderServerBuffer():
    @property
    def __cuda_array_interface__(self) -> dict:
        """
        :type: dict
        """
    @property
    def nbytes(self) -> int:
        """
        :type: int
        """
    @property
    def pointer(self) -> int:
        """
        :type: int
        """
    @property
    def shape(self) -> tuple:
        """
        :type: tuple
        """
    @property
    def type(self) -> str:
        """
        :type: str
        """
    pass
